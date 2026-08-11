#include "mem_debug.h"

#if MEM_DEBUG

#include "debug.h"
#include <new>
#include <cstring>
#include <cstddef>
#include <cstdint>

struct _reent;

static constexpr size_t kDebugHeapSize = 16U * 1024U;
static constexpr uint32_t kDebugHeapMagic = 0xDEADBEEFu;
static constexpr uint32_t kDebugFenceValue = 0xF1F1F1F1u;
static constexpr size_t kFenceSize = sizeof(uint32_t);
static constexpr size_t kMinBlockSize = 32U;
static constexpr uint8_t kAllocPattern = 0xCD;
static constexpr uint8_t kFreePattern = 0xDD;

static uint8_t debug_heap_buffer[kDebugHeapSize] __attribute__((aligned(alignof(std::max_align_t))));
static bool debug_heap_initialized = false;

struct alignas(std::max_align_t) BlockHeader {
    uint32_t magic;
    size_t size;
    BlockHeader *prev;
    BlockHeader *next;
    bool free;
    uint8_t reserved[3];
    uint32_t front_fence;
};

static constexpr size_t align_up(size_t size)
{
    constexpr size_t alignment = alignof(std::max_align_t);
    return (size + (alignment - 1U)) & ~(alignment - 1U);
}

static constexpr size_t kHeaderSize = align_up(sizeof(BlockHeader));

static BlockHeader *debug_heap_head = nullptr;

static inline uint8_t *payload_ptr(BlockHeader *hdr)
{
    return reinterpret_cast<uint8_t *>(hdr + 1);
}

static inline const uint8_t *payload_ptr(const BlockHeader *hdr)
{
    return reinterpret_cast<const uint8_t *>(hdr + 1);
}

static inline uint8_t *tail_fence_ptr(BlockHeader *hdr)
{
    return payload_ptr(hdr) + hdr->size;
}

static inline const uint8_t *tail_fence_ptr(const BlockHeader *hdr)
{
    return payload_ptr(hdr) + hdr->size;
}

static inline bool ptr_in_heap(const void *ptr)
{
    const uint8_t *p = static_cast<const uint8_t *>(ptr);
    return p >= debug_heap_buffer && p < (debug_heap_buffer + kDebugHeapSize);
}

static void debug_heap_init(void)
{
    if (debug_heap_initialized) {
        return;
    }

    debug_heap_head = reinterpret_cast<BlockHeader *>(debug_heap_buffer);
    debug_heap_head->magic = kDebugHeapMagic;
    debug_heap_head->size = kDebugHeapSize - kHeaderSize - kFenceSize;
    debug_heap_head->prev = nullptr;
    debug_heap_head->next = nullptr;
    debug_heap_head->free = true;
    debug_heap_head->front_fence = kDebugFenceValue;
    debug_heap_initialized = true;

    *reinterpret_cast<uint32_t *>(tail_fence_ptr(debug_heap_head)) = kDebugFenceValue;
    std::memset(payload_ptr(debug_heap_head), kFreePattern, debug_heap_head->size);
}

static bool is_valid_header(BlockHeader *hdr)
{
    if (!hdr) {
        return false;
    }
    if (!ptr_in_heap(hdr)) {
        return false;
    }
    if (hdr->magic != kDebugHeapMagic) {
        return false;
    }
    if (hdr->size > (kDebugHeapSize - kHeaderSize - kFenceSize)) {
        return false;
    }
    const uint8_t *start = debug_heap_buffer;
    const uint8_t *end = debug_heap_buffer + kDebugHeapSize;
    const uint8_t *payload_start = payload_ptr(hdr);
    const uint8_t *fence = tail_fence_ptr(hdr);
    if (payload_start < start || fence + kFenceSize > end) {
        return false;
    }
    return true;
}

static void set_fence(BlockHeader *hdr)
{
    hdr->front_fence = kDebugFenceValue;
    *reinterpret_cast<uint32_t *>(tail_fence_ptr(hdr)) = kDebugFenceValue;
}

static bool check_fence(BlockHeader *hdr)
{
    if (hdr->front_fence != kDebugFenceValue) {
        return false;
    }
    return *reinterpret_cast<uint32_t *>(tail_fence_ptr(hdr)) == kDebugFenceValue;
}

static BlockHeader *find_header_by_payload(const void *ptr)
{
    if (!ptr_in_heap(ptr)) {
        return nullptr;
    }

    BlockHeader *current = debug_heap_head;
    while (current) {
        if (!is_valid_header(current)) {
            return nullptr;
        }
        if (payload_ptr(current) == ptr) {
            return current;
        }
        current = current->next;
    }
    return nullptr;
}

static void coalesce_block(BlockHeader *hdr)
{
    if (hdr->next && hdr->next->free) {
        hdr->size += kHeaderSize + hdr->next->size + kFenceSize;
        hdr->next = hdr->next->next;
        if (hdr->next) {
            hdr->next->prev = hdr;
        }
    }

    if (hdr->prev && hdr->prev->free) {
        hdr = hdr->prev;
        hdr->size += kHeaderSize + hdr->next->size + kFenceSize;
        hdr->next = hdr->next->next;
        if (hdr->next) {
            hdr->next->prev = hdr;
        }
    }

    set_fence(hdr);
}

static BlockHeader *find_free_block(size_t size)
{
    BlockHeader *current = debug_heap_head;
    while (current) {
        if (current->free && current->size >= size) {
            return current;
        }
        current = current->next;
    }
    return nullptr;
}

static void split_block(BlockHeader *hdr, size_t size)
{
    size_t available = hdr->size;
    if (available < size + kHeaderSize + kFenceSize + kMinBlockSize) {
        return;
    }

    uint8_t *payload = reinterpret_cast<uint8_t *>(hdr + 1);
    uint8_t *next_header_address = payload + size + kFenceSize;
    BlockHeader *next_header = reinterpret_cast<BlockHeader *>(next_header_address);

    next_header->magic = kDebugHeapMagic;
    next_header->size = available - size - kHeaderSize - kFenceSize;
    next_header->free = true;
    next_header->prev = hdr;
    next_header->next = hdr->next;
    next_header->front_fence = kDebugFenceValue;
    if (next_header->next) {
        next_header->next->prev = next_header;
    }

    hdr->size = size;
    hdr->next = next_header;
    set_fence(next_header);
}

void *debug_malloc(size_t size)
{
    if (!size) {
        size = 1;
    }

    debug_heap_init();
    size = align_up(size);

    BlockHeader *hdr = find_free_block(size);
    if (!hdr) {
        DEBUG_PRINT(DebugType::MEM, "debug_malloc OOM size=%u", static_cast<unsigned>(size));
        return nullptr;
    }

    split_block(hdr, size);
    hdr->free = false;
    set_fence(hdr);
    std::memset(payload_ptr(hdr), kAllocPattern, hdr->size);

    return reinterpret_cast<void *>(hdr + 1);
}

void debug_free(void *ptr)
{
    if (!ptr) {
        return;
    }

    BlockHeader *hdr = find_header_by_payload(ptr);
    if (!is_valid_header(hdr)) {
        DEBUG_PRINT(DebugType::MEM, "debug_free invalid pointer %p", ptr);
        return;
    }

    if (hdr->free) {
        DEBUG_PRINT(DebugType::MEM, "debug_free double free %p", ptr);
        return;
    }

    if (!check_fence(hdr)) {
        DEBUG_PRINT(DebugType::MEM, "debug_free fence corrupted %p", ptr);
        debug_heap_dump();
    }

    hdr->free = true;
    std::memset(payload_ptr(hdr), kFreePattern, hdr->size);
    set_fence(hdr);
    coalesce_block(hdr);
}

void *debug_realloc(void *ptr, size_t size)
{
    if (!ptr) {
        return debug_malloc(size);
    }

    if (!size) {
        debug_free(ptr);
        return nullptr;
    }

    BlockHeader *hdr = find_header_by_payload(ptr);
    if (!is_valid_header(hdr)) {
        DEBUG_PRINT(DebugType::MEM, "debug_realloc invalid pointer %p", ptr);
        return nullptr;
    }

    if (!check_fence(hdr)) {
        DEBUG_PRINT(DebugType::MEM, "debug_realloc fence corrupted %p", ptr);
        debug_heap_dump();
    }

    size = align_up(size);
    if (hdr->size >= size) {
        split_block(hdr, size);
        set_fence(hdr);
        return ptr;
    }

    void *new_ptr = debug_malloc(size);
    if (!new_ptr) {
        DEBUG_PRINT(DebugType::MEM, "debug_realloc failed size=%u", static_cast<unsigned>(size));
        return nullptr;
    }

    std::memcpy(new_ptr, ptr, hdr->size);
    debug_free(ptr);
    return new_ptr;
}

void *debug_calloc(size_t nmemb, size_t size)
{
    size_t total = nmemb * size;
    void *ptr = debug_malloc(total);
    if (!ptr) {
        DEBUG_PRINT(DebugType::MEM, "debug_calloc failed nmemb=%u size=%u", static_cast<unsigned>(nmemb), static_cast<unsigned>(size));
        return nullptr;
    }
    std::memset(ptr, 0, total);
    return ptr;
}

bool debug_heap_check(void)
{
    if (!debug_heap_initialized) {
        return true;
    }

    const uint8_t *last_addr = debug_heap_buffer;
    BlockHeader *current = debug_heap_head;
    while (current) {
        if (!is_valid_header(current)) {
            DEBUG_PRINT(DebugType::MEM, "debug_heap_check invalid block %p", current);
            return false;
        }

        const uint8_t *current_addr = reinterpret_cast<const uint8_t *>(current);
        if (current_addr < last_addr) {
            DEBUG_PRINT(DebugType::MEM, "debug_heap_check out-of-order block %p", current);
            return false;
        }

        if (current->next && current->next->prev != current) {
            DEBUG_PRINT(DebugType::MEM, "debug_heap_check broken link current=%p next=%p", current, current->next);
            return false;
        }

        if (!check_fence(current)) {
            DEBUG_PRINT(DebugType::MEM, "debug_heap_check fence corrupted %p", payload_ptr(current));
            debug_heap_dump();
            return false;
        }

        last_addr = current_addr;
        current = current->next;
    }
    return true;
}

void debug_heap_dump(void)
{
    if (!debug_heap_initialized) {
        DEBUG_PRINT(DebugType::MEM, "debug_heap_dump empty");
        return;
    }

    DEBUG_PRINT(DebugType::MEM, "debug_heap_dump start");
    BlockHeader *current = debug_heap_head;
    while (current) {
        uintptr_t payload_addr = reinterpret_cast<uintptr_t>(current + 1);
        DEBUG_PRINT(DebugType::MEM, "block=%p size=%u free=%u next=%p", reinterpret_cast<void *>(payload_addr), static_cast<unsigned>(current->size), static_cast<unsigned>(current->free), reinterpret_cast<void *>(current->next));
        current = current->next;
    }
    DEBUG_PRINT(DebugType::MEM, "debug_heap_dump end");
}

extern "C" void *malloc(size_t size)
{
    return debug_malloc(size);
}

extern "C" void *_malloc_r(struct _reent *, size_t size)
{
    return debug_malloc(size);
}

extern "C" void free(void *ptr)
{
    debug_free(ptr);
}

extern "C" void _free_r(struct _reent *, void *ptr)
{
    debug_free(ptr);
}

extern "C" void *realloc(void *ptr, size_t size)
{
    return debug_realloc(ptr, size);
}

extern "C" void *_realloc_r(struct _reent *, void *ptr, size_t size)
{
    return debug_realloc(ptr, size);
}

extern "C" void *calloc(size_t nmemb, size_t size)
{
    return debug_calloc(nmemb, size);
}

extern "C" void *_calloc_r(struct _reent *, size_t nmemb, size_t size)
{
    return debug_calloc(nmemb, size);
}

void *operator new(std::size_t size) noexcept
{
    void *ptr = debug_malloc(size);
    if (!ptr) {
        DEBUG_PRINT(DebugType::MEM, "operator new failed size=%u", static_cast<unsigned>(size));
        return nullptr;
    }
    return ptr;
}

void operator delete(void *ptr) noexcept
{
    debug_free(ptr);
}

void operator delete(void *ptr, std::size_t) noexcept
{
    debug_free(ptr);
}

void *operator new[](std::size_t size) noexcept
{
    void *ptr = debug_malloc(size);
    if (!ptr) {
        DEBUG_PRINT(DebugType::MEM, "operator new[] failed size=%u", static_cast<unsigned>(size));
        return nullptr;
    }
    return ptr;
}

void operator delete[](void *ptr) noexcept
{
    debug_free(ptr);
}

void operator delete[](void *ptr, std::size_t) noexcept
{
    debug_free(ptr);
}

void *operator new(std::size_t size, const std::nothrow_t &) noexcept
{
    return debug_malloc(size);
}

void operator delete(void *ptr, const std::nothrow_t &) noexcept
{
    debug_free(ptr);
}

void *operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
    return debug_malloc(size);
}

void operator delete[](void *ptr, const std::nothrow_t &) noexcept
{
    debug_free(ptr);
}

#endif
