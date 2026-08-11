#pragma once

#if MEM_DEBUG

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void *debug_malloc(size_t size);
void debug_free(void *ptr);
void *debug_realloc(void *ptr, size_t size);
void *debug_calloc(size_t nmemb, size_t size);
bool debug_heap_check(void);
void debug_heap_dump(void);

#ifdef __cplusplus
}
#endif

#endif
