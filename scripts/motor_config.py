#!/usr/bin/env python3
"""
Motor configuration and parameter tuning tool for SWO or USB/serial.

Author: sascha_lammers@gmx.de

*** Mostly AI generated code be aware of utter garbage that i might have missed ***
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import queue
import re
import socket
import struct
import subprocess
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from collections import deque
from dataclasses import dataclass
from tkinter import messagebox, simpledialog, ttk
from typing import Callable, Iterable, List, Optional, Tuple

from PIL import Image
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.ticker import MultipleLocator


# Mirrors include/adc_converters.h aliases:
# Voltage = VoltageConverterT<100000, 9100, 3300>
# Current = CurrentConverterT<4, 20, 3300>
# NTC = NTCConverterT<10000, 10000, 3950, 25, false>
ADC_MAX = 4095
VREF_MV = 3300
VOLTAGE_RES_TOP = 100000
VOLTAGE_RES_BOTTOM = 9100
CURRENT_SHUNT_MOHM = 4
CURRENT_GAIN = 20
NTC_SERIES_RESISTANCE = 10000
NTC_NOMINAL_RESISTANCE = 10000
NTC_BETA = 3950
NTC_NOMINAL_TEMP_C = 25.0
MAX_RPM = 55000
MAX_INPUT_CURRENT_LIMIT = 40000  # mA, matches the EEPROM input current limit UI maximum
# Current limit level selectable values, matches firmware kCurrentLimitLevelItems (menu.cpp)
CURRENT_LIMIT_LEVEL_NAMES = ("Low", "Medium", "High", "Very High")
MAX_CURRENT_LIMIT_LEVEL = len(CURRENT_LIMIT_LEVEL_NAMES) - 1  # 3
FIRMWARE_LOG_PATTERN = re.compile(r"^\[(\d{6,})\]\s+([^\s]+)\s+(.*)$")
PYOCD_LINE_PATTERN = re.compile(r"^\s*\d{6,}\s+[A-Za-z]\s+(.*?)(?:\s+\[[^\]]+\])?\s*$")
LOG_ENTRY_TYPE_PATTERN = re.compile(r"^\d{2}:\d{2}:\d{2}\.\d{4}\s+([A-Z0-9_]+):\s")
LOG_TYPE_NAMES = (
    "ERROR",
    "WARNING",
    "NOTICE",
    "INFO",
    "MEM",
    "UI",
    "PID",
    "GDB",
    "SERIAL",
)

@dataclass
class Sample:
    rpm: int
    pwm_level: int
    voltage_adc: int
    voltage_mv: int
    current_ocp_adc: int
    current_ocp_ma: int
    current_avg_adc: int
    current_avg_ma: int
    dac_motor_current: int
    dac_motor_current_ma: int
    dac_input_current: int
    dac_input_current_ma: int
    motor_temp_adc: int
    motor_temp_c: float
    mosfet_temp_adc: int
    mosfet_temp_c: float
    error: float
    integral: float
    derivative: float
    running: int
    drv_fault: int
    ocp_fault: int
    snsout_fault: int


@dataclass
class AppConfig:
    transport: str
    serial_port: str
    serial_baud: int
    uid: str
    target: str
    system_clock: int
    swo_clock: int
    swd_frequency: int
    connect_mode: str
    raw_port: int
    pid_port: int
    gdb_port: int


def _load_saved_settings_defaults() -> dict[str, object]:
    config_path = Path(__file__).resolve().with_suffix(".json")
    if not config_path.exists():
        return {}

    try:
        with config_path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except Exception:
        return {}

    return data if isinstance(data, dict) else {}


# Firmware protocol on SWO port 1: first byte is the payload length, followed by the raw PID item bytes.
PID_FRAME_LENGTH_PREFIX = 1
PID_ITEM_STRUCT = "<H10HBB"
PID_ITEM_SIZE = struct.calcsize(PID_ITEM_STRUCT)
PID_INTERVAL = 2.0
PID_SAMPLE_HZ_DEFAULT = int(1000 / PID_INTERVAL)
PID_PWM_MAX_LEVEL = 100.0
PID_ANTI_WINDUP_FACTOR = 512.0

# USBSerial::BinaryHeader: uint32_t magic, uint16_t size, BinaryType type, uint32_t crc.
BINARY_HEADER_STRUCT = "<IHHI"
BINARY_HEADER_SIZE = struct.calcsize(BINARY_HEADER_STRUCT)
BINARY_MAGIC = 0xDEADBEEF
BINARY_TYPE_PID = 0
BINARY_TYPE_TOGGLE_PID = 1
BINARY_TYPE_SCREENSHOT = 2
BINARY_TYPE_REQUEST_SCREENSHOT = 3
BINARY_TYPE_PARAMETERS = 4
BINARY_TYPE_REQUEST_PARAMETERS = 5
BINARY_TYPE_EEPROM = 6
BINARY_TYPE_REQUEST_EEPROM = 7
BINARY_TYPE_SYSTEM_RESET = 8
BINARY_MAX_PAYLOAD_SIZE = 65535
PID_PARAMETERS_STRUCT = "<fffHHHBx"
PID_PARAMETERS_SIZE = struct.calcsize(PID_PARAMETERS_STRUCT)


def _build_stm32_crc_tables() -> tuple[tuple[int, ...], ...]:
    byte_table = []
    for byte in range(256):
        crc = byte << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if crc & 0x80000000 else (crc << 1) & 0xFFFFFFFF
        byte_table.append(crc)

    def advance_word(value: int) -> int:
        for _ in range(4):
            value = ((value << 8) & 0xFFFFFFFF) ^ byte_table[value >> 24]
        return value

    return tuple(
        tuple(advance_word(byte << shift) for byte in range(256))
        for shift in (24, 16, 8, 0)
    )


STM32_CRC_TABLES = _build_stm32_crc_tables()


def binary_crc32(data: bytes) -> int:
    """Calculate the STM32 CRC result, zero-padding a partial final word."""
    data += b"\x00" * (-len(data) % 4)

    crc = 0xFFFFFFFF
    table_0, table_1, table_2, table_3 = STM32_CRC_TABLES
    for (word,) in struct.iter_unpack("<I", data):
        value = crc ^ word
        crc = (
            table_0[value >> 24]
            ^ table_1[(value >> 16) & 0xFF]
            ^ table_2[(value >> 8) & 0xFF]
            ^ table_3[value & 0xFF]
        )
    return crc

# Screenshot stream uses the same length-prefix scheme: [len][payload] and the stream terminated by a single 0 byte [0][no payload]
SCREENSHOT_END_MARKER = 0
SCREENSHOT_PIXEL_FORMAT_RGB565_LITTLE_ENDIAN = 0
SCREENSHOT_PIXEL_FORMAT_RGB565_BIG_ENDIAN = 1
SCREENSHOT_MAX_WIDTH = 320
SCREENSHOT_MAX_HEIGHT = 320
SCREENSHOT_MAX_PIXELS = SCREENSHOT_MAX_WIDTH * SCREENSHOT_MAX_HEIGHT
SCREENSHOT_BUFFER_SOFT_LIMIT = 65536
SCREENSHOT_BUFFER_TAIL_KEEP = 256
SCREENSHOT_TIMEOUT_SECONDS = 2.0
GRAPH_REFRESH_INTERVAL_SECONDS = 0.10
EEPROM_DIALOG_GRAPH_REFRESH_INTERVAL_SECONDS = 1.0
SCREENSHOT_TILE_HEADER_STRUCT = "<HHHHI"
SCREENSHOT_TILE_HEADER_SIZE = struct.calcsize(SCREENSHOT_TILE_HEADER_STRUCT)
SCREENSHOT_FRAME_HEADER_STRUCT = "<HHB3x"
SCREENSHOT_FRAME_HEADER_SIZE = struct.calcsize(SCREENSHOT_FRAME_HEADER_STRUCT)
SCREENSHOT_PORT = 2

# SWO::DataType layout in firmware with fixed RAM address
SWO_DATA_FIXED_RAM_ADDRESS = 0x2000F000
SWO_ENABLE_DISABLED = 0
SWO_ENABLE_SWO = 1
SWO_ENABLE_USB = 2
SWO_DATA_STRUCT = "<fffHHI?3xI?3x?xHB3x"
SWO_DATA_SIZE = struct.calcsize(SWO_DATA_STRUCT)
SWO_DATA_EEPROM_COMMIT_OFFSET = struct.calcsize("<fffHHI?3xI")
SWO_DATA_SEND_SCREENSHOT_OFFSET = struct.calcsize("<fffHHI?3xI?3x")

# EEPROM::DataType layout in firmware with dynamic EEPROM address from SWO::DataType
EEPROM_DATA_STRUCT = "<IIIIBBHHHHHBBBBBBBBHxxfffHHH?B"
EEPROM_DATA_SIZE = struct.calcsize(EEPROM_DATA_STRUCT)
NEW_EEPROM_SLOT = "New EEPROM Config..."

@dataclass
class SWOData:
    kp: float
    ki: float
    kd: float
    anti_windup: int
    rpm: int
    enabled_state: int
    changed: bool
    eeprom_address: int
    eeprom_commit: bool
    send_screenshot: bool
    input_current_limit: int
    current_limit_level: int


@dataclass
class EEPROMData:
    magic: int
    version: int
    sequence: int
    crc: int
    tft_brightness: int
    led_brightness: int
    input_current_limit: int
    motor_current_limit: int
    min_rpm: int
    max_rpm: int
    motor_stall_timeout: int
    motor_direction: int
    sensor_direction: int
    motor_brake: int
    control_mode: int
    mosfet_temperature_limit: int
    motor_temperature_limit: int
    max_pwm: int
    motor_pwm: int
    motor_rpm: int
    kp: float
    ki: float
    kd: float
    anti_windup: int
    ovp_protection: int
    pwm_frequency: int
    motor_chime: bool
    current_limit_level: int


EEPROM_FIELD_SPECS = (
    ("TFT Brightness", "tft_brightness", "int", 5, 100, None),
    ("LED Brightness", "led_brightness", "int", 0, 100, None),
    ("Input Current (mA)", "input_current_limit", "int", 500, 40000, None),
    ("Motor Current (mA)", "motor_current_limit", "int", 500, 40000, None),
    ("Current Limit Level", "current_limit_level", "choice", None, None, (("Low", 0), ("Medium", 1), ("High", 2), ("Very High", 3))),
    ("Min RPM", "min_rpm", "int", 10, MAX_RPM, None),
    ("Max RPM", "max_rpm", "int", 10, MAX_RPM, None),
    ("Motor Direction", "motor_direction", "choice", None, None, (("Forward", 0), ("Reverse", 1))),
    ("Sensor Direction", "sensor_direction", "choice", None, None, (("Forward", 0), ("Reverse", 1))),
    ("Control Mode", "control_mode", "choice", None, None, (("PWM / Open Loop", 0), ("PID / Closed Loop", 1))),
    ("Stall Timeout (ms)", "motor_stall_timeout", "int", 250, 10000, None),
    ("Motor Brake (%)", "motor_brake", "int", 0, 100, None),
    ("Max PWM (%)", "max_pwm", "int", 0, 100, None),
    ("MOSFET Temp Limit (C)", "mosfet_temperature_limit", "int", 25, 125, None),
    ("Motor Temp Limit (C)", "motor_temperature_limit", "int", 25, 85, None),
    ("Motor PWM (%)", "motor_pwm", "int", 0, 100, None),
    ("Motor RPM", "motor_rpm", "int", 0, 65535, None),
    ("Kp", "kp", "float", 0.0, 1000.0, None),
    ("Ki", "ki", "float", 0.0, 1000.0, None),
    ("Kd", "kd", "float", 0.0, 1000.0, None),
    ("Anti-Windup (%)", "anti_windup", "percent", 0.0, 100.0, None),
    ("OVP Protection (mV)", "ovp_protection", "int", 8000, 40000, None),
    ("PWM Frequency (Hz)", "pwm_frequency", "int", 5000, 40000, None),
    ("Welcome Chime", "motor_chime", "choice", None, None, (("Off", 0), ("On", 1))),
)


def convert_voltage_mv(adc_value: int) -> int:
    divider_ratio = int(VREF_MV * ((VOLTAGE_RES_TOP + VOLTAGE_RES_BOTTOM) / VOLTAGE_RES_BOTTOM))
    return (adc_value * divider_ratio) // ADC_MAX


def convert_current_ma(adc_value: int) -> int:
    mv_per_lsb_times_1000 = (VREF_MV * 1000) // ADC_MAX
    return (adc_value * mv_per_lsb_times_1000) // (CURRENT_SHUNT_MOHM * CURRENT_GAIN)

def convert_ntc_celsius(adc_value: int) -> float:
    if adc_value <= 0 or adc_value >= ADC_MAX:
        return 0.0

    resistance = (
        float(NTC_SERIES_RESISTANCE)
        * float(adc_value)
        / (float(ADC_MAX) - float(adc_value))
    )

    temperature = math.log(resistance / float(NTC_NOMINAL_RESISTANCE))
    temperature /= float(NTC_BETA)
    temperature += 1.0 / (NTC_NOMINAL_TEMP_C + 273.15)
    temperature = 1.0 / temperature
    return temperature - 273.15


def decode_fault_word(word: int) -> Tuple[int, int, int, int]:
    # C++ bitfield order in PidLoopType:
    # bit0=running, bit1=drv8701Fault, bit2=ocpFault, bit3=snsoutFault.
    running = word & 0x1
    drv_fault = (word >> 1) & 0x1
    ocp_fault = (word >> 2) & 0x1
    snsout_fault = (word >> 3) & 0x1
    return running, drv_fault, ocp_fault, snsout_fault


def anti_windup_raw_to_percent(raw_value: int) -> float:
    return float(raw_value) / PID_ANTI_WINDUP_FACTOR


def anti_windup_percent_to_raw(percent_value: float) -> int:
    return int(round(percent_value * PID_ANTI_WINDUP_FACTOR))


def current_limit_level_to_name(value: int) -> str:
    if 0 <= value < len(CURRENT_LIMIT_LEVEL_NAMES):
        return CURRENT_LIMIT_LEVEL_NAMES[value]
    return str(value)


def current_limit_level_name_to_int(name: str) -> int:
    try:
        return CURRENT_LIMIT_LEVEL_NAMES.index(str(name).strip())
    except ValueError:
        raise ValueError(f"Unknown current limit level: {name}")


def _uint16_to_float(raw_value: int) -> float:
    return float(raw_value) * 65535.0


def decode_pid_item(payload: bytes) -> Optional[Sample]:
    if len(payload) != PID_ITEM_SIZE:
        return None

    rpm, voltage, i_ocp, i_avg, motor_ntc, mosfet_ntc, dac_motor, dac_input, error_raw, integral_raw, derivative_raw, pwm, faults = struct.unpack(
        PID_ITEM_STRUCT, payload
    )
    error = _uint16_to_float(error_raw)
    integral = _uint16_to_float(integral_raw)
    derivative = _uint16_to_float(derivative_raw)

    if rpm > MAX_RPM:  # RPM might go negative due to small vibrations when the motor is stalled and the sensor limit is 55k RPM
        rpm = 0

    running, drv_fault, ocp_fault, snsout_fault = decode_fault_word(faults)
    return Sample(
        rpm=rpm,
        pwm_level=pwm,
        voltage_adc=voltage,
        voltage_mv=convert_voltage_mv(voltage),
        current_ocp_adc=i_ocp,
        current_ocp_ma=convert_current_ma(i_ocp),
        current_avg_adc=i_avg,
        current_avg_ma=convert_current_ma(i_avg),
        dac_motor_current=dac_motor,
        dac_motor_current_ma=convert_current_ma(dac_motor),
        dac_input_current=dac_input,
        dac_input_current_ma=convert_current_ma(dac_input),
        motor_temp_adc=motor_ntc,
        motor_temp_c=convert_ntc_celsius(motor_ntc),
        mosfet_temp_adc=mosfet_ntc,
        mosfet_temp_c=convert_ntc_celsius(mosfet_ntc),
        error=error,
        integral=integral,
        derivative=derivative,
        running=running,
        drv_fault=drv_fault,
        ocp_fault=ocp_fault,
        snsout_fault=snsout_fault,
    )


def decode_pid_parameters(payload: bytes) -> Optional[tuple[float, float, float, int, int, int, int]]:
    if len(payload) != PID_PARAMETERS_SIZE:
        return None
    return struct.unpack(PID_PARAMETERS_STRUCT, payload)


def decode_eeprom_data(payload: bytes) -> Optional[EEPROMData]:
    if len(payload) != EEPROM_DATA_SIZE:
        return None
    return EEPROMData(*struct.unpack(EEPROM_DATA_STRUCT, payload))


def is_plausible_sample(sample: Sample) -> bool:
    return not sample_validation_errors(sample)


def sample_validation_errors(sample: Sample) -> list[str]:
    errors: list[str] = []
    for value in (
        sample.voltage_adc,
        sample.current_ocp_adc,
        sample.current_avg_adc,
        sample.dac_motor_current,
        sample.dac_input_current,
        sample.motor_temp_adc,
        sample.mosfet_temp_adc,
    ):
        if value > ADC_MAX:
            errors.append(f"ADC value {value} exceeds {ADC_MAX}")

    for name, value in (
        ("running", sample.running),
        ("drv_fault", sample.drv_fault),
        ("ocp_fault", sample.ocp_fault),
        ("snsout_fault", sample.snsout_fault),
    ):
        if value not in (0, 1):
            errors.append(f"{name}={value} is not 0 or 1")

    return errors


class BinaryPacketParser:
    """Parse USBSerial binary packets and report non-packet bytes as text."""

    def __init__(
        self,
        sample_callback: Callable[[Sample], None],
        log_callback: Callable[[str], None],
        screenshot_callback: Optional[Callable[[bytes], None]] = None,
        screenshot_error_callback: Optional[Callable[[str], None]] = None,
        parameters_callback: Optional[Callable[[bytes], None]] = None,
        eeprom_callback: Optional[Callable[[bytes], None]] = None,
    ) -> None:
        self.sample_callback = sample_callback
        self.log_callback = log_callback
        self.screenshot_callback = screenshot_callback
        self.screenshot_error_callback = screenshot_error_callback
        self.parameters_callback = parameters_callback
        self.eeprom_callback = eeprom_callback
        self.buffer = bytearray()
        self.debug_text = bytearray()
        self.last_pid_payload: Optional[bytes] = None
        self._screenshot_stream = bytearray()
        self._screenshot_corrupt = False

    def feed(self, data: bytes) -> None:
        self.buffer.extend(data)

        while True:
            magic_bytes = struct.pack("<I", BINARY_MAGIC)
            header_offset = self.buffer.find(magic_bytes)
            if header_offset < 0:
                keep = min(len(self.buffer), len(magic_bytes) - 1)
                if len(self.buffer) > keep:
                    self._consume_text(bytes(self.buffer[:-keep]))
                    del self.buffer[:-keep]
                return

            if header_offset:
                self._consume_text(bytes(self.buffer[:header_offset]))
                del self.buffer[:header_offset]

            if len(self.buffer) < BINARY_HEADER_SIZE:
                return

            magic, payload_size, packet_type, _crc = struct.unpack(
                BINARY_HEADER_STRUCT,
                self.buffer[:BINARY_HEADER_SIZE],
            )
            if magic != BINARY_MAGIC:
                self._consume_text(bytes(self.buffer[:1]))
                del self.buffer[:1]
                continue

            packet_size = BINARY_HEADER_SIZE + payload_size
            if len(self.buffer) < packet_size:
                return

            payload = bytes(self.buffer[BINARY_HEADER_SIZE:packet_size])
            expected_crc = binary_crc32(payload)
            if _crc != expected_crc:
                error = (
                    f"Binary CRC mismatch: type={packet_type} size={payload_size} "
                    f"received=0x{_crc:08x} calculated=0x{expected_crc:08x}"
                )
                self.log_callback(error)
                if packet_type == BINARY_TYPE_SCREENSHOT:
                    if not self._screenshot_corrupt:
                        self._screenshot_corrupt = True
                        self._screenshot_stream.clear()
                        if self.screenshot_error_callback is not None:
                            self.screenshot_error_callback(error)
                del self.buffer[:len(magic_bytes)]
                continue
            del self.buffer[:packet_size]
            self._dispatch(packet_type, payload)

    def flush(self) -> None:
        if self.buffer:
            self._consume_text(bytes(self.buffer))
            self.buffer.clear()
        if self._screenshot_corrupt:
            self._screenshot_stream.clear()
            self._screenshot_corrupt = False
        elif self._screenshot_stream:
            self._screenshot_stream.clear()
            if self.screenshot_error_callback is not None:
                self.screenshot_error_callback(
                    "Screenshot stream ended without zero-byte terminator"
                )
        if self.debug_text:
            self.log_callback(self.debug_text.decode("utf-8", errors="replace"))
            self.debug_text.clear()

    def _consume_text(self, data: bytes) -> None:
        self.debug_text.extend(data)
        while b"\n" in self.debug_text:
            line_end = self.debug_text.index(0x0A)
            line = bytes(self.debug_text[:line_end]).rstrip(b"\r")
            del self.debug_text[: line_end + 1]
            if line:
                self.log_callback(line.decode("utf-8", errors="replace"))

    def _dispatch(self, packet_type: int, payload: bytes) -> None:
        if packet_type == BINARY_TYPE_SCREENSHOT:
            if self.screenshot_callback is None:
                return

            if payload == b"\x00":
                if self._screenshot_corrupt:
                    self._screenshot_stream.clear()
                    self._screenshot_corrupt = False
                elif self._screenshot_stream:
                    self.screenshot_callback(bytes(self._screenshot_stream) + payload)
                    self._screenshot_stream.clear()
                else:
                    self.screenshot_callback(payload)
                return

            self._screenshot_stream.extend(payload)
            return
        if packet_type == BINARY_TYPE_PARAMETERS:
            if self.parameters_callback is not None:
                self.parameters_callback(payload)
            return
        if packet_type == BINARY_TYPE_EEPROM:
            if self.eeprom_callback is not None:
                self.eeprom_callback(payload)
            return
        if packet_type != BINARY_TYPE_PID:
            return

        sample = decode_pid_item(payload)
        if sample is None:
            return
        validation_errors = sample_validation_errors(sample)
        if validation_errors:
            return
        if payload == self.last_pid_payload:
            return

        self.last_pid_payload = payload
        self.sample_callback(sample)


def parse_itm_packets(buffer: bytearray) -> Iterable[Tuple[int, bytes]]:
    """Parse ITM packets from raw SWV stream.

    This parser is intentionally small and focused on instrumentation packets.
    """
    while buffer:
        header = buffer[0]

        # Idle/sync/overflow markers.
        if header in (0x00, 0x80, 0x70):
            del buffer[0]
            continue

        size_code = header & 0x3
        if (header & 0x4) == 0 and size_code != 0:
            size = 4 if size_code == 0x3 else size_code
            if len(buffer) < 1 + size:
                break
            port = header >> 3
            payload = bytes(buffer[1 : 1 + size])
            del buffer[: 1 + size]
            yield port, payload
            continue

        # Skip other packet kinds.
        if len(buffer) == 1:
            break
        del buffer[0]
        while buffer:
            value = buffer[0]
            del buffer[0]
            if (value & 0x80) == 0:
                break


class SWOBackend:
    def __init__(
        self,
        config: AppConfig,
        log_callback: Callable[[str], None],
        screenshot_path_factory: Callable[[], Path],
        screenshot_event_callback: Callable[[str, object], None],
        sample_callback: Callable[[Sample], None],
    ) -> None:
        self.config = config
        self.log = log_callback
        self._screenshot_path_factory = screenshot_path_factory
        self._screenshot_event_callback = screenshot_event_callback
        self.sample_callback = sample_callback
        self.proc: Optional[subprocess.Popen] = None
        self.stop_event = threading.Event()
        self.stdout_thread: Optional[threading.Thread] = None
        self.swv_thread: Optional[threading.Thread] = None
        self.running = False
        self._screenshot_lock = threading.RLock()
        self._screenshot_output_path: Optional[Path] = None
        self._screenshot_buffer = bytearray()
        self._screenshot_image: Optional[Image.Image] = None
        self._screenshot_tiles: list[tuple[int, int, int, int, bytes]] = []
        self._screenshot_width = 0
        self._screenshot_height = 0
        self._screenshot_format = -1
        self._screenshot_has_explicit_frame = False
        self._screenshot_last_packet_time = 0.0

    def request_screenshot(self, output_path: Path) -> None:
        with self._screenshot_lock:
            self._screenshot_output_path = output_path
            self._screenshot_buffer.clear()
            self._screenshot_image = None
            self._screenshot_tiles = []
            self._screenshot_width = 0
            self._screenshot_height = 0
            self._screenshot_format = -1
            self._screenshot_has_explicit_frame = False
            self._screenshot_last_packet_time = time.monotonic()

    def cancel_screenshot(self) -> None:
        with self._screenshot_lock:
            self._screenshot_output_path = None
            self._screenshot_buffer.clear()
            self._screenshot_image = None
            self._screenshot_tiles = []
            self._screenshot_width = 0
            self._screenshot_height = 0
            self._screenshot_format = -1
            self._screenshot_has_explicit_frame = False
            self._screenshot_last_packet_time = 0.0

    def has_pending_screenshot(self) -> bool:
        with self._screenshot_lock:
            return self._screenshot_output_path is not None

    def fail_screenshot_if_idle(self, idle_seconds: float) -> bool:
        with self._screenshot_lock:
            if self._screenshot_output_path is None:
                return False
            if self._screenshot_last_packet_time <= 0.0:
                return False
            if (time.monotonic() - self._screenshot_last_packet_time) < idle_seconds:
                return False

        self._screenshot_event_callback(
            "screenshot-error",
            "Screenshot incomplete: zero-byte terminator was not received",
        )
        self.cancel_screenshot()
        return True

    def build_pyocd_command(self, reset_run: bool) -> List[str]:
        cmd = [
            "pyocd",
            "gdbserver",
            "--target",
            self.config.target,
            "-O",
            f"frequency={self.config.swd_frequency}",
            "-O",
            f"connect_mode={self.config.connect_mode}",
            "-O",
            "enable_semihosting=1",
            "-O",
            "semihost_console_type=console",
            "-O",
            "enable_swv=1",
            "-O",
            "swv_raw_enable=1",
            "-O",
            f"swv_raw_port={self.config.raw_port}",
            "-O",
            f"swv_system_clock={self.config.system_clock}",
            "-O",
            f"swv_clock={self.config.swo_clock}",
            "--persist",
        ]
        if reset_run:
            cmd.insert(4, "--reset-run")
        if self.config.uid:
            cmd.extend(["--uid", self.config.uid])
        return cmd

    def start(self, reset_run: bool) -> bool:
        if self.running:
            return True

        cmd = self.build_pyocd_command(reset_run=reset_run)
        self.log("Launching: " + " ".join(cmd), "GDB")

        try:
            # pyOCD's SWVEventSink writes decoded ITM port 0 text to its stdout, which would be a
            # second copy of what _read_raw_swv() decodes. Discard stdout and keep stderr, where
            # pyOCD logging goes.
            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except FileNotFoundError:
            self.log("pyOCD not found. Install with: pip install pyocd", "ERROR")
            return False
        except Exception as exc:  # pragma: no cover - defensive path
            self.log(f"Failed to launch pyOCD: {exc}", "ERROR")
            return False

        self.stop_event.clear()
        self.stdout_thread = threading.Thread(target=self._read_pyocd_log, daemon=True)
        self.swv_thread = threading.Thread(target=self._read_raw_swv, daemon=True)
        self.stdout_thread.start()
        self.swv_thread.start()
        self.running = True
        return True

    def stop(self) -> None:
        self.stop_event.set()
        self.cancel_screenshot()

        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                self.proc.kill()

        self.proc = None
        self.running = False

    def reset_target(self) -> bool:
        cmd = [
            "pyocd",
            "commander",
            "--target",
            self.config.target,
            "-O",
            f"frequency={self.config.swd_frequency}",
            "-M",
            "attach",
        ]
        if self.config.uid:
            cmd.extend(["--uid", self.config.uid])
        cmd.extend(["-c", "reset"])

        self.log("Resetting firmware: " + " ".join(cmd), "GDB")
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
                timeout=8.0,
            )
        except FileNotFoundError:
            self.log("pyOCD not found. Install with: pip install pyocd", "ERROR")
            return False
        except subprocess.TimeoutExpired as exc:
            self.log("Firmware reset timed out after 8s", "ERROR")
            if exc.stdout:
                self.log(str(exc.stdout).rstrip(), "ERROR")
            return False
        except Exception as exc:  # pragma: no cover - defensive path
            self.log(f"Failed to reset firmware: {exc}", "ERROR")
            return False

        if result.returncode != 0:
            self.log(f"Firmware reset failed with exit code {result.returncode}", "ERROR")
            if result.stdout:
                self.log(result.stdout.rstrip(), "ERROR")
            return False

        if result.stdout:
            self.log(result.stdout.rstrip())
        return True

    def _read_pyocd_log(self) -> None:
        if not self.proc or self.proc.stderr is None:
            return

        buffer = bytearray()
        while not self.stop_event.is_set():
            chunk = self.proc.stderr.read(1)
            if not chunk:
                if buffer:
                    self.log(self._clean_pyocd_line(buffer.decode("utf-8", errors="replace")), "GDB")
                break

            buffer.extend(chunk)
            if chunk == b"\n":
                self.log(self._clean_pyocd_line(buffer.decode("utf-8", errors="replace")), "GDB")
                buffer.clear()

        self.running = False
        return_code = None
        try:
            if self.proc is not None:
                return_code = self.proc.poll()
        except Exception:
            return_code = None
        self.log(
            f"pyOCD log loop ended (return code: {return_code})",
            "ERROR" if return_code not in (None, 0) else "GDB",
        )

    @staticmethod
    def _clean_pyocd_line(text: str) -> str:
        line = text.rstrip("\r\n")
        match = PYOCD_LINE_PATTERN.match(line)
        return match.group(1) if match is not None else line

    def _read_raw_swv(self) -> None:
        last_connect_log = 0.0

        while not self.stop_event.is_set():
            try:
                with socket.create_connection(("127.0.0.1", self.config.raw_port), timeout=1.0) as sock:
                    sock.settimeout(1.0)
                    self.log(
                        f"Connected to SWV raw stream on tcp://127.0.0.1:{self.config.raw_port}",
                        "GDB",
                    )

                    accumulated = bytearray()
                    pid_bytes = bytearray()
                    itm_text = bytearray()
                    last_sample_payload: Optional[bytes] = None

                    while not self.stop_event.is_set():
                        try:
                            chunk = sock.recv(4096)
                        except socket.timeout:
                            continue

                        if not chunk:
                            break

                        accumulated.extend(chunk)
                        for port, payload in parse_itm_packets(accumulated):
                            if port == 0:
                                # Sole source of firmware text; pyOCD's own decoded copy on stdout is discarded.
                                itm_text.extend(payload)
                                while b"\n" in itm_text:
                                    line_end = itm_text.index(0x0A)
                                    line = bytes(itm_text[:line_end]).rstrip(b"\r")
                                    del itm_text[: line_end + 1]
                                    if line:
                                        self.log(line.decode("utf-8", errors="replace"))
                                continue

                            if port == SCREENSHOT_PORT:
                                try:
                                    self._consume_screenshot_payload(payload)
                                except Exception as exc:  # pragma: no cover - defensive path
                                    self._screenshot_event_callback(
                                        "screenshot-error",
                                        f"Screenshot stream decode failed: {exc}",
                                    )
                                    self._reset_screenshot_stream()
                                continue

                            if port != self.config.pid_port:
                                continue

                            pid_bytes.extend(payload)

                            while len(pid_bytes) >= PID_FRAME_LENGTH_PREFIX:
                                frame_length = pid_bytes[0]
                                if frame_length == 0:
                                    del pid_bytes[:1]
                                    continue

                                frame_size = PID_FRAME_LENGTH_PREFIX + frame_length
                                if len(pid_bytes) < frame_size:
                                    break

                                raw_item = bytes(pid_bytes[1:frame_size])
                                sample = decode_pid_item(raw_item)
                                if sample and raw_item != last_sample_payload and is_plausible_sample(sample):
                                    del pid_bytes[:frame_size]
                                    last_sample_payload = raw_item
                                    self.sample_callback(sample)
                                    continue

                                # Invalid candidate frame: shift one byte to recover from partial or malformed
                                # sequences. The firmware precedes each PID item with a single-byte length.
                                del pid_bytes[0]

            except OSError:
                accumulated = bytearray()
                pid_bytes = bytearray()
                now = time.monotonic()
                if now - last_connect_log > 2.0:
                    self.log(
                        f"Waiting for SWV raw server on tcp://127.0.0.1:{self.config.raw_port}...",
                        "GDB",
                    )
                    last_connect_log = now
                time.sleep(0.2)

    @staticmethod
    def _rgb565_to_rgb_bytes(payload: bytes, pixel_format: int) -> bytes:
        if len(payload) % 2 != 0:
            raise ValueError("RGB565 payload length must be even")

        rgb = bytearray((len(payload) // 2) * 3)
        dst = 0
        for src in range(0, len(payload), 2):
            if pixel_format == SCREENSHOT_PIXEL_FORMAT_RGB565_LITTLE_ENDIAN:
                value = payload[src] | (payload[src + 1] << 8)
            else:
                value = (payload[src] << 8) | payload[src + 1]
            red = ((value >> 11) & 0x1F) * 255 // 31
            green = ((value >> 5) & 0x3F) * 255 // 63
            blue = (value & 0x1F) * 255 // 31
            rgb[dst] = red
            rgb[dst + 1] = green
            rgb[dst + 2] = blue
            dst += 3
        return bytes(rgb)

    def _reset_screenshot_stream(self) -> None:
        with self._screenshot_lock:
            self._screenshot_buffer.clear()
            self._screenshot_image = None
            self._screenshot_tiles = []
            self._screenshot_width = 0
            self._screenshot_height = 0
            self._screenshot_format = -1
            self._screenshot_has_explicit_frame = False

    def _finalize_screenshot_stream(self) -> None:
        with self._screenshot_lock:
            image = self._screenshot_image
            tiles = self._screenshot_tiles
            output_path = self._screenshot_output_path
            self._screenshot_output_path = None
            self._screenshot_buffer.clear()
            self._screenshot_image = None
            self._screenshot_tiles = []
            self._screenshot_width = 0
            self._screenshot_height = 0
            self._screenshot_format = -1
            self._screenshot_has_explicit_frame = False

        if image is None or output_path is None:
            return

        try:
            for x, y, width, height, tile_bytes in tiles:
                rgb_bytes = self._rgb565_to_rgb_bytes(tile_bytes, self._screenshot_format)
                tile_image = Image.frombytes("RGB", (width, height), rgb_bytes)
                image.paste(tile_image, (x, y))
            output_path.parent.mkdir(parents=True, exist_ok=True)
            image.save(output_path, format="PNG")
        except Exception as exc:  # pragma: no cover - defensive path
            self._screenshot_event_callback("screenshot-error", f"Failed to save screenshot: {exc}")
            return

        self._screenshot_event_callback("screenshot-complete", str(output_path))

    def _begin_screenshot_capture_locked(self, source: str) -> bool:
        if self._screenshot_output_path is not None:
            return True
        try:
            self._screenshot_output_path = self._screenshot_path_factory()
            self._screenshot_event_callback(
                "screenshot-started",
                str(self._screenshot_output_path),
            )
            self.log(
                f"Auto-capturing unsolicited screenshot ({source}) -> {self._screenshot_output_path.name}"
            )
            return True
        except Exception as exc:
            self._screenshot_event_callback(
                "screenshot-error",
                f"Failed to allocate screenshot path: {exc}",
            )
            self._reset_screenshot_stream()
            return False

    def _expand_screenshot_canvas_locked(self, required_width: int, required_height: int) -> bool:
        if required_width <= self._screenshot_width and required_height <= self._screenshot_height:
            return True
        if required_width > SCREENSHOT_MAX_WIDTH or required_height > SCREENSHOT_MAX_HEIGHT:
            return False
        if self._screenshot_image is None:
            return False

        new_width = max(self._screenshot_width, required_width)
        new_height = max(self._screenshot_height, required_height)
        expanded = Image.new("RGB", (new_width, new_height))
        expanded.paste(self._screenshot_image, (0, 0))
        self._screenshot_image = expanded
        self._screenshot_width = new_width
        self._screenshot_height = new_height
        return True

    def _fail_screenshot_stream_locked(self, message: str) -> None:
        self._screenshot_event_callback("screenshot-error", message)
        self._reset_screenshot_stream()

    @staticmethod
    def _validate_frame_dimensions(width: int, height: int) -> Optional[str]:
        if width == 0 or height == 0:
            return "Invalid screenshot frame dimensions"
        if width > SCREENSHOT_MAX_WIDTH or height > SCREENSHOT_MAX_HEIGHT:
            return f"Screenshot frame too large: {width}x{height}"
        if (width * height) > SCREENSHOT_MAX_PIXELS:
            return f"Screenshot pixel count too large: {width * height}"
        return None

    @staticmethod
    def _validate_tile_payload(width: int, height: int, byte_count: int) -> Optional[str]:
        if width == 0 or height == 0:
            return "Invalid screenshot tile dimensions"
        expected_bytes = int(width) * int(height) * 2
        if byte_count != expected_bytes or (byte_count % 2) != 0:
            return f"Invalid screenshot tile payload size: got {byte_count}, expected {expected_bytes}"
        return None

    def _start_tile_first_capture_locked(self, x: int, y: int, width: int, height: int) -> bool:
        if not self._begin_screenshot_capture_locked("tile-first"):
            return False

        required_width = x + width
        required_height = y + height
        if required_width <= 0 or required_height <= 0:
            self._fail_screenshot_stream_locked("Invalid screenshot tile bounds")
            return False
        if required_width > SCREENSHOT_MAX_WIDTH or required_height > SCREENSHOT_MAX_HEIGHT:
            self._fail_screenshot_stream_locked(
                f"Screenshot tile exceeds max bounds: {required_width}x{required_height}"
            )
            return False

        self._screenshot_width = required_width
        self._screenshot_height = required_height
        self._screenshot_format = -1
        self._screenshot_has_explicit_frame = False
        self._screenshot_image = Image.new("RGB", (required_width, required_height))
        self._screenshot_tiles = []
        return True

    def _consume_screenshot_payload(self, payload: bytes) -> None:
        with self._screenshot_lock:
            self._screenshot_last_packet_time = time.monotonic()
            self._screenshot_buffer.extend(payload)

            # Prevent unbounded buffer growth from corrupt/stuck state.
            if len(self._screenshot_buffer) > SCREENSHOT_BUFFER_SOFT_LIMIT:
                # Keep the tail in case we're in the middle of reading pixel data.
                self._screenshot_buffer = self._screenshot_buffer[-SCREENSHOT_BUFFER_TAIL_KEEP:]

            while len(self._screenshot_buffer) > 0:
                # Check for end-of-stream marker.
                if self._screenshot_buffer[0] == SCREENSHOT_END_MARKER:
                    del self._screenshot_buffer[:1]
                    self._finalize_screenshot_stream()
                    # Clear buffer to prevent stale bytes from being misinterpreted by next capture.
                    self._screenshot_buffer.clear()
                    return

                # Need at least 1 byte for the record length prefix.
                if len(self._screenshot_buffer) < 1:
                    return

                record_length = self._screenshot_buffer[0]
                record_total_size = 1 + record_length

                # Sanity check: record length should be reasonable
                # If we get an unexpected length, skip this byte and try to resync.
                if record_length not in (SCREENSHOT_FRAME_HEADER_SIZE, SCREENSHOT_TILE_HEADER_SIZE):
                    # Skip malformed record length byte.
                    del self._screenshot_buffer[0:1]
                    continue

                # Need the full length-prefixed record.
                if len(self._screenshot_buffer) < record_total_size:
                    return

                # Extract the record payload (without the length prefix).
                record = bytes(self._screenshot_buffer[1:record_total_size])
                del self._screenshot_buffer[:record_total_size]

                # Parse frame header
                if len(record) == SCREENSHOT_FRAME_HEADER_SIZE:
                    width, height, pixel_format, = struct.unpack(SCREENSHOT_FRAME_HEADER_STRUCT, record)

                    if not self._begin_screenshot_capture_locked("frame-header"):
                        return

                    if pixel_format not in (SCREENSHOT_PIXEL_FORMAT_RGB565_LITTLE_ENDIAN, SCREENSHOT_PIXEL_FORMAT_RGB565_BIG_ENDIAN):
                        self._fail_screenshot_stream_locked(
                            f"Unsupported screenshot format: {pixel_format}"
                        )
                        return

                    frame_error = self._validate_frame_dimensions(width, height)
                    if frame_error is not None:
                        self._fail_screenshot_stream_locked(frame_error)
                        return

                    self._screenshot_width = width
                    self._screenshot_height = height
                    self._screenshot_format = pixel_format
                    self._screenshot_has_explicit_frame = True
                    self._screenshot_image = Image.new("RGB", (width, height))
                    self._screenshot_tiles = []
                    continue

                # Parse tile header
                if len(record) == SCREENSHOT_TILE_HEADER_SIZE:
                    x, y, width, height, byte_count = struct.unpack(SCREENSHOT_TILE_HEADER_STRUCT, record)

                    tile_error = self._validate_tile_payload(width, height, byte_count)
                    if tile_error is not None:
                        self._fail_screenshot_stream_locked(tile_error)
                        return

                    # If this is the first tile, auto-create canvas.
                    if self._screenshot_image is None:
                        if not self._start_tile_first_capture_locked(x, y, width, height):
                            return

                    # Read the pixel data (byteCount bytes).
                    if len(self._screenshot_buffer) < byte_count:
                        # Put the record back and wait for more data.
                        self._screenshot_buffer = bytearray([record_length]) + record + self._screenshot_buffer
                        return

                    tile_bytes = bytes(self._screenshot_buffer[:byte_count])
                    del self._screenshot_buffer[:byte_count]

                    # Validate bounds.
                    if x + width > self._screenshot_width or y + height > self._screenshot_height:
                        if self._screenshot_has_explicit_frame:
                            self._fail_screenshot_stream_locked("Screenshot tile out of bounds")
                            return
                        if not self._expand_screenshot_canvas_locked(x + width, y + height):
                            self._fail_screenshot_stream_locked("Screenshot tile exceeds dynamic bounds")
                            return

                    self._screenshot_tiles.append((x, y, width, height, tile_bytes))
                    continue

                # Should not reach here given the record_length check above.
                # Unknown record type; skip one byte and try again.
                # This helps recover from protocol desynchronization.
                del self._screenshot_buffer[0:1]


class SerialBackend(SWOBackend):
    """Direct USB CDC backend using the firmware BinaryHeader protocol."""

    def __init__(
        self,
        config: AppConfig,
        log_callback: Callable[[str], None],
        screenshot_path_factory: Callable[[], Path],
        screenshot_event_callback: Callable[[str, object], None],
        sample_callback: Callable[[Sample], None],
        parameters_callback: Callable[[bytes], None],
        eeprom_callback: Callable[[bytes], None],
    ) -> None:
        super().__init__(
            config,
            log_callback,
            screenshot_path_factory,
            screenshot_event_callback,
            sample_callback,
        )
        self.stop_event = threading.Event()
        self.reader_thread: Optional[threading.Thread] = None
        self.serial_port: Optional[object] = None
        self.running = False
        self.parameters_callback = parameters_callback
        self.eeprom_callback = eeprom_callback

    @staticmethod
    def _is_com_port(port_name: str) -> bool:
        return bool(re.fullmatch(r"COM\d+", port_name.strip(), re.IGNORECASE))

    def _resolve_serial_port_name(self, port_name: str) -> str:
        requested_port = str(port_name).strip()
        if self._is_com_port(requested_port):
            return requested_port

        try:
            list_ports = importlib.import_module("serial.tools.list_ports")
        except ImportError:
            return requested_port

        def normalize(value: Optional[str]) -> str:
            return str(value or "").strip().lower()

        windows_device_id = re.fullmatch(
            r"USB\\VID_([0-9A-F]{4})&PID_([0-9A-F]{4})\\(.+)",
            requested_port,
            re.IGNORECASE,
        )
        parts = [part.strip() for part in requested_port.split(":") if part.strip()]
        vid = pid = serial_number = None
        if windows_device_id is not None:
            vid = int(windows_device_id.group(1), 16)
            pid = int(windows_device_id.group(2), 16)
            serial_number = windows_device_id.group(3)
        elif len(parts) >= 2:
            try:
                vid = int(parts[0], 16)
                pid = int(parts[1], 16)
            except ValueError:
                vid = pid = None
            if len(parts) >= 3:
                serial_number = parts[2]

        for port in list_ports.comports():
            if normalize(port.device) == normalize(requested_port):
                return port.device
            if vid is not None and pid is not None and port.vid == vid and port.pid == pid:
                if serial_number is not None:
                    if normalize(port.serial_number) == normalize(serial_number):
                        return port.device
                    continue
                return port.device
            if normalize(port.serial_number) == normalize(requested_port):
                return port.device
            if normalize(port.hwid).startswith(normalize(requested_port)):
                return port.device
            if normalize(port.description) == normalize(requested_port):
                return port.device

        return requested_port

    def _send_binary_command(self, command_type: int, payload: bytes = b"\x00\x00\x00\x00") -> None:
        self._send_binary_payload(command_type, payload)

    def _send_binary_payload(self, packet_type: int, payload: bytes) -> None:
        serial_port = self.serial_port
        if serial_port is None:
            raise RuntimeError("USB serial transport is not running")

        crc = binary_crc32(payload)
        header = struct.pack(BINARY_HEADER_STRUCT, BINARY_MAGIC, len(payload), packet_type, crc)
        serial_port.write(header)  # type: ignore[attr-defined]
        serial_port.write(payload)  # type: ignore[attr-defined]
        serial_port.flush()  # type: ignore[attr-defined]

    def start(self, reset_run: bool = False) -> bool:
        del reset_run
        if self.running:
            return True

        port_name = self.config.serial_port
        try:
            serial = importlib.import_module("serial")

            port_name = self._resolve_serial_port_name(self.config.serial_port)
            is_usb_device = port_name.casefold() != self.config.serial_port.casefold()
            serial_port = serial.Serial(
                port_name,
                self.config.serial_baud,
                timeout=0.2,
            )
            try:
                serial_port.set_buffer_size(rx_size=256 * 1024)
            except (AttributeError, OSError):
                pass
        except ImportError:
            self.log("pyserial not found. Install with: pip install pyserial", "ERROR")
            return False
        except Exception as exc:
            port_description = f"COM port {port_name}"
            if is_usb_device:
                port_description += f" (USB device ID {self.config.serial_port})"
            self.log(
                f"Failed to open {port_description}: {exc}",
                "ERROR",
            )
            try:
                serial_port.close()  # type: ignore[union-attr]
            except Exception:
                pass
            return False

        self.serial_port = serial_port
        try:
            self._send_binary_command(BINARY_TYPE_TOGGLE_PID, struct.pack("<I", 1))
        except Exception as exc:
            self.log(f"Failed to send PID start command: {exc}", "ERROR")
            try:
                serial_port.close()  # type: ignore[attr-defined]
            except Exception:
                pass
            self.serial_port = None
            return False

        self.stop_event.clear()
        self.running = True
        self.reader_thread = threading.Thread(target=self._read_serial, daemon=True)
        self.reader_thread.start()
        self.log(f"Connected to {port_name}%s at {self.config.serial_baud} baud" % (is_usb_device and f" ({self.config.serial_port})" or ""))
        self.log("Sent PID start command")
        return True

    def stop(self) -> None:
        self.stop_event.set()
        serial_port = self.serial_port

        if serial_port is not None:
            try:
                self._send_binary_command(BINARY_TYPE_TOGGLE_PID, struct.pack("<I", 0))
                self.log("Sent PID stop command")
            except Exception as exc:
                self.log(f"Failed to send PID stop command: {exc}", "ERROR")
            try:
                serial_port.close()  # type: ignore[attr-defined]
            except Exception as exc:
                self.log(f"Failed to close serial port: {exc}", "ERROR")
            finally:
                self.serial_port = None

        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)
            self.reader_thread = None
        self.running = False

    def request_screenshot(self, output_path: Path) -> None:
        super().request_screenshot(output_path)
        if self.serial_port is None:
            self.cancel_screenshot()
            raise RuntimeError("USB serial transport is not running")
        try:
            self._send_binary_command(BINARY_TYPE_REQUEST_SCREENSHOT)
        except Exception:
            self.cancel_screenshot()
            raise
        self.log(f"Requested screenshot -> {output_path.name}")

    def _fail_corrupt_screenshot(self, error: str) -> None:
        if not self.has_pending_screenshot():
            return
        self._screenshot_event_callback("screenshot-error", error)
        self.cancel_screenshot()

    def request_eeprom(self) -> None:
        self._send_binary_command(BINARY_TYPE_REQUEST_EEPROM)
        self.log("Requested EEPROM data")

    def reset_target(self) -> bool:
        if self.serial_port is None:
            self.log("USB serial transport is not running", "ERROR")
            return False
        try:
            self._send_binary_command(BINARY_TYPE_SYSTEM_RESET)
            self.log("Sent system reset command")
            return True
        except Exception as exc:
            self.log(f"Failed to send system reset command: {exc}", "ERROR")
            return False

    def _read_serial(self) -> None:
        parser = BinaryPacketParser(
            self.sample_callback,
            self._log_serial_debug,
            screenshot_callback=self._consume_screenshot_payload,
            screenshot_error_callback=self._fail_corrupt_screenshot,
            parameters_callback=self.parameters_callback,
            eeprom_callback=self.eeprom_callback,
        )
        serial_port = self.serial_port
        if serial_port is None:
            return

        try:
            while not self.stop_event.is_set():
                waiting = serial_port.in_waiting  # type: ignore[attr-defined]
                read_size = max(4096, min(waiting, BINARY_MAX_PAYLOAD_SIZE))
                chunk = serial_port.read(read_size)  # type: ignore[attr-defined]
                if chunk:
                    try:
                        parser.feed(bytes(chunk))
                    except Exception as exc:  # pragma: no cover - defensive path
                        self._screenshot_event_callback(
                            "screenshot-error",
                            f"Screenshot stream decode failed: {exc}",
                        )
                        self._reset_screenshot_stream()
        except Exception as exc:
            if not self.stop_event.is_set():
                self.log(f"Serial read failed: {exc}", "ERROR")
        finally:
            parser.flush()
            if not self.stop_event.is_set():
                self.running = False
                self.log("Serial reader stopped")

    def _log_serial_debug(self, message: str) -> None:
        if re.match(r"^\[\d+\] ", message) or message.startswith("Binary CRC mismatch:"):
            self.log(message)


class GDBMemoryClient:
    """Direct GDB RSP client for reading/writing target memory via pyOCD gdbserver."""

    def __init__(self, config: AppConfig, timeout: float = 4.0) -> None:
        self.config = config
        self.timeout = timeout
        self._lock = threading.Lock()

    @staticmethod
    def _checksum(payload: str) -> int:
        return sum(payload.encode("ascii")) & 0xFF

    def _read_byte(self, sock: socket.socket) -> bytes:
        value = sock.recv(1)
        if not value:
            raise ConnectionError("GDB connection closed")
        return value

    def _read_packet(self, sock: socket.socket) -> str:
        while True:
            ch = self._read_byte(sock)
            if ch == b"$":
                break

        data = bytearray()
        while True:
            ch = self._read_byte(sock)
            if ch == b"#":
                break
            data.extend(ch)

        checksum = sock.recv(2)
        if len(checksum) != 2:
            raise ConnectionError("Invalid GDB checksum")
        sock.sendall(b"+")
        return data.decode("ascii", errors="replace")

    def _send_packet(self, sock: socket.socket, payload: str) -> str:
        packet = f"${payload}#{self._checksum(payload):02x}".encode("ascii")
        sock.sendall(packet)

        while True:
            ch = self._read_byte(sock)
            if ch == b"+":
                break
            if ch == b"-":
                sock.sendall(packet)
                continue
            if ch == b"$":
                # Ignore async traffic and consume its checksum/ack.
                data = bytearray()
                while True:
                    c2 = self._read_byte(sock)
                    if c2 == b"#":
                        break
                    data.extend(c2)
                _ = sock.recv(2)
                sock.sendall(b"+")

        return self._read_packet(sock)

    def _connect(self) -> socket.socket:
        sock = socket.create_connection(("127.0.0.1", self.config.gdb_port), timeout=self.timeout)
        sock.settimeout(self.timeout)
        return sock

    def _reset_connection(self) -> None:
        self.close()

    def _detach_and_close(self, sock: socket.socket) -> None:
        # Detach so the target resumes, then close this short-lived RSP session.
        try:
            response = self._send_packet(sock, "D")
            if response not in ("OK", ""):
                pass
        except Exception:
            pass
        finally:
            try:
                sock.close()
            except Exception:
                pass

    def close(self) -> None:
        # Connections are short-lived and closed per operation.
        return

    @staticmethod
    def _parse_hex_bytes(output: str) -> bytes:
        output = output.strip()
        if not output:
            return b""
        if output.startswith("E"):
            raise RuntimeError(f"GDB read/write error: {output}")
        if all(ch in "0123456789abcdefABCDEF" for ch in output) and len(output) % 2 == 0:
            return bytes.fromhex(output)
        words = re.findall(r"0x[0-9a-fA-F]+", output)
        if words:
            return b"".join(struct.pack("<I", int(word, 16)) for word in words)
        raise RuntimeError(f"Could not parse GDB output: {output}")

    def read_memory(self, address: int, size: int) -> bytes:
        with self._lock:
            last_error: Optional[BaseException] = None
            for attempt in range(2):
                sock = self._connect()
                try:
                    response = self._send_packet(sock, f"m{address:x},{size:x}")
                    return self._parse_hex_bytes(response)[:size]
                except (OSError, ConnectionError, RuntimeError) as exc:
                    last_error = exc
                    self._reset_connection()
                    if attempt == 0:
                        continue
                    break
                finally:
                    self._detach_and_close(sock)

            assert last_error is not None
            raise last_error

    def write_memory(self, address: int, data: bytes) -> None:
        with self._lock:
            last_error: Optional[BaseException] = None
            for attempt in range(2):
                sock = self._connect()
                try:
                    response = self._send_packet(sock, f"M{address:x},{len(data):x}:{data.hex()}")
                    if response != "OK":
                        raise RuntimeError(f"Unexpected GDB write response: {response}")
                    return
                except (OSError, ConnectionError, RuntimeError) as exc:
                    last_error = exc
                    self._reset_connection()
                    if attempt == 0:
                        continue
                    break
                finally:
                    self._detach_and_close(sock)

            assert last_error is not None
            raise last_error


class PIDTuningApp:
    PRESETS = (5, 10, 20, 30)
    STARTUP_PACKET_TIMEOUT_SECONDS = 5.0
    EEPROM_CONFIG_SLOT_COUNT = 10

    @staticmethod
    def _make_series(length: int) -> deque[float]:
        return deque([math.nan] * length, maxlen=length)

    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self._firmware_timestamp_offset: Optional[float] = None
        self._log_timestamp_lock = threading.Lock()
        self.log_file_path = self._configured_log_file_path()
        self._log_write_error_reported = False
        self._log_write_error_path: Optional[str] = None
        self._log_type_vars: dict[str, tk.BooleanVar] = {}
        self._log_type_visibility: dict[str, bool] = {}
        self.event_queue: queue.Queue[Tuple[str, object]] = queue.Queue()
        log_callback = lambda msg, log_type="INFO": self.event_queue.put(("log", (msg, log_type)))
        sample_callback = lambda sample: self.event_queue.put(("sample", sample))
        screenshot_event_callback = lambda kind, payload: self.event_queue.put((kind, payload))
        parameters_callback = lambda payload: self.event_queue.put(("serial-parameters", payload))
        eeprom_callback = lambda payload: self.event_queue.put(("serial-eeprom", payload))
        if config.transport == "serial":
            self.backend = SerialBackend(
                config,
                log_callback=log_callback,
                screenshot_path_factory=self._next_screenshot_path,
                screenshot_event_callback=screenshot_event_callback,
                sample_callback=sample_callback,
                parameters_callback=parameters_callback,
                eeprom_callback=eeprom_callback,
            )
        else:
            self.backend = SWOBackend(
                config,
                log_callback=log_callback,
                screenshot_path_factory=self._next_screenshot_path,
                screenshot_event_callback=screenshot_event_callback,
                sample_callback=sample_callback,
            )

        self.root = tk.Tk()
        self.root.title("Motor Controller Config")
        self.root.geometry("1920x1200")
        self.root.minsize(1000, 640)

        self.window_seconds_var = tk.StringVar(value="10")
        self.status_var = tk.StringVar(value="Stopped")
        self.kp_var = tk.StringVar(value="0.0")
        self.ki_var = tk.StringVar(value="0.0")
        self.kd_var = tk.StringVar(value="0.0")
        self.anti_windup_var = tk.StringVar(value="0.00")
        self.rpm_var = tk.StringVar(value="0")
        self.input_current_limit_var = tk.StringVar(value="0")
        self.current_limit_level_var = tk.StringVar(value=CURRENT_LIMIT_LEVEL_NAMES[0])
        self._pid_fields_updating = False
        self._pid_fields_dirty = False
        self._last_loaded_swo_data: Optional[SWOData] = None
        self._config_dialog: Optional[tk.Toplevel] = None
        self._config_dialog_vars: dict[str, tk.StringVar] = {}
        self._eeprom_dialog: Optional[tk.Toplevel] = None
        self._eeprom_dialog_status_var: Optional[tk.StringVar] = None
        self._eeprom_dialog_vars: dict[str, tk.StringVar] = {}
        self._eeprom_dialog_widgets: list[tuple[tk.Widget, str]] = []
        self._eeprom_dialog_summary_vars: dict[str, tk.StringVar] = {}
        self._eeprom_dialog_commit_button: Optional[ttk.Button] = None
        self._eeprom_dialog_source: Optional[EEPROMData] = None
        self._eeprom_dialog_address: Optional[int] = None
        self._restore_ui_state()
        self.screenshot_in_progress = False
        self._screenshot_deadline = 0.0
        self._install_pid_field_traces()

        self.data_address: Optional[int] = SWO_DATA_FIXED_RAM_ADDRESS
        self.sync_in_progress = False
        self.reset_in_progress = False
        self._auto_restart_after_reset = False
        self._startup_sync_job: Optional[str] = None
        self._waiting_for_first_sample = False
        self._first_sample_deadline = 0.0
        self.gdb_mem = GDBMemoryClient(config)

        self._initial_sash_done = False
        self._build_layout()
        self._update_control_button_states()
        self.root.after_idle(self._update_control_button_states)
        self.window_seconds_var.set(str(self._load_graph_time_window()))
        self._init_buffers(window_seconds=int(self.window_seconds_var.get()))
        self._apply_graph_visibility()
        self._refresh_plot()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(50, self._set_initial_sashes)
        self.root.after(40, self._process_events)

    def _build_layout(self) -> None:
        self.panes = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        self.panes.pack(fill=tk.BOTH, expand=True)

        self.graph_frame = ttk.Frame(self.panes)
        self.right_frame = ttk.Frame(self.panes)

        self.graph_frame.configure(width=1440)
        self.right_frame.configure(width=480)
        self.panes.add(self.graph_frame, weight=3)
        self.panes.add(self.right_frame, weight=1)

        self.right_panes = ttk.Panedwindow(self.right_frame, orient=tk.VERTICAL)
        self.right_panes.pack(fill=tk.BOTH, expand=True)

        self.log_frame = ttk.Frame(self.right_panes)
        self.faults_frame = ttk.Frame(self.right_panes)
        self.controls_frame = ttk.Frame(self.right_panes)

        self.log_frame.configure(height=390)
        self.faults_frame.configure(height=120)
        self.controls_frame.configure(height=310)
        self.right_panes.add(self.log_frame, weight=2)
        self.right_panes.add(self.faults_frame, weight=1)
        self.right_panes.add(self.controls_frame, weight=1)

        self._build_graph_panel()
        self._build_log_panel()
        self._build_faults_panel()
        self._build_controls_panel()

    def _set_initial_sashes(self) -> None:
        if self._initial_sash_done:
            return
        width = self.panes.winfo_width()
        right_height = self.right_panes.winfo_height()
        if width <= 10 or right_height <= 10:
            self.root.after(50, self._set_initial_sashes)
            return

        self.panes.sashpos(0, int(width * 0.75))
        # Keep enough room for controls and faults so they are visible at startup.
        controls_min_height = 260
        faults_height = 120
        log_height = int(right_height * 0.42)
        log_height = min(log_height, right_height - controls_min_height - faults_height)
        log_height = max(log_height, 120)
        self.right_panes.sashpos(0, log_height)
        self.right_panes.sashpos(1, log_height + faults_height)

        self._initial_sash_done = True

    def _build_graph_panel(self) -> None:
        self.figure = Figure(figsize=(12, 7), dpi=100)
        self.axes = self.figure.subplots(6, 1, sharex=True)
        self.figure.subplots_adjust(left=0.055, right=0.995, top=0.963, bottom=0.065, hspace=0.30)

        titles = [
            "RPM / Avg RPM",
            "PWM (%) / Avg PWM (%)",
            "Current Avg/OCP + DAC Limits (mA)",
            "Voltage (mV)",
            "Temperatures (C)",
            "Integral",
        ]
        ylabels = ["RPM", "%", "mA", "mV", "C", "I"]

        for axis, title, ylabel in zip(self.axes, titles, ylabels):
            axis.set_title(title, loc="left", fontsize=10)
            axis.set_ylabel(ylabel)
            axis.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)

        self._apply_x_grid()
        self.axes[-1].set_xlabel("Time (s)")

        self.canvas = FigureCanvasTkAgg(self.figure, master=self.graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=0, pady=0)

    def _build_log_panel(self) -> None:
        wrap = ttk.Frame(self.log_frame)
        wrap.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        wrap.rowconfigure(0, weight=1)
        wrap.columnconfigure(0, weight=1)

        self.log_text = tk.Text(
            wrap,
            height=8,
            state=tk.DISABLED,
            font=("Consolas", 10),
            wrap=tk.NONE,
        )
        v_scrollbar = ttk.Scrollbar(wrap, orient=tk.VERTICAL, command=self.log_text.yview)
        h_scrollbar = ttk.Scrollbar(wrap, orient=tk.HORIZONTAL, command=self.log_text.xview)
        self.log_text.configure(yscrollcommand=v_scrollbar.set, xscrollcommand=h_scrollbar.set)

        self.log_text.grid(row=0, column=0, sticky="nsew")
        v_scrollbar.grid(row=0, column=1, sticky="ns")
        h_scrollbar.grid(row=1, column=0, sticky="ew")

        settings = self._load_settings_data()
        filter_frame = ttk.LabelFrame(wrap, text="Log Types")
        filter_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        for column in range(5):
            filter_frame.columnconfigure(column, weight=1)

        for index, log_type in enumerate(LOG_TYPE_NAMES):
            enabled = bool(settings.get(f"log_type_{log_type}", True))
            self._log_type_visibility[log_type] = enabled
            variable = tk.BooleanVar(value=enabled)
            self._log_type_vars[log_type] = variable
            ttk.Checkbutton(
                filter_frame,
                text=log_type,
                variable=variable,
                command=lambda type_name=log_type: self._set_log_type_visibility(type_name),
            ).grid(
                row=index // 5,
                column=index % 5,
                sticky="w",
                padx=6,
                pady=2,
            )

    def _set_fault_indicator(self, widget: tk.Label, active: bool) -> None:
        if active:
            widget.configure(text="ACTIVE", bg="#9A031E", fg="#FFFFFF")
        else:
            widget.configure(text="OK", bg="#2A9D8F", fg="#FFFFFF")

    def _build_faults_panel(self) -> None:
        wrap = ttk.LabelFrame(self.faults_frame, text="Fault Indicators")
        wrap.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)
        wrap.columnconfigure(0, weight=1)
        wrap.columnconfigure(1, weight=0)

        ttk.Label(wrap, text="Over Current Protection").grid(row=0, column=0, sticky="w", padx=8, pady=4)
        self.ocp_indicator = tk.Label(wrap, width=10, anchor="center", relief="groove")
        self.ocp_indicator.grid(row=0, column=1, sticky="e", padx=8, pady=4)

        ttk.Label(wrap, text="DRV8701 Fault").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        self.driver_fault_indicator = tk.Label(wrap, width=10, anchor="center", relief="groove")
        self.driver_fault_indicator.grid(row=1, column=1, sticky="e", padx=8, pady=4)

        ttk.Label(wrap, text="DRV8701 SNSOUT (OCP)").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        self.driver_ocp_indicator = tk.Label(wrap, width=10, anchor="center", relief="groove")
        self.driver_ocp_indicator.grid(row=2, column=1, sticky="e", padx=8, pady=4)

        self._set_fault_indicator(self.ocp_indicator, False)
        self._set_fault_indicator(self.driver_fault_indicator, False)
        self._set_fault_indicator(self.driver_ocp_indicator, False)

    def _build_controls_panel(self) -> None:
        outer = ttk.Frame(self.controls_frame)
        outer.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        outer.columnconfigure(0, weight=0)
        outer.columnconfigure(1, weight=1)

        self.start_stop_button = ttk.Button(outer, text="Start", command=self._toggle_start_stop)
        self.start_stop_button.grid(row=0, column=0, rowspan=2, sticky="nsw", padx=(0, 12))

        group = ttk.LabelFrame(outer, text="Time Window (seconds)")
        group.grid(row=0, column=1, sticky="ew", pady=(0, 6))
        group.columnconfigure(0, weight=0)
        group.columnconfigure(1, weight=0)
        group.columnconfigure(2, weight=1)

        ttk.Label(group, text="Window:").grid(row=0, column=0, padx=6, pady=6)
        self.window_entry = ttk.Entry(group, width=8, textvariable=self.window_seconds_var)
        self.window_entry.grid(row=0, column=1, padx=6, pady=6)
        ttk.Button(group, text="Apply", command=self._apply_window).grid(row=0, column=2, padx=6, pady=6, sticky="w")

        presets_wrap = ttk.Frame(group)
        presets_wrap.grid(row=1, column=0, columnspan=3, sticky="w", padx=6, pady=(0, 6))
        ttk.Label(presets_wrap, text="Presets:").pack(side=tk.LEFT, padx=(0, 6))
        for preset in self.PRESETS:
            ttk.Button(
                presets_wrap,
                text=f"{preset}s",
                command=lambda value=preset: self._set_window_preset(value),
            ).pack(side=tk.LEFT, padx=(0, 4))

        status_frame = ttk.Frame(outer)
        status_frame.grid(row=1, column=1, sticky="ew")
        ttk.Label(status_frame, text="Status:").pack(side=tk.LEFT)
        ttk.Label(status_frame, textvariable=self.status_var).pack(side=tk.LEFT, padx=(6, 0))

        pid_group = ttk.LabelFrame(outer, text="PID Parameters")
        pid_group.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 6))
        pid_group.columnconfigure(1, weight=1)

        ttk.Label(pid_group, text="Kp:").grid(row=0, column=0, padx=6, pady=4, sticky="w")
        ttk.Entry(pid_group, textvariable=self.kp_var, width=14).grid(row=0, column=1, padx=6, pady=4, sticky="ew")

        ttk.Label(pid_group, text="Ki:").grid(row=1, column=0, padx=6, pady=4, sticky="w")
        ttk.Entry(pid_group, textvariable=self.ki_var, width=14).grid(row=1, column=1, padx=6, pady=4, sticky="ew")

        ttk.Label(pid_group, text="Kd:").grid(row=2, column=0, padx=6, pady=4, sticky="w")
        ttk.Entry(pid_group, textvariable=self.kd_var, width=14).grid(row=2, column=1, padx=6, pady=4, sticky="ew")

        ttk.Label(pid_group, text="Anti-Windup (%):").grid(row=3, column=0, padx=6, pady=4, sticky="w")
        ttk.Entry(pid_group, textvariable=self.anti_windup_var, width=14).grid(row=3, column=1, padx=6, pady=4, sticky="ew")

        ttk.Label(pid_group, text="RPM:").grid(row=4, column=0, padx=6, pady=4, sticky="w")
        ttk.Entry(pid_group, textvariable=self.rpm_var, width=14).grid(row=4, column=1, padx=6, pady=4, sticky="ew")

        ttk.Label(pid_group, text="Limit (mA):").grid(row=5, column=0, padx=6, pady=4, sticky="w")
        ttk.Entry(pid_group, textvariable=self.input_current_limit_var, width=14).grid(row=5, column=1, padx=6, pady=4, sticky="ew")

        ttk.Label(pid_group, text="Limit Level:").grid(row=6, column=0, padx=6, pady=4, sticky="w")
        ttk.Combobox(
            pid_group,
            textvariable=self.current_limit_level_var,
            values=CURRENT_LIMIT_LEVEL_NAMES,
            state="readonly",
            width=12,
        ).grid(row=6, column=1, padx=6, pady=4, sticky="ew")

        self.sync_button = ttk.Button(pid_group, text="Sync", command=self._sync_to_target, state=tk.DISABLED)
        self.sync_button.grid(
            row=7, column=0, columnspan=2, padx=6, pady=(8, 6), sticky="ew"
        )

        self.eeprom_button = ttk.Button(pid_group, text="EEPROM...", command=self._open_eeprom_dialog, state=tk.DISABLED)
        self.eeprom_button.grid(row=8, column=0, columnspan=2, padx=6, pady=(0, 6), sticky="ew")

        self.screenshot_button = ttk.Button(pid_group, text="Screenshot", command=self._request_screenshot, state=tk.DISABLED)
        self.screenshot_button.grid(row=9, column=0, columnspan=2, padx=6, pady=(0, 6), sticky="ew")

        self.reset_firmware_button = ttk.Button(
            pid_group,
            text="Reset Firmware",
            command=self._reset_firmware_manual,
            state=tk.DISABLED,
        )
        self.reset_firmware_button.grid(row=10, column=0, columnspan=2, padx=6, pady=(0, 6), sticky="ew")

        self.config_button = ttk.Button(pid_group, text="Config...", command=self._open_config_dialog)
        self.config_button.grid(row=11, column=0, columnspan=2, padx=6, pady=(0, 6), sticky="ew")

        ttk.Label(
            outer,
            text="Panels are resizable; drag separators to adjust Graph / Logs / Faults / Controls.",
            justify=tk.LEFT,
        ).grid(row=4, column=0, columnspan=2, sticky="w", pady=(6, 0))

    def _pack_swo_data(
        self,
        data: SWOData,
    ) -> bytes:
        # C++ layout (SWO::DataType): float Kp, float Ki, float Kd, uint16_t antiWindup,
        # uint16_t rpm, uint32_t enabled, bool changed, 3x padding, uint32_t address,
        # bool commit, 3x padding, bool sendScreenshot, 1x padding (uint16 alignment),
        # uint16_t inputCurrentLimit, uint8_t currentLimitLevel, 3x trailing padding.
        return struct.pack(
            SWO_DATA_STRUCT,
            data.kp,
            data.ki,
            data.kd,
            data.anti_windup,
            data.rpm,
            data.enabled_state,
            data.changed,
            data.eeprom_address,
            data.eeprom_commit,
            data.send_screenshot,
            data.input_current_limit,
            data.current_limit_level,
        )

    def _pack_eeprom_data(self, data: EEPROMData) -> bytes:
        return struct.pack(
            EEPROM_DATA_STRUCT,
            data.magic,
            data.version,
            data.sequence,
            data.crc,
            data.tft_brightness,
            data.led_brightness,
            data.input_current_limit,
            data.motor_current_limit,
            data.min_rpm,
            data.max_rpm,
            data.motor_stall_timeout,
            data.motor_direction,
            data.sensor_direction,
            data.motor_brake,
            data.control_mode,
            data.mosfet_temperature_limit,
            data.motor_temperature_limit,
            data.max_pwm,
            data.motor_pwm,
            data.motor_rpm,
            data.kp,
            data.ki,
            data.kd,
            data.anti_windup,
            data.ovp_protection,
            data.pwm_frequency,
            data.motor_chime,
            data.current_limit_level,
        )

    def _unpack_eeprom_data(self, payload: bytes) -> EEPROMData:
        values = struct.unpack(EEPROM_DATA_STRUCT, payload[:EEPROM_DATA_SIZE])
        return EEPROMData(*values)

    def _validate_swo_payload(self, payload: bytes) -> SWOData:
        if len(payload) < SWO_DATA_SIZE:
            raise RuntimeError(f"SWO::data read returned too few bytes ({len(payload)} < {SWO_DATA_SIZE})")

        kp, ki, kd, anti_windup_raw, rpm, enabled_state, changed, eeprom_address, eeprom_commit, send_screenshot, input_current_limit, current_limit_level = struct.unpack(
            SWO_DATA_STRUCT,
            payload[:SWO_DATA_SIZE],
        )
        data = SWOData(
            kp=kp,
            ki=ki,
            kd=kd,
            anti_windup=anti_windup_raw,
            rpm=rpm,
            enabled_state=enabled_state,
            changed=changed,
            eeprom_address=eeprom_address,
            eeprom_commit=eeprom_commit,
            send_screenshot=send_screenshot,
            input_current_limit=input_current_limit,
            current_limit_level=current_limit_level,
        )

        if enabled_state not in (SWO_ENABLE_DISABLED, SWO_ENABLE_SWO, SWO_ENABLE_USB):
            raise RuntimeError(f"Invalid SWO::data enabled state: {enabled_state}")

        for name, value in (("Kp", data.kp), ("Ki", data.ki), ("Kd", data.kd)):
            if not math.isfinite(value):
                raise RuntimeError(f"Invalid SWO::data {name}: not finite")

        if not (0 <= data.rpm <= MAX_RPM):
            raise RuntimeError(f"Invalid SWO::data RPM: {data.rpm}")

        if not (0 <= data.anti_windup <= anti_windup_percent_to_raw(100.0)):
            raise RuntimeError(
                f"Invalid SWO::data anti-windup raw value: {data.anti_windup}"
            )

        if not (0 <= data.input_current_limit <= MAX_INPUT_CURRENT_LIMIT):
            raise RuntimeError(f"Invalid SWO::data input current limit: {data.input_current_limit}")

        if not (0 <= data.current_limit_level <= MAX_CURRENT_LIMIT_LEVEL):
            raise RuntimeError(f"Invalid SWO::data current limit level: {data.current_limit_level}")

        if data.eeprom_address < 0x20000000:
            raise RuntimeError("Invalid SWO::data EEPROM address: %08x" % data.eeprom_address)

        return data

    def _install_pid_field_traces(self) -> None:
        for var in (self.kp_var, self.ki_var, self.kd_var, self.anti_windup_var, self.rpm_var, self.input_current_limit_var, self.current_limit_level_var):
            var.trace_add("write", self._on_pid_field_edited)

    def _on_pid_field_edited(self, *_: object) -> None:
        if self._pid_fields_updating:
            return
        self._pid_fields_dirty = True

    def _set_pid_fields(self, data: SWOData) -> None:
        self._pid_fields_updating = True
        try:
            self.kp_var.set(f"{data.kp:.6f}")
            self.ki_var.set(f"{data.ki:.6f}")
            self.kd_var.set(f"{data.kd:.6f}")
            self.anti_windup_var.set(f"{anti_windup_raw_to_percent(data.anti_windup):.2f}")
            self.rpm_var.set(str(data.rpm))
            self.input_current_limit_var.set(str(data.input_current_limit))
            self.current_limit_level_var.set(current_limit_level_to_name(data.current_limit_level))
            self._last_loaded_swo_data = data
            self._pid_fields_dirty = False
        finally:
            self._pid_fields_updating = False

    def _apply_serial_parameters(self, payload: bytes) -> None:
        parameters = decode_pid_parameters(payload)
        if parameters is None:
            self._append_log(f"Invalid serial parameter packet size: {len(payload)}", "ERROR")
            return

        kp, ki, kd, anti_windup, rpm, input_current_limit, current_limit_level = parameters
        if not all(math.isfinite(value) for value in (kp, ki, kd)):
            self._append_log("Invalid serial parameter packet: non-finite PID value", "ERROR")
            return
        if anti_windup > anti_windup_percent_to_raw(100.0) or rpm > MAX_RPM:
            self._append_log("Invalid serial parameter packet: value out of range", "ERROR")
            return
        if not (0 <= input_current_limit <= MAX_INPUT_CURRENT_LIMIT) or not (0 <= current_limit_level <= MAX_CURRENT_LIMIT_LEVEL):
            self._append_log("Invalid serial parameter packet: current limit out of range", "ERROR")
            return

        self._pid_fields_updating = True
        try:
            self.kp_var.set(f"{kp:.6f}")
            self.ki_var.set(f"{ki:.6f}")
            self.kd_var.set(f"{kd:.6f}")
            self.anti_windup_var.set(f"{anti_windup_raw_to_percent(anti_windup):.2f}")
            self.rpm_var.set(str(rpm))
            self.input_current_limit_var.set(str(input_current_limit))
            self.current_limit_level_var.set(current_limit_level_to_name(current_limit_level))
            self._pid_fields_dirty = False
        finally:
            self._pid_fields_updating = False

        self._append_log(
            f"Received PID params: Kp={kp:.6f} Ki={ki:.6f} Kd={kd:.6f} "
            f"AWR={anti_windup_raw_to_percent(anti_windup):.2f}% RPM={rpm} "
            f"Current={input_current_limit}mA Level={current_limit_level}",
            "INFO",
        )

    def _set_sync_enabled(self, enabled: bool) -> None:
        self.sync_button.configure(state=(tk.NORMAL if enabled else tk.DISABLED))

    def _set_eeprom_button_enabled(self, enabled: bool) -> None:
        self.eeprom_button.configure(state=(tk.NORMAL if enabled else tk.DISABLED))

    def _set_screenshot_button_enabled(self, enabled: bool) -> None:
        self.screenshot_button.configure(state=(tk.NORMAL if enabled else tk.DISABLED))

    def _set_reset_firmware_button_enabled(self, enabled: bool) -> None:
        self.reset_firmware_button.configure(state=(tk.NORMAL if enabled else tk.DISABLED))

    def _update_control_button_states(self) -> None:
        swo_transport = self.config.transport == "swo"
        config_enabled = not self.backend.running and not self.reset_in_progress
        eeprom_enabled = (swo_transport or self.config.transport == "serial") and self.backend.running and not self.reset_in_progress and self._eeprom_dialog is None
        screenshot_enabled = self.backend.running and not self.reset_in_progress and not self.screenshot_in_progress
        reset_enabled = swo_transport and not self.reset_in_progress and self._eeprom_dialog is None
        self._set_sync_enabled(
            (swo_transport or self.config.transport == "serial")
            and self.backend.running
            and not self.reset_in_progress
        )
        self._set_eeprom_button_enabled(eeprom_enabled)
        self._set_screenshot_button_enabled(screenshot_enabled)
        self._set_reset_firmware_button_enabled(reset_enabled)
        self.config_button.configure(state=(tk.NORMAL if config_enabled else tk.DISABLED))

    def _next_screenshot_path(self) -> Path:
        script_dir = Path(__file__).resolve().parent
        existing = [
            int(p.stem[2:])
            for p in script_dir.glob("ss*.png")
            if p.stem[2:].isdigit() and len(p.stem) == 8
        ]
        index = (max(existing) + 1) if existing else 0
        return script_dir / f"ss{index:06d}.png"

    def _request_screenshot(self) -> None:
        if self.config.transport == "swo" and self.data_address is None:
            self._append_log("Cannot take screenshot: no dataAddress received yet", "WARNING")
            return
        if not self.backend.running or self.reset_in_progress:
            self._append_log("Cannot take screenshot: monitor is not running", "WARNING")
            return
        if self.screenshot_in_progress:
            self._append_log("Screenshot already in progress", "WARNING")
            return

        output_path = self._next_screenshot_path()
        self.screenshot_in_progress = True
        self._screenshot_deadline = time.monotonic() + SCREENSHOT_TIMEOUT_SECONDS
        self._update_control_button_states()

        def worker() -> None:
            try:
                if self.config.transport == "serial":
                    self.backend.request_screenshot(output_path)
                else:
                    payload = self.gdb_mem.read_memory(self.data_address, SWO_DATA_SIZE)
                    self._validate_swo_payload(payload)
                    self.backend.request_screenshot(output_path)
                    self.gdb_mem.write_memory(
                        self.data_address + SWO_DATA_SEND_SCREENSHOT_OFFSET,
                        b"\x01",
                    )
                    verify = self.gdb_mem.read_memory(self.data_address + SWO_DATA_SEND_SCREENSHOT_OFFSET, 1)
                    if verify not in (b"\x00", b"\x01"):
                        raise RuntimeError(f"Screenshot flag write failed: {verify.hex()}")
                    if verify == b"\x00":
                        self.event_queue.put(("log", "Screenshot flag consumed immediately by firmware"))
                    self.event_queue.put(("log", f"Requested screenshot -> {output_path.name}"))
            except Exception as exc:
                self.backend.cancel_screenshot()
                self.event_queue.put(("screenshot-error", f"Screenshot request failed: {exc}"))

        threading.Thread(target=worker, daemon=True).start()

    def _set_target_enabled(self, enabled: bool, quiet: bool = False) -> None:
        if self.config.transport != "swo":
            return
        if self.data_address is None:
            return

        def worker() -> None:
            try:
                payload = self.gdb_mem.read_memory(self.data_address, SWO_DATA_SIZE)
                current_data = self._validate_swo_payload(payload)
                current_data.enabled_state = SWO_ENABLE_SWO if enabled else SWO_ENABLE_DISABLED
                self.gdb_mem.write_memory(self.data_address, self._pack_swo_data(current_data))
            except Exception as exc:
                if not quiet:
                    self.event_queue.put(("log", f"Set SWO::data.enabled failed: {exc}"))

        threading.Thread(target=worker, daemon=True).start()

    def _format_eeprom_field_value(self, spec: tuple[str, str, str, Optional[float], Optional[float], Optional[tuple[tuple[str, int], ...]]], data: EEPROMData) -> str:
        _, field_name, field_kind, _, _, choices = spec
        value = getattr(data, field_name)

        if field_kind == "choice" and choices is not None:
            for label, candidate in choices:
                if candidate == value:
                    return label
            return str(value)

        if field_kind == "float":
            return f"{float(value):.6f}"

        if field_kind == "percent":
            return f"{anti_windup_raw_to_percent(int(value)):.2f}"

        return str(int(value))

    def _set_eeprom_dialog_editable(self, enabled: bool) -> None:
        state = tk.NORMAL if enabled else tk.DISABLED
        combo_state = "readonly" if enabled else tk.DISABLED

        for widget, widget_kind in self._eeprom_dialog_widgets:
            widget.configure(state=(combo_state if widget_kind == "choice" else state))

        if self._eeprom_dialog_commit_button is not None:
            self._eeprom_dialog_commit_button.configure(state=(tk.NORMAL if enabled else tk.DISABLED))

    def _close_eeprom_dialog(self) -> None:
        if self._eeprom_dialog is not None:
            try:
                self._eeprom_dialog.grab_release()
            except Exception:
                pass
            try:
                self._eeprom_dialog.geometry()
                self._save_ui_state()
            except Exception:
                pass
            try:
                self._eeprom_dialog.destroy()
            except Exception:
                pass

        self._eeprom_dialog = None
        self._eeprom_dialog_status_var = None
        self._eeprom_dialog_vars = {}
        self._eeprom_dialog_widgets = []
        self._eeprom_dialog_summary_vars = {}
        self._eeprom_dialog_commit_button = None
        self._eeprom_dialog_save_button = None
        self._eeprom_dialog_load_button = None
        self._eeprom_dialog_slot_var = None
        self._eeprom_dialog_slot_combo = None
        self._eeprom_dialog_source = None
        self._eeprom_dialog_address = None
        self._update_control_button_states()

    def _default_eeprom_slot_names(self) -> list[str]:
        return [f"slot_{index + 1}" for index in range(self.EEPROM_CONFIG_SLOT_COUNT)]

    def _eeprom_slot_names(self) -> list[str]:
        data = self._load_settings_data()
        slots = data.get("eeprom_slots", []) if isinstance(data.get("eeprom_slots", []), list) else []
        names = [
            str(item.get("name", "")).strip()
            for item in slots
            if isinstance(item, dict) and str(item.get("name", "")).strip()
        ]
        return (names[: self.EEPROM_CONFIG_SLOT_COUNT] or self._default_eeprom_slot_names()) + [NEW_EEPROM_SLOT]

    def _settings_path(self) -> Path:
        return Path(__file__).resolve().with_suffix(".json")

    def _default_log_file_path(self) -> Path:
        return Path(__file__).resolve().with_suffix(".log")

    def _configured_log_file_path(self) -> Path:
        value = self._load_settings_data().get("log_file")
        if isinstance(value, str) and value.strip():
            return Path(value.strip()).expanduser()
        return self._default_log_file_path()

    def _load_config_fields(self) -> dict[str, object]:
        data = self._load_settings_data()
        fields = {
            "transport": data.get("transport", self.config.transport),
            "serial_port": data.get("serial_port", self.config.serial_port),
            "serial_baud": data.get("serial_baud", self.config.serial_baud),
            "uid": data.get("uid", self.config.uid),
            "target": data.get("target", self.config.target),
            "system_clock": data.get("system_clock", self.config.system_clock),
            "swo_clock": data.get("swo_clock", self.config.swo_clock),
            "swd_frequency": data.get("swd_frequency", self.config.swd_frequency),
            "connect_mode": data.get("connect_mode", self.config.connect_mode),
            "raw_port": data.get("raw_port", self.config.raw_port),
            "gdb_port": data.get("gdb_port", self.config.gdb_port),
            "log_file": data.get("log_file", str(self._default_log_file_path())),
            "graph_rpm": data.get("graph_rpm", True),
            "graph_pwm": data.get("graph_pwm", True),
            "graph_current": data.get("graph_current", True),
            "graph_voltage": data.get("graph_voltage", True),
            "graph_temperature": data.get("graph_temperature", True),
            "graph_integral": data.get("graph_integral", True),
            "graph_time_window": data.get("graph_time_window", 60),
        }
        return fields

    def _load_graph_time_window(self) -> int:
        data = self._load_settings_data()
        raw_value = data.get("graph_time_window", 60)
        try:
            return max(1, int(raw_value))
        except Exception:
            return 60

    def _save_config_fields(self, values: dict[str, object]) -> None:
        data = self._load_settings_data()
        for key in (
            "transport",
            "serial_port",
            "serial_baud",
            "uid",
            "target",
            "system_clock",
            "swo_clock",
            "swd_frequency",
            "connect_mode",
            "raw_port",
            "gdb_port",
            "log_file",
            "graph_rpm",
            "graph_pwm",
            "graph_current",
            "graph_voltage",
            "graph_temperature",
            "graph_integral",
            "graph_time_window",
        ):
            if key in values:
                if key == "graph_time_window":
                    data[key] = max(1, int(values[key]))
                elif key == "transport":
                    data[key] = str(values[key]).strip().lower()
                elif key == "log_file":
                    log_file = str(values[key]).strip()
                    data[key] = log_file or str(self._default_log_file_path())
                else:
                    data[key] = values[key]
        self._save_settings_data(data)

        self.config.transport = str(values.get("transport", self.config.transport)).strip().lower()
        self.config.serial_port = str(values.get("serial_port", self.config.serial_port))
        self.config.serial_baud = int(values.get("serial_baud", self.config.serial_baud))
        self.config.uid = str(values.get("uid", self.config.uid))
        self.config.target = str(values.get("target", self.config.target))
        self.config.system_clock = int(values.get("system_clock", self.config.system_clock))
        self.config.swo_clock = int(values.get("swo_clock", self.config.swo_clock))
        self.config.swd_frequency = int(values.get("swd_frequency", self.config.swd_frequency))
        self.config.connect_mode = str(values.get("connect_mode", self.config.connect_mode))
        self.config.raw_port = int(values.get("raw_port", self.config.raw_port))
        self.config.gdb_port = int(values.get("gdb_port", self.config.gdb_port))

        new_log_path = self._configured_log_file_path()
        if new_log_path != self.log_file_path:
            self._log_write_error_reported = False
            self._log_write_error_path = None
        self.log_file_path = new_log_path

    def _close_config_dialog(self) -> None:
        if self._config_dialog is not None:
            try:
                self._save_ui_state()
            except Exception:
                pass
            try:
                self._config_dialog.grab_release()
            except Exception:
                pass
            try:
                self._config_dialog.destroy()
            except Exception:
                pass
        self._config_dialog = None
        self._config_dialog_vars = {}

    def _save_config_dialog(self) -> None:
        if self._config_dialog is None:
            return

        try:
            previous_transport = str(self.config.transport).strip().lower()
            previous_window_seconds = self._load_graph_time_window()
            values = {}
            for key, var in self._config_dialog_vars.items():
                raw_value = var.get()
                if isinstance(raw_value, bool):
                    values[key] = raw_value
                elif isinstance(raw_value, str):
                    values[key] = raw_value.strip()
                else:
                    values[key] = raw_value
            self._save_config_fields(values)

            current_transport = str(self.config.transport).strip().lower()
            if current_transport != previous_transport:
                was_running = self.backend.running
                if was_running:
                    self._stop_monitoring(None)
                log_callback = lambda msg, log_type="INFO": self.event_queue.put(("log", (msg, log_type)))
                sample_callback = lambda sample: self.event_queue.put(("sample", sample))
                screenshot_event_callback = lambda kind, payload: self.event_queue.put((kind, payload))
                parameters_callback = lambda payload: self.event_queue.put(("serial-parameters", payload))
                eeprom_callback = lambda payload: self.event_queue.put(("serial-eeprom", payload))
                if current_transport == "serial":
                    self.backend = SerialBackend(
                        self.config,
                        log_callback=log_callback,
                        screenshot_path_factory=self._next_screenshot_path,
                        screenshot_event_callback=screenshot_event_callback,
                        sample_callback=sample_callback,
                        parameters_callback=parameters_callback,
                        eeprom_callback=eeprom_callback,
                    )
                else:
                    self.backend = SWOBackend(
                        self.config,
                        log_callback=log_callback,
                        screenshot_path_factory=self._next_screenshot_path,
                        screenshot_event_callback=screenshot_event_callback,
                        sample_callback=sample_callback,
                    )
                self._append_log(f"Switched connection type to {self.config.transport}", "INFO")

            if "graph_time_window" in values:
                current_window_seconds = int(values["graph_time_window"])
                if current_window_seconds != previous_window_seconds:
                    self.window_seconds_var.set(str(current_window_seconds))
                    self._apply_window()
            self._apply_graph_visibility()
            self._append_log(f"Saved config to {self._settings_path().name}", "INFO")
        except Exception as exc:
            self._append_log(f"Failed to save config: {exc}", "ERROR")
        finally:
            self._close_config_dialog()

    def _open_config_dialog(self) -> None:
        if self._config_dialog is not None:
            self._config_dialog.lift()
            self._config_dialog.focus_force()
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("Config")
        dialog.transient(self.root)
        dialog.grab_set()
        dialog.minsize(520, 420)
        dialog.protocol("WM_DELETE_WINDOW", self._close_config_dialog)
        self._config_dialog = dialog

        ui_state = self._load_ui_state()
        config_geometry = ui_state.get("config_geometry") if isinstance(ui_state, dict) else None
        if isinstance(config_geometry, str) and config_geometry:
            try:
                dialog.geometry(config_geometry)
            except Exception:
                self._position_dialog_centered(dialog, width=620, height=460)
        else:
            self._position_dialog_centered(dialog, width=620, height=460)

        values = self._load_config_fields()
        self._config_dialog_vars = {}

        outer = ttk.Frame(dialog, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)

        transport_frame = ttk.LabelFrame(outer, text="Connection Type")
        transport_frame.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        transport_frame.columnconfigure(1, weight=1)
        ttk.Label(transport_frame, text="Transport:").grid(row=0, column=0, padx=(8, 6), pady=4, sticky="w")
        transport_value = str(values.get("transport", "")).strip().lower()
        transport_display = {"serial": "Serial", "swo": "SWO"}.get(transport_value, transport_value)
        var = tk.StringVar(value=transport_display)
        self._config_dialog_vars["transport"] = var
        widget = ttk.Combobox(transport_frame, textvariable=var, values=["Serial", "SWO"], state="readonly", width=24)
        widget.grid(row=0, column=1, padx=(0, 8), pady=4, sticky="ew")

        inner = ttk.Frame(outer)
        inner.grid(row=1, column=0, sticky="nsew")
        inner.columnconfigure(0, weight=1)
        inner.columnconfigure(1, weight=1)

        serial_frame = ttk.LabelFrame(inner, text="Serial")
        serial_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=(0, 8))
        serial_frame.columnconfigure(1, weight=1)

        swo_frame = ttk.LabelFrame(inner, text="SWO")
        swo_frame.grid(row=0, column=1, sticky="nsew", padx=(8, 0), pady=(0, 8))
        swo_frame.columnconfigure(1, weight=1)

        serial_fields = (
            ("serial_port", "COM port / USB device ID", "entry", None),
            ("serial_baud", "Serial baud", "entry", None),
        )
        swo_fields = (
            ("connect_mode", "Connect mode", "choice", ["halt", "pre-reset", "under-reset", "attach"]),
            ("swo_clock", "SWO clock", "entry", None),
            ("swd_frequency", "SWD freq", "entry", None),
            ("raw_port", "SWV raw port", "entry", None),
            ("gdb_port", "GDB port", "entry", None),
        )

        for row_index, (key, label_text, kind, options) in enumerate(serial_fields):
            ttk.Label(serial_frame, text=f"{label_text}:").grid(row=row_index, column=0, padx=(8, 6), pady=4, sticky="w")
            var = tk.StringVar(value=str(values.get(key, "")))
            self._config_dialog_vars[key] = var
            if kind == "choice" and options is not None:
                widget = ttk.Combobox(serial_frame, textvariable=var, values=options, state="readonly", width=22)
            else:
                widget = ttk.Entry(serial_frame, textvariable=var, width=24)
            widget.grid(row=row_index, column=1, padx=(0, 8), pady=4, sticky="ew")

        for row_index, (key, label_text, kind, options) in enumerate(swo_fields):
            ttk.Label(swo_frame, text=f"{label_text}:").grid(row=row_index, column=0, padx=(8, 6), pady=4, sticky="w")
            var = tk.StringVar(value=str(values.get(key, "")))
            self._config_dialog_vars[key] = var
            if kind == "choice" and options is not None:
                widget = ttk.Combobox(swo_frame, textvariable=var, values=options, state="readonly", width=22)
            else:
                widget = ttk.Entry(swo_frame, textvariable=var, width=24)
            widget.grid(row=row_index, column=1, padx=(0, 8), pady=4, sticky="ew")

        log_frame = ttk.LabelFrame(outer, text="Log file")
        log_frame.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        log_frame.columnconfigure(1, weight=1)
        ttk.Label(log_frame, text="Filename:").grid(row=0, column=0, padx=(8, 6), pady=6, sticky="w")
        log_file_var = tk.StringVar(value=str(values.get("log_file", self._default_log_file_path())))
        self._config_dialog_vars["log_file"] = log_file_var
        ttk.Entry(log_frame, textvariable=log_file_var).grid(
            row=0, column=1, padx=(0, 8), pady=6, sticky="ew"
        )

        graph_frame = ttk.LabelFrame(outer, text="Graphs")
        graph_frame.grid(row=3, column=0, sticky="ew", pady=(12, 0))
        graph_frame.columnconfigure(0, weight=1)
        graph_frame.columnconfigure(1, weight=1)

        graph_specs = (
            ("graph_rpm", "RPM", 0, 0),
            ("graph_pwm", "PWM", 0, 1),
            ("graph_current", "Current", 1, 0),
            ("graph_voltage", "Voltage", 1, 1),
            ("graph_temperature", "Temperature", 2, 0),
            ("graph_integral", "Integral", 2, 1),
        )

        for key, label_text, row, col in graph_specs:
            var = tk.BooleanVar(value=bool(values.get(key, True)))
            self._config_dialog_vars[key] = var
            widget = ttk.Checkbutton(graph_frame, text=label_text, variable=var)
            widget.grid(row=row, column=col, padx=8, pady=4, sticky="w")

        ttk.Label(graph_frame, text="Time window (s):").grid(row=3, column=0, padx=8, pady=(12, 4), sticky="w")
        time_window_var = tk.StringVar(value=str(values.get("graph_time_window", 60)))
        self._config_dialog_vars["graph_time_window"] = time_window_var
        ttk.Entry(graph_frame, textvariable=time_window_var, width=12).grid(row=3, column=1, padx=(0, 8), pady=(12, 4), sticky="w")

        buttons = ttk.Frame(dialog)
        buttons.pack(fill=tk.X, padx=12, pady=(0, 12))
        ttk.Button(buttons, text="Cancel", command=self._close_config_dialog).pack(side=tk.RIGHT)
        ttk.Button(buttons, text="Save", command=self._save_config_dialog).pack(side=tk.RIGHT, padx=(0, 6))

        dialog.bind("<Configure>", lambda _: self._save_ui_state())

    def _load_settings_data(self) -> dict[str, object]:
        config_path = self._settings_path()
        if not config_path.exists():
            return {}

        try:
            with config_path.open("r", encoding="utf-8") as handle:
                data = json.load(handle)
        except Exception:
            return {}

        return data if isinstance(data, dict) else {}

    def _save_settings_data(self, data: dict[str, object]) -> None:
        config_path = self._settings_path()
        config_path.parent.mkdir(parents=True, exist_ok=True)
        with config_path.open("w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
            handle.write("\n")

    def _load_ui_state(self) -> dict[str, object]:
        data = self._load_settings_data()
        ui = data.get("ui", {}) if isinstance(data.get("ui", {}), dict) else {}
        return ui

    def _save_ui_state(self) -> None:
        data = self._load_settings_data()
        ui = data.get("ui", {}) if isinstance(data.get("ui", {}), dict) else {}
        if self.root is not None:
            try:
                ui["main_geometry"] = self.root.winfo_geometry()
            except Exception:
                pass
        if self._config_dialog is not None:
            try:
                ui["config_geometry"] = self._config_dialog.winfo_geometry()
            except Exception:
                pass
        if self._eeprom_dialog is not None:
            try:
                ui["eeprom_geometry"] = self._eeprom_dialog.winfo_geometry()
            except Exception:
                pass
        if self.panes is not None:
            try:
                ui["main_panes"] = [self.panes.sashpos(0)]
            except Exception:
                pass
        if self.right_panes is not None:
            try:
                ui["right_panes"] = [self.right_panes.sashpos(0), self.right_panes.sashpos(1)]
            except Exception:
                pass
        data["ui"] = ui
        self._save_settings_data(data)

    def _restore_ui_state(self) -> None:
        ui_state = self._load_ui_state()
        geometry = ui_state.get("main_geometry")
        if isinstance(geometry, str) and geometry:
            try:
                self.root.geometry(geometry)
            except Exception:
                pass

        main_panes = ui_state.get("main_panes")
        if isinstance(main_panes, list) and len(main_panes) >= 1:
            try:
                self.panes.sashpos(0, int(main_panes[0]))
            except Exception:
                pass

        right_panes = ui_state.get("right_panes")
        if isinstance(right_panes, list) and len(right_panes) >= 2:
            try:
                self.right_panes.sashpos(0, int(right_panes[0]))
                self.right_panes.sashpos(1, int(right_panes[1]))
            except Exception:
                pass

        eeprom_geometry = ui_state.get("eeprom_geometry")
        if isinstance(eeprom_geometry, str) and eeprom_geometry and self._eeprom_dialog is not None:
            try:
                self._eeprom_dialog.geometry(eeprom_geometry)
            except Exception:
                pass

    def _snapshot_eeprom_dialog_as_slot_dict(self, slot_name: str) -> dict[str, object]:
        if self._eeprom_dialog_source is None:
            raise RuntimeError("EEPROM data is not loaded yet")

        values = self._read_eeprom_dialog_values()
        return {
            "name": slot_name,
            "values": {
                field_name: getattr(values, field_name)
                for _, field_name, _, _, _, _ in EEPROM_FIELD_SPECS
            },
        }

    def _save_eeprom_config_file(self) -> None:
        if self._eeprom_dialog is None:
            return

        current_slot_name = (
            self._eeprom_dialog_slot_var.get()
            if self._eeprom_dialog_slot_var is not None
            else "slot_1"
        ).strip() or "slot_1"
        is_new_slot = current_slot_name == NEW_EEPROM_SLOT
        slot_name = simpledialog.askstring(
            "Store EEPROM Config",
            "Name for the new slot:" if is_new_slot else "Name of the slot to replace:",
            initialvalue="" if is_new_slot else current_slot_name,
            parent=self._eeprom_dialog,
        )
        if slot_name is None:
            return
        slot_name = slot_name.strip()
        if not slot_name or slot_name == NEW_EEPROM_SLOT:
            if self._eeprom_dialog_status_var is not None:
                self._eeprom_dialog_status_var.set("EEPROM slot name is invalid")
            return

        data = self._load_settings_data()
        slots = data.get("eeprom_slots", []) if isinstance(data.get("eeprom_slots", []), list) else []

        slot_map = {str(item.get("name", "")): item for item in slots if isinstance(item, dict)}
        if not is_new_slot and slot_name != current_slot_name:
            slot_map.pop(current_slot_name, None)
        slot_map[slot_name] = self._snapshot_eeprom_dialog_as_slot_dict(slot_name)

        ordered_slots = []
        for existing_name in self._default_eeprom_slot_names():
            if existing_name in slot_map:
                ordered_slots.append(slot_map[existing_name])
        for name, item in slot_map.items():
            if name not in {slot["name"] for slot in ordered_slots}:
                ordered_slots.append(item)

        data["eeprom_slots"] = ordered_slots[: self.EEPROM_CONFIG_SLOT_COUNT]
        self._save_settings_data(data)

        if self._eeprom_dialog_slot_combo is not None:
            slot_names = [str(slot["name"]) for slot in data["eeprom_slots"]] + [NEW_EEPROM_SLOT]
            self._eeprom_dialog_slot_combo.configure(values=slot_names)
        if self._eeprom_dialog_slot_var is not None:
            self._eeprom_dialog_slot_var.set(slot_name)

        if self._eeprom_dialog_status_var is not None:
            self._eeprom_dialog_status_var.set(f"Saved EEPROM config to {self._settings_path().name}")

    def _load_eeprom_config_file(self) -> None:
        if self._eeprom_dialog is None:
            return

        data = self._load_settings_data()
        slots = data.get("eeprom_slots", []) if isinstance(data.get("eeprom_slots", []), list) else []
        if not slots:
            if self._eeprom_dialog_status_var is not None:
                self._eeprom_dialog_status_var.set(f"No EEPROM config file found at {self._settings_path().name}")
            return

        slot_name = (self._eeprom_dialog_slot_var.get() if self._eeprom_dialog_slot_var is not None else "slot_1").strip()
        if not slot_name:
            slot_name = "slot_1"

        selected = None
        for item in slots:
            if not isinstance(item, dict):
                continue
            if str(item.get("name", "")) == slot_name:
                selected = item
                break

        if selected is None:
            if self._eeprom_dialog_status_var is not None:
                self._eeprom_dialog_status_var.set(f"No EEPROM preset named '{slot_name}' found")
            return

        values = selected.get("values", {})
        if not isinstance(values, dict):
            if self._eeprom_dialog_status_var is not None:
                self._eeprom_dialog_status_var.set("EEPROM preset values are invalid")
            return

        for spec in EEPROM_FIELD_SPECS:
            _, field_name, field_kind, _, _, _ = spec
            if field_name not in values:
                continue
            value = values[field_name]
            if field_kind == "choice":
                choices = spec[5] or ()
                label = next((label for label, candidate in choices if candidate == value), str(value))
                self._eeprom_dialog_vars[field_name].set(label)
            elif field_kind == "percent":
                self._eeprom_dialog_vars[field_name].set(f"{anti_windup_raw_to_percent(int(value)):.2f}")
            else:
                self._eeprom_dialog_vars[field_name].set(str(value))

        if self._eeprom_dialog_status_var is not None:
            self._eeprom_dialog_status_var.set(f"Loaded EEPROM preset '{slot_name}'")

    def _delete_eeprom_config_file(self) -> None:
        if self._eeprom_dialog is None:
            return

        slot_name = (
            self._eeprom_dialog_slot_var.get()
            if self._eeprom_dialog_slot_var is not None
            else ""
        ).strip()
        if not slot_name or slot_name == NEW_EEPROM_SLOT:
            if self._eeprom_dialog_status_var is not None:
                self._eeprom_dialog_status_var.set("Select a saved EEPROM config to delete")
            return

        if not messagebox.askyesno(
            "Delete EEPROM Config",
            f"Delete the EEPROM config '{slot_name}'?",
            parent=self._eeprom_dialog,
        ):
            return

        data = self._load_settings_data()
        slots = data.get("eeprom_slots", []) if isinstance(data.get("eeprom_slots", []), list) else []
        remaining_slots = [
            item
            for item in slots
            if not (isinstance(item, dict) and str(item.get("name", "")) == slot_name)
        ]
        if len(remaining_slots) == len(slots):
            if self._eeprom_dialog_status_var is not None:
                self._eeprom_dialog_status_var.set(f"EEPROM preset '{slot_name}' was not found")
            return

        data["eeprom_slots"] = remaining_slots[: self.EEPROM_CONFIG_SLOT_COUNT]
        self._save_settings_data(data)

        saved_names = [str(item["name"]) for item in data["eeprom_slots"]]
        slot_names = saved_names + [NEW_EEPROM_SLOT]
        if self._eeprom_dialog_slot_combo is not None:
            self._eeprom_dialog_slot_combo.configure(values=slot_names)
        if self._eeprom_dialog_slot_var is not None:
            self._eeprom_dialog_slot_var.set(saved_names[0] if saved_names else NEW_EEPROM_SLOT)
        if self._eeprom_dialog_status_var is not None:
            self._eeprom_dialog_status_var.set(f"Deleted EEPROM preset '{slot_name}'")

    def _populate_eeprom_dialog(
        self,
        data: EEPROMData,
        swo_data: Optional[SWOData] = None,
    ) -> None:
        if self._eeprom_dialog is None:
            return

        self._eeprom_dialog_source = data
        self._eeprom_dialog_address = swo_data.eeprom_address if swo_data is not None else None

        summary_vars = self._eeprom_dialog_summary_vars
        summary_vars["magic"].set(f"0x{data.magic:08X}")
        summary_vars["version"].set(str(data.version))
        summary_vars["sequence"].set(str(data.sequence))
        summary_vars["crc"].set(f"0x{data.crc:08X}")
        summary_vars["address"].set(
            f"0x{self._eeprom_dialog_address:08X}" if self._eeprom_dialog_address is not None else "serial"
        )
        summary_vars["commit"].set(
            ("set" if swo_data.eeprom_commit else "clear") if swo_data is not None else "n/a"
        )

        for spec in EEPROM_FIELD_SPECS:
            _, field_name, _, _, _, _ = spec
            self._eeprom_dialog_vars[field_name].set(self._format_eeprom_field_value(spec, data))

        if self._eeprom_dialog_status_var is not None:
            self._eeprom_dialog_status_var.set("Loaded EEPROM data")

        self._set_eeprom_dialog_editable(True)

    def _read_eeprom_dialog_values(self) -> EEPROMData:
        if self._eeprom_dialog_source is None:
            raise RuntimeError("EEPROM data not loaded yet")

        values = {
            "magic": self._eeprom_dialog_source.magic,
            "version": self._eeprom_dialog_source.version,
            "sequence": self._eeprom_dialog_source.sequence,
            "crc": self._eeprom_dialog_source.crc,
        }

        for spec in EEPROM_FIELD_SPECS:
            _, field_name, field_kind, minimum, maximum, choices = spec
            raw_text = self._eeprom_dialog_vars[field_name].get().strip()

            if field_kind == "choice":
                if choices is None:
                    raise RuntimeError(f"No choices defined for {field_name}")
                mapping = {label: value for label, value in choices}
                if raw_text not in mapping:
                    raise ValueError(f"Invalid value for {field_name}: {raw_text}")
                values[field_name] = mapping[raw_text]
                continue

            if field_kind == "float":
                parsed_value = float(raw_text)
            elif field_kind == "percent":
                parsed_percent = float(raw_text)
                if minimum is not None and parsed_percent < minimum:
                    raise ValueError(f"{field_name} below minimum {minimum}")
                if maximum is not None and parsed_percent > maximum:
                    raise ValueError(f"{field_name} above maximum {maximum}")
                values[field_name] = anti_windup_percent_to_raw(parsed_percent)
                continue
            else:
                parsed_value = int(raw_text)

            if minimum is not None and parsed_value < minimum:
                raise ValueError(f"{field_name} below minimum {minimum}")
            if maximum is not None and parsed_value > maximum:
                raise ValueError(f"{field_name} above maximum {maximum}")

            values[field_name] = parsed_value

        return EEPROMData(**values)

    def _position_dialog_centered(self, dialog: tk.Toplevel, width: int, height: int) -> None:
        # Place modal near the parent window center so it does not appear at desktop origin.
        self.root.update_idletasks()
        root_width = self.root.winfo_width()
        root_height = self.root.winfo_height()
        root_x = self.root.winfo_rootx()
        root_y = self.root.winfo_rooty()

        if root_width <= 1 or root_height <= 1:
            # Fallback when root geometry is not available yet.
            dialog.geometry(f"{width}x{height}")
            return

        pos_x = root_x + max(0, (root_width - width) // 2)
        pos_y = root_y + max(0, (root_height - height) // 2)
        dialog.geometry(f"{width}x{height}+{pos_x}+{pos_y}")

    def _open_eeprom_dialog(self) -> None:
        if self._eeprom_dialog is not None:
            self._eeprom_dialog.lift()
            self._eeprom_dialog.focus_force()
            return

        if not self.backend.running:
            self._append_log("EEPROM editor is only available while monitoring is started", "WARNING")
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("EEPROM Editor")
        dialog.transient(self.root)
        dialog.grab_set()
        dialog.minsize(980, 560)
        dialog.geometry("1080x640")
        dialog.protocol("WM_DELETE_WINDOW", self._close_eeprom_dialog)
        self._eeprom_dialog = dialog

        ui_state = self._load_ui_state()
        eeprom_geometry = ui_state.get("eeprom_geometry") if isinstance(ui_state, dict) else None
        if isinstance(eeprom_geometry, str) and eeprom_geometry:
            try:
                dialog.geometry(eeprom_geometry)
            except Exception:
                self._position_dialog_centered(dialog, width=1080, height=640)
        else:
            self._position_dialog_centered(dialog, width=1080, height=640)

        dialog.columnconfigure(0, weight=1)
        dialog.rowconfigure(1, weight=1)

        summary_frame = ttk.LabelFrame(dialog, text="EEPROM Header")
        summary_frame.grid(row=0, column=0, sticky="ew", padx=12, pady=(12, 6))
        summary_frame.columnconfigure(1, weight=1)
        summary_frame.columnconfigure(3, weight=1)
        summary_frame.columnconfigure(5, weight=1)

        self._eeprom_dialog_summary_vars = {
            "magic": tk.StringVar(value="..."),
            "version": tk.StringVar(value="..."),
            "sequence": tk.StringVar(value="..."),
            "crc": tk.StringVar(value="..."),
            "address": tk.StringVar(value="..."),
            "commit": tk.StringVar(value="..."),
        }

        header_items = (
            ("Magic", "magic", 0),
            ("Version", "version", 2),
            ("Sequence", "sequence", 4),
            ("CRC", "crc", 0),
            ("RAM Address", "address", 2),
            ("Commit Flag", "commit", 4),
        )
        for label_text, key, column in header_items[:3]:
            row = 0
            ttk.Label(summary_frame, text=f"{label_text}:").grid(row=row, column=column, padx=(8, 4), pady=4, sticky="w")
            ttk.Label(summary_frame, textvariable=self._eeprom_dialog_summary_vars[key]).grid(row=row, column=column + 1, padx=(0, 12), pady=4, sticky="w")
        for label_text, key, column in header_items[3:]:
            row = 1
            ttk.Label(summary_frame, text=f"{label_text}:").grid(row=row, column=column, padx=(8, 4), pady=4, sticky="w")
            ttk.Label(summary_frame, textvariable=self._eeprom_dialog_summary_vars[key]).grid(row=row, column=column + 1, padx=(0, 12), pady=4, sticky="w")

        form_frame = ttk.LabelFrame(dialog, text="Editable Fields")
        form_frame.grid(row=1, column=0, sticky="nsew", padx=12, pady=6)
        form_frame.columnconfigure(1, weight=1)
        form_frame.columnconfigure(3, weight=1)

        self._eeprom_dialog_vars = {}
        self._eeprom_dialog_widgets = []

        for row_index, start_index in enumerate(range(0, len(EEPROM_FIELD_SPECS), 2)):
            for column_offset, spec in enumerate(EEPROM_FIELD_SPECS[start_index : start_index + 2]):
                label_text, field_name, field_kind, _, _, choices = spec
                column = column_offset * 2
                ttk.Label(form_frame, text=f"{label_text}:").grid(row=row_index, column=column, padx=(8, 4), pady=4, sticky="w")

                field_var = tk.StringVar(value="...")
                self._eeprom_dialog_vars[field_name] = field_var

                if field_kind == "choice" and choices is not None:
                    widget = ttk.Combobox(
                        form_frame,
                        textvariable=field_var,
                        values=[label for label, _ in choices],
                        width=22,
                        state=tk.DISABLED,
                    )
                    widget.configure(state=tk.DISABLED)
                    widget_kind = "choice"
                else:
                    widget = ttk.Entry(form_frame, textvariable=field_var, width=24, state=tk.DISABLED)
                    widget_kind = "entry"

                widget.grid(row=row_index, column=column + 1, padx=(0, 12), pady=4, sticky="ew")
                self._eeprom_dialog_widgets.append((widget, widget_kind))

        button_frame = ttk.Frame(dialog)
        button_frame.grid(row=2, column=0, sticky="ew", padx=12, pady=(6, 12))
        button_frame.columnconfigure(0, weight=1)

        self._eeprom_dialog_status_var = tk.StringVar(value="Loading EEPROM data...")
        ttk.Label(button_frame, textvariable=self._eeprom_dialog_status_var).grid(row=0, column=0, sticky="w")

        slot_row = ttk.Frame(button_frame)
        slot_row.grid(row=1, column=1, sticky="e")
        ttk.Label(slot_row, text="Slot:").pack(side=tk.LEFT, padx=(0, 6))
        slot_names = self._eeprom_slot_names()
        self._eeprom_dialog_slot_var = tk.StringVar(value=slot_names[0])
        self._eeprom_dialog_slot_combo = ttk.Combobox(
            slot_row,
            textvariable=self._eeprom_dialog_slot_var,
            values=slot_names,
            state="readonly",
            width=18,
        )
        self._eeprom_dialog_slot_combo.pack(side=tk.LEFT, padx=(0, 8))
        self._eeprom_dialog_save_button = ttk.Button(slot_row, text="Store", command=self._save_eeprom_config_file)
        self._eeprom_dialog_save_button.pack(side=tk.LEFT, padx=(0, 6))
        self._eeprom_dialog_load_button = ttk.Button(slot_row, text="Load", command=self._load_eeprom_config_file)
        self._eeprom_dialog_load_button.pack(side=tk.LEFT, padx=(0, 6))
        self._eeprom_dialog_delete_button = ttk.Button(
            slot_row,
            text="Delete",
            command=self._delete_eeprom_config_file,
        )
        self._eeprom_dialog_delete_button.pack(side=tk.LEFT)

        button_row = ttk.Frame(button_frame)
        button_row.grid(row=2, column=1, sticky="e", pady=(8, 0))
        self._eeprom_dialog_commit_button = ttk.Button(button_row, text="Commit", command=self._commit_eeprom_dialog, state=tk.DISABLED)
        self._eeprom_dialog_commit_button.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(button_row, text="Cancel", command=self._close_eeprom_dialog).pack(side=tk.LEFT)

        dialog.bind("<Configure>", lambda _: self._save_ui_state())
        self._set_eeprom_dialog_editable(False)

        if self.config.transport == "serial":
            try:
                if not isinstance(self.backend, SerialBackend):
                    raise RuntimeError("Serial backend is not available")
                self.backend.request_eeprom()
            except Exception as exc:
                self.event_queue.put(("eeprom-load-error", str(exc)))
            return

        def worker() -> None:
            try:
                swo_payload = self.gdb_mem.read_memory(self.data_address, SWO_DATA_SIZE)
                swo_data = self._validate_swo_payload(swo_payload)
                eeprom_payload = self.gdb_mem.read_memory(swo_data.eeprom_address, EEPROM_DATA_SIZE)
                eeprom_data = self._unpack_eeprom_data(eeprom_payload)
                self.event_queue.put(("eeprom-load", (swo_data, eeprom_data)))
            except Exception as exc:
                self.event_queue.put(("eeprom-load-error", str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def _commit_eeprom_dialog(self) -> None:
        if self._eeprom_dialog_source is None:
            if self._eeprom_dialog_status_var is not None:
                self._eeprom_dialog_status_var.set("EEPROM data is not loaded yet")
            return

        try:
            eeprom_data = self._read_eeprom_dialog_values()
        except Exception as exc:
            if self._eeprom_dialog_status_var is not None:
                self._eeprom_dialog_status_var.set(f"Invalid EEPROM input: {exc}")
            return

        self._set_eeprom_dialog_editable(False)
        if self._eeprom_dialog_status_var is not None:
            self._eeprom_dialog_status_var.set("Committing EEPROM data...")

        if self.config.transport == "serial":
            if not isinstance(self.backend, SerialBackend):
                if self._eeprom_dialog_status_var is not None:
                    self._eeprom_dialog_status_var.set("Serial backend is not available")
                self._set_eeprom_dialog_editable(True)
                return

            def serial_worker() -> None:
                try:
                    self.backend._send_binary_payload(BINARY_TYPE_EEPROM, self._pack_eeprom_data(eeprom_data))
                    self.event_queue.put(("eeprom-commit-complete", eeprom_data))
                except Exception as exc:
                    self.event_queue.put(("eeprom-commit-error", str(exc)))

            threading.Thread(target=serial_worker, daemon=True).start()
            return

        if self._eeprom_dialog_address is None:
            if self._eeprom_dialog_status_var is not None:
                self._eeprom_dialog_status_var.set("EEPROM data address is not available")
            self._set_eeprom_dialog_editable(True)
            return

        def worker() -> None:
            try:
                self.gdb_mem.write_memory(self._eeprom_dialog_address, self._pack_eeprom_data(eeprom_data))
                if self.data_address is None:
                    raise RuntimeError("SWO data address not available")
                self.gdb_mem.write_memory(
                    self.data_address + SWO_DATA_EEPROM_COMMIT_OFFSET,
                    b"\x01",
                )
                self.event_queue.put(("eeprom-commit-complete", eeprom_data))
            except Exception as exc:
                self.event_queue.put(("eeprom-commit-error", str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def _cancel_startup_sync_job(self) -> None:
        if self._startup_sync_job is None:
            return
        try:
            self.root.after_cancel(self._startup_sync_job)
        except Exception:
            pass
        self._startup_sync_job = None

    def _request_read_swo_data(self, reason: str) -> None:
        if self.config.transport != "swo":
            return
        if self.data_address is None:
            self._append_log("Cannot read SWO::data yet: dataAddress not available", "WARNING")
            return
        if self.sync_in_progress:
            return

        self.sync_in_progress = True

        def worker() -> None:
            try:
                payload = self.gdb_mem.read_memory(self.data_address, SWO_DATA_SIZE)
                data = self._validate_swo_payload(payload)
                # The SWO::data current/level mirror can be stale from a previous session
                # (the target keeps running under pyocd attach/persist). The EEPROM holds
                # the values actually in effect, so show those instead for display reads.
                try:
                    eeprom_payload = self.gdb_mem.read_memory(data.eeprom_address, EEPROM_DATA_SIZE)
                    eeprom_data = self._unpack_eeprom_data(eeprom_payload)
                    data.input_current_limit = eeprom_data.input_current_limit
                    data.current_limit_level = eeprom_data.current_limit_level
                except Exception as exc:
                    self.event_queue.put(("log", f"EEPROM read for current/level failed: {exc}"))
                self.event_queue.put(("swo-read", (data, reason)))
            except Exception as exc:
                self.event_queue.put(("log", f"Read SWO::data failed: {exc}"))
                self.event_queue.put(("swo-read-invalid", reason))
                if reason == "startup":
                    self.event_queue.put(("startup-sync-retry", None))
                self.event_queue.put(("sync-done", None))

        threading.Thread(target=worker, daemon=True).start()

    def _sync_to_target(self) -> None:
        if self.config.transport not in ("swo", "serial"):
            self._append_log("PID parameter sync requires SWO or serial transport", "ERROR")
            return
        if self.config.transport == "swo" and self.data_address is None:
            self._append_log("Cannot sync: no dataAddress received yet", "WARNING")
            return
        if self.sync_in_progress:
            self._append_log("Sync already in progress", "WARNING")
            return

        # Read-only sync unless user edited fields. This prevents accidental
        # writes of defaults and keeps Sync useful for refresh.
        if not self._pid_fields_dirty:
            if self.config.transport == "swo":
                self._request_read_swo_data("manual")
            else:
                if not isinstance(self.backend, SerialBackend):
                    self._append_log("Serial backend is not available", "ERROR")
                    return
                try:
                    self.backend._send_binary_command(BINARY_TYPE_REQUEST_PARAMETERS)
                    self._append_log("Requested PID parameters from serial target", "INFO")
                except Exception as exc:
                    self._append_log(f"Serial PID parameter request failed: {exc}", "ERROR")
            return

        try:
            kp = float(self.kp_var.get().strip())
            ki = float(self.ki_var.get().strip())
            kd = float(self.kd_var.get().strip())
            anti_windup = float(self.anti_windup_var.get().strip())
            if anti_windup < 0.0 or anti_windup > 100.0:
                raise ValueError("Anti-windup out of range (0.00..100.00)")
            rpm = int(self.rpm_var.get().strip())
            if rpm < 0 or rpm > MAX_RPM:
                raise ValueError("RPM out of range (0..%u)" % MAX_RPM)
            input_current_limit = int(self.input_current_limit_var.get().strip())
            if input_current_limit < 0 or input_current_limit > MAX_INPUT_CURRENT_LIMIT:
                raise ValueError("Current limit out of range (0..%u)" % MAX_INPUT_CURRENT_LIMIT)
            current_limit_level = current_limit_level_name_to_int(self.current_limit_level_var.get())
        except Exception as exc:
            self._append_log(f"Invalid PID input: {exc}", "ERROR")
            return

        if self.config.transport == "serial":
            if not isinstance(self.backend, SerialBackend):
                self._append_log("Serial backend is not available", "ERROR")
                return

            payload = struct.pack(
                PID_PARAMETERS_STRUCT,
                kp,
                ki,
                kd,
                anti_windup_percent_to_raw(anti_windup),
                rpm,
                input_current_limit,
                current_limit_level,
            )
            self.sync_in_progress = True

            def serial_worker() -> None:
                try:
                    self.backend._send_binary_payload(BINARY_TYPE_PARAMETERS, payload)
                    self.event_queue.put(("log", "Sent PID parameters to serial target"))
                except Exception as exc:
                    self.event_queue.put(("log", f"Serial PID parameter sync failed: {exc}"))
                finally:
                    self.event_queue.put(("sync-done", None))

            threading.Thread(target=serial_worker, daemon=True).start()
            return

        self.sync_in_progress = True
        data = SWOData(
            kp=kp,
            ki=ki,
            kd=kd,
            anti_windup=anti_windup_percent_to_raw(anti_windup),
            rpm=rpm,
            enabled_state=SWO_ENABLE_DISABLED,
            changed=True,
            eeprom_address=0,
            eeprom_commit=False,
            send_screenshot=False,
            input_current_limit=input_current_limit,
            current_limit_level=current_limit_level,
        )

        def worker() -> None:
            try:
                current_payload = self.gdb_mem.read_memory(self.data_address, SWO_DATA_SIZE)
                current_data = self._validate_swo_payload(current_payload)
                data.enabled_state = current_data.enabled_state
                data.eeprom_address = current_data.eeprom_address
                data.eeprom_commit = current_data.eeprom_commit
                data.send_screenshot = current_data.send_screenshot
                self.gdb_mem.write_memory(
                    self.data_address,
                    self._pack_swo_data(data),
                )
                self.event_queue.put(
                    ("log", f"Synced PID params to SWO::data (changed=true) "
                     f"Current={data.input_current_limit}mA Level={data.current_limit_level}")
                )
                # Read back once after write for confirmation.
                payload = self.gdb_mem.read_memory(self.data_address, SWO_DATA_SIZE)
                data_verify = self._validate_swo_payload(payload)
                self.event_queue.put(("swo-read", (data_verify, "after write")))
            except Exception as exc:
                self.event_queue.put(("log", f"Write SWO::data failed: {exc}"))
                self.event_queue.put(("sync-done", None))

        threading.Thread(target=worker, daemon=True).start()

    def _init_buffers(self, window_seconds: int) -> None:
        self.window_seconds = max(1, int(window_seconds))
        self.samples_per_window = max(16, self.window_seconds * PID_SAMPLE_HZ_DEFAULT)
        self.filled = 0
        self._plot_dirty = False
        self._last_plot_refresh = 0.0
        self._rpm_sum = 0.0
        self._pwm_sum = 0.0

        if self.samples_per_window > 1:
            dt = self.window_seconds / float(self.samples_per_window - 1)
            self.x_values = [(-self.window_seconds + i * dt) for i in range(self.samples_per_window)]
        else:
            self.x_values = [0.0]

        self.rpm = self._make_series(self.samples_per_window)
        self.rpm_avg = self._make_series(self.samples_per_window)
        self.pwm = self._make_series(self.samples_per_window)
        self.pwm_avg = self._make_series(self.samples_per_window)
        self.current_avg_ma = self._make_series(self.samples_per_window)
        self.current_ocp_ma = self._make_series(self.samples_per_window)
        self.dac_motor_current_ma = self._make_series(self.samples_per_window)
        self.dac_input_current_ma = self._make_series(self.samples_per_window)
        self.voltage_mv = self._make_series(self.samples_per_window)
        self.motor_temp_c = self._make_series(self.samples_per_window)
        self.mosfet_temp_c = self._make_series(self.samples_per_window)
        self.error = self._make_series(self.samples_per_window)
        self.integral = self._make_series(self.samples_per_window)
        self.derivative = self._make_series(self.samples_per_window)

        if hasattr(self, "axes"):
            self._apply_x_grid()

    def _build_plot_lines(self) -> None:
        visibility = getattr(self, 'graph_visibility', {
            'rpm': True, 'pwm': True, 'current': True,
            'voltage': True, 'temperature': True, 'integral': True
        })

        ax_map = {}
        axis_idx = 0
        for graph_type in ['rpm', 'pwm', 'current', 'voltage', 'temperature', 'integral']:
            if visibility.get(graph_type, True):
                ax_map[graph_type] = self.axes[axis_idx]
                axis_idx += 1

        if 'rpm' in ax_map:
            ax0 = ax_map['rpm']
            (self.line_rpm,) = ax0.plot(self.x_values, self.rpm, label="RPM", color="#0077B6")
            (self.line_rpm_avg,) = ax0.plot(self.x_values, self.rpm_avg, label="Avg RPM", color="#E85D04", linestyle="--")
            ax0.legend(loc="upper left")
            # Live Avg RPM readout shown at the bottom of the graph.
            self.rpm_text = ax0.text(
                0.02,
                0.07,
                "Avg RPM: --",
                transform=ax0.transAxes,
                ha="left",
                va="bottom",
                fontsize=11,
                fontweight="bold",
                color="#0077B6",
                bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="#0077B6", alpha=0.85),
            )
        else:
            self.rpm_text = None

        if 'pwm' in ax_map:
            ax1 = ax_map['pwm']
            (self.line_pwm,) = ax1.plot(self.x_values, self.pwm, label="PWM %", color="#2A9D8F")
            (self.line_pwm_avg,) = ax1.plot(self.x_values, self.pwm_avg, label="Avg PWM %", color="#9A031E", linestyle="--")
            ax1.legend(loc="upper left")
            # Live Avg PWM % readout shown below the avg line at the bottom of the graph.
            self.pwm_text = ax1.text(
                0.02,
                0.07,
                "Avg PWM: --",
                transform=ax1.transAxes,
                ha="left",
                va="bottom",
                fontsize=11,
                fontweight="bold",
                color="#2A9D8F",
                bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="#2A9D8F", alpha=0.85),
            )
        else:
            self.pwm_text = None

        if 'current' in ax_map:
            ax2 = ax_map['current']
            (self.line_i_ocp,) = ax2.plot(
                self.x_values,
                self.current_ocp_ma,
                label="Current OCP",
                color="#F72585",
                zorder=2,
            )
            (self.line_i_avg,) = ax2.plot(
                self.x_values,
                self.current_avg_ma,
                label="Current Avg",
                color="#4361EE",
                zorder=3,
            )
            (self.line_i_motor_limit,) = ax2.plot(
                self.x_values,
                self.dac_motor_current_ma,
                label="Motor Limit (DAC)",
                color="#2A9D8F",
                linestyle=":",
                linewidth=1.2,
            )
            (self.line_i_input_limit,) = ax2.plot(
                self.x_values,
                self.dac_input_current_ma,
                label="Input Limit (DAC)",
                color="#FF9F1C",
                linestyle=":",
                linewidth=1.2,
            )
            ax2.legend(loc="upper left")

        if 'voltage' in ax_map:
            ax3 = ax_map['voltage']
            (self.line_u_mv,) = ax3.plot(self.x_values, self.voltage_mv, label="Voltage", color="#FF9F1C")
            ax3.legend(loc="upper left")

        if 'temperature' in ax_map:
            ax4 = ax_map['temperature']
            (self.line_motor_t,) = ax4.plot(self.x_values, self.motor_temp_c, label="Motor", color="#3A86FF")
            (self.line_mosfet_t,) = ax4.plot(self.x_values, self.mosfet_temp_c, label="MOSFET", color="#FB5607")
            ax4.legend(loc="upper left")

        if 'integral' in ax_map:
            ax5 = ax_map['integral']
            (self.line_error,) = ax5.plot(self.x_values, self.error, label="Error", color="#E76F51")
            (self.line_integral,) = ax5.plot(self.x_values, self.integral, label="Integral", color="#6A4C93")
            (self.line_derivative,) = ax5.plot(self.x_values, self.derivative, label="Derivative", color="#2A9D8F")
            ax5.legend(loc="upper left")

    def _append_value(self, series: List[float], value: float) -> None:
        series.append(value)

    def _handle_sample(self, sample: Sample) -> None:
        self._waiting_for_first_sample = False
        self._first_sample_deadline = 0.0
        self.filled = min(self.filled + 1, self.samples_per_window)

        pwm_percent = (float(sample.pwm_level) * 100.0) / PID_PWM_MAX_LEVEL

        if self.filled < self.samples_per_window:
            self._rpm_sum += float(sample.rpm)
            self._pwm_sum += pwm_percent
        else:
            oldest_rpm = self.rpm[0]
            oldest_pwm = self.pwm[0]
            self._rpm_sum += float(sample.rpm) - (0.0 if math.isnan(oldest_rpm) else oldest_rpm)
            self._pwm_sum += pwm_percent - (0.0 if math.isnan(oldest_pwm) else oldest_pwm)

        self._append_value(self.rpm, float(sample.rpm))
        self._append_value(self.pwm, pwm_percent)
        self._append_value(self.current_avg_ma, float(sample.current_avg_ma))
        self._append_value(self.current_ocp_ma, float(sample.current_ocp_ma))
        self._append_value(self.dac_motor_current_ma, float(sample.dac_motor_current_ma))
        self._append_value(self.dac_input_current_ma, float(sample.dac_input_current_ma))
        self._append_value(self.voltage_mv, float(sample.voltage_mv))
        self._append_value(self.motor_temp_c, float(sample.motor_temp_c))
        self._append_value(self.mosfet_temp_c, float(sample.mosfet_temp_c))
        self._append_value(self.error, float(sample.error))
        self._append_value(self.integral, float(sample.integral))
        self._append_value(self.derivative, float(sample.derivative))

        self._set_fault_indicator(self.ocp_indicator, bool(sample.ocp_fault))
        self._set_fault_indicator(self.driver_fault_indicator, bool(sample.drv_fault))
        self._set_fault_indicator(self.driver_ocp_indicator, bool(sample.snsout_fault))
        self._update_control_button_states()

        current_count = max(1, self.filled)
        rpm_mean = self._rpm_sum / float(current_count)
        pwm_mean = self._pwm_sum / float(current_count)

        missing = self.samples_per_window - self.filled
        self.rpm_avg = deque(
            ([math.nan] * missing) + ([rpm_mean] * self.filled),
            maxlen=self.samples_per_window,
        )
        self.pwm_avg = deque(
            ([math.nan] * missing) + ([pwm_mean] * self.filled),
            maxlen=self.samples_per_window,
        )
        self._plot_dirty = True

    def _apply_x_grid(self) -> None:
        # X-axis major grid: 1s spacing for windows above 1s, 100ms at/below 1s.
        window_seconds = getattr(self, "window_seconds", 1)
        step = 1.0 if window_seconds > 1 else 0.1
        locator = MultipleLocator(step)
        for axis in self.axes:
            axis.xaxis.set_major_locator(locator)

    def _refresh_plot(self) -> None:
        if not self._plot_dirty:
            return

        if hasattr(self, 'line_rpm'):
            self.line_rpm.set_data(self.x_values, self.rpm)
            self.line_rpm_avg.set_data(self.x_values, self.rpm_avg)
            if getattr(self, 'rpm_text', None) is not None:
                current_rpm = self.rpm_avg[-1] if len(self.rpm_avg) else math.nan
                if math.isnan(current_rpm):
                    self.rpm_text.set_text("Avg RPM: --")
                else:
                    self.rpm_text.set_text(f"Avg RPM: {int(current_rpm)}")
        if hasattr(self, 'line_pwm'):
            self.line_pwm.set_data(self.x_values, self.pwm)
            self.line_pwm_avg.set_data(self.x_values, self.pwm_avg)
            if getattr(self, 'pwm_text', None) is not None:
                current_pwm = self.pwm_avg[-1] if len(self.pwm_avg) else math.nan
                if math.isnan(current_pwm):
                    self.pwm_text.set_text("Avg PWM: --")
                else:
                    self.pwm_text.set_text(f"Avg PWM: {current_pwm:.1f}%")
        if hasattr(self, 'line_i_avg'):
            self.line_i_avg.set_data(self.x_values, self.current_avg_ma)
            self.line_i_ocp.set_data(self.x_values, self.current_ocp_ma)
            self.line_i_motor_limit.set_data(self.x_values, self.dac_motor_current_ma)
            self.line_i_input_limit.set_data(self.x_values, self.dac_input_current_ma)
        if hasattr(self, 'line_u_mv'):
            self.line_u_mv.set_data(self.x_values, self.voltage_mv)
        if hasattr(self, 'line_motor_t'):
            self.line_motor_t.set_data(self.x_values, self.motor_temp_c)
            self.line_mosfet_t.set_data(self.x_values, self.mosfet_temp_c)
        if hasattr(self, 'line_error'):
            self.line_error.set_data(self.x_values, self.error)
            self.line_integral.set_data(self.x_values, self.integral)
            self.line_derivative.set_data(self.x_values, self.derivative)

        for axis in self.axes:
            axis.set_xlim(self.x_values[0], self.x_values[-1])
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)

        pwm_graph_visible = getattr(self, 'graph_visibility', {}).get('pwm', True)
        if pwm_graph_visible and len(self.axes) > 1:
            self.axes[1].set_ylim(0.0, 100.0)
        elif hasattr(self, 'line_pwm'):
            for ax in self.axes:
                for line in ax.get_lines():
                    if line == self.line_pwm:
                        ax.set_ylim(0.0, 100.0)
                        break

        self.canvas.draw_idle()
        self._plot_dirty = False

    def _apply_graph_visibility(self) -> None:
        if not hasattr(self, 'canvas'):
            return

        data = self._load_settings_data()
        self.graph_visibility = {
            'rpm': bool(data.get("graph_rpm", True)),
            'pwm': bool(data.get("graph_pwm", True)),
            'current': bool(data.get("graph_current", True)),
            'voltage': bool(data.get("graph_voltage", True)),
            'temperature': bool(data.get("graph_temperature", True)),
            'integral': bool(data.get("graph_integral", True)),
        }

        self.figure.clear()
        num_enabled = sum(1 for v in self.graph_visibility.values() if v)
        if num_enabled == 0:
            num_enabled = 1
            self.graph_visibility['rpm'] = True

        self.axes = self.figure.subplots(num_enabled, 1, sharex=True)
        if num_enabled == 1:
            self.axes = [self.axes]
        else:
            self.axes = list(self.axes)
        hspace = max(0.08, 0.30 * (num_enabled / 6.0))
        self.figure.subplots_adjust(left=0.055, right=0.995, top=0.963, bottom=0.065, hspace=hspace)

        graph_info = [
            ('rpm', "RPM / Avg RPM", "RPM"),
            ('pwm', "PWM (%) / Avg PWM (%)", "%"),
            ('current', "Current Avg/OCP + DAC Limits (mA)", "mA"),
            ('voltage', "Voltage (mV)", "mV"),
            ('temperature', "Temperatures (C)", "C"),
            ('integral', "Integral", "I"),
        ]

        axis_idx = 0
        for graph_type, title, ylabel in graph_info:
            if self.graph_visibility.get(graph_type, True):
                ax = self.axes[axis_idx]
                ax.set_title(title, loc="left", fontsize=10)
                ax.set_ylabel(ylabel)
                ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
                axis_idx += 1

        self._apply_x_grid()
        if len(self.axes) > 0:
            self.axes[-1].set_xlabel("Time (s)")

        self._build_plot_lines()
        self._refresh_plot()
        self.canvas.draw_idle()

    def _write_log_entry(self, text: str) -> None:
        match = LOG_ENTRY_TYPE_PATTERN.match(text)
        if match is not None and not self._log_type_visibility.get(match.group(1), True):
            return
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _set_log_type_visibility(self, log_type: str) -> None:
        enabled = bool(self._log_type_vars[log_type].get())
        self._log_type_visibility[log_type] = enabled
        settings = self._load_settings_data()
        settings[f"log_type_{log_type}"] = enabled
        self._save_settings_data(settings)

    @staticmethod
    def _format_timestamp(timestamp: float) -> str:
        timestamp_date = datetime.fromtimestamp(timestamp)
        return timestamp_date.strftime("%H:%M:%S") + f".{timestamp_date.microsecond // 100:04d}"

    def _format_log_message(self, text: str, log_type: str = "INFO") -> str:
        match = FIRMWARE_LOG_PATTERN.match(text)
        if match is not None:
            firmware_timestamp = int(match.group(1))
            with self._log_timestamp_lock:
                if self._firmware_timestamp_offset is None:
                    self._firmware_timestamp_offset = time.time() - (firmware_timestamp / 1000.0)
                timestamp = (firmware_timestamp / 1000.0) + self._firmware_timestamp_offset
            log_type = match.group(2).upper()
            message = match.group(3)
        else:
            timestamp = time.time()
            log_type = str(log_type).strip().upper() or "INFO"
            message = text

        return f"{self._format_timestamp(timestamp)} {log_type}: {message}"

    def _append_log(self, text: str, log_type: str = "INFO") -> None:
        formatted_text = self._format_log_message(text, log_type)
        try:
            with self.log_file_path.open("a", encoding="utf-8") as handle:
                handle.write(formatted_text + "\n")
            self._log_write_error_reported = False
            self._log_write_error_path = None
        except OSError as exc:
            path = str(self.log_file_path)
            if not self._log_write_error_reported or self._log_write_error_path != path:
                self._log_write_error_reported = True
                self._log_write_error_path = path
                self._write_log_entry(
                    self._format_log_message(f"Failed to write log file {self.log_file_path}: {exc}")
                )

        self._write_log_entry(formatted_text)

    def _request_firmware_reset(self, reason: str, auto_restart: bool = False) -> None:
        if self.config.transport != "swo":
            self._append_log("Firmware reset requires SWO transport", "ERROR")
            return
        if self.reset_in_progress:
            self._append_log("Firmware reset already in progress", "WARNING")
            return

        self.reset_in_progress = True
        self._auto_restart_after_reset = auto_restart
        self._append_log(reason, "WARNING")
        self.status_var.set("Resetting")
        self.backend.stop()
        self._cancel_startup_sync_job()
        self._waiting_for_first_sample = False
        self._first_sample_deadline = 0.0
        self.screenshot_in_progress = False
        self._screenshot_deadline = 0.0
        self.backend.cancel_screenshot()
        self._set_sync_enabled(False)
        self._update_control_button_states()

        def worker() -> None:
            success = self.backend.reset_target()
            self.event_queue.put(("firmware-reset-complete", success))

        threading.Thread(target=worker, daemon=True).start()

    def _stop_monitoring(self, log_message: Optional[str] = "Stopped") -> None:
        self._cancel_startup_sync_job()
        if self.config.transport == "swo":
            # Backend shutdown races with this asynchronous memory write; keep stop quiet.
            self._set_target_enabled(False, quiet=True)
        self.backend.stop()
        self.start_stop_button.configure(text="Start")
        self.status_var.set("Stopped")
        if log_message:
            self._append_log(log_message, "INFO")
        self.sync_in_progress = False
        self._set_sync_enabled(False)
        self.screenshot_in_progress = False
        self._screenshot_deadline = 0.0
        self.backend.cancel_screenshot()
        self._set_fault_indicator(self.ocp_indicator, False)
        self._set_fault_indicator(self.driver_fault_indicator, False)
        self._set_fault_indicator(self.driver_ocp_indicator, False)
        self._waiting_for_first_sample = False
        self._first_sample_deadline = 0.0
        self._update_control_button_states()

    def _reset_backend_after_invalid_swo_data(self, context: str) -> None:
        self._request_firmware_reset(f"Invalid SWO::data detected ({context}); resetting firmware once")

    def _reset_firmware_manual(self) -> None:
        self._request_firmware_reset("Manual firmware reset requested")

    def _start_monitoring(self) -> None:
        self._cancel_startup_sync_job()
        self._set_sync_enabled(False)
        started = self.backend.start(reset_run=False)
        if started:
            self.start_stop_button.configure(text="Stop")
            self.status_var.set("Running")
            self.sync_in_progress = False
            self._waiting_for_first_sample = self.config.transport == "swo"
            self._first_sample_deadline = (
                time.monotonic() + self.STARTUP_PACKET_TIMEOUT_SECONDS
                if self.config.transport == "swo"
                else 0.0
            )
            self._update_control_button_states()
            if self.config.transport == "swo":
                self._set_target_enabled(True)
                self._append_log("Waiting for SWO::data read...", "INFO")
                self._request_read_swo_data("startup")
            self._append_log("Started", "INFO")
        else:
            self.status_var.set("Error")
            self._update_control_button_states()

    def _toggle_start_stop(self) -> None:
        if self.backend.running:
            self._stop_monitoring("Stopped")
            return

        self._start_monitoring()

    def _set_window_preset(self, value: int) -> None:
        self.window_seconds_var.set(str(value))
        self._apply_window()

    def _apply_window(self) -> None:
        raw = self.window_seconds_var.get().strip()
        try:
            seconds = int(raw)
            if seconds <= 0:
                raise ValueError
        except ValueError:
            self._append_log(f"Invalid time window: {raw}", "ERROR")
            return

        data = self._load_settings_data()
        data["graph_time_window"] = seconds
        self._save_settings_data(data)
        self._init_buffers(seconds)
        self._refresh_plot()
        self._append_log(f"Applied time window: {seconds}s", "INFO")

    def _process_events(self) -> None:
        got_sample = False
        while True:
            try:
                kind, payload = self.event_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "log":
                if isinstance(payload, tuple) and len(payload) == 2:
                    text, log_type = payload
                    self._append_log(str(text), str(log_type))
                else:
                    self._append_log(str(payload))
            elif kind == "serial-parameters":
                self._apply_serial_parameters(bytes(payload))
            elif kind == "serial-eeprom":
                data = decode_eeprom_data(bytes(payload))
                if data is None:
                    self.event_queue.put(("eeprom-load-error", f"Invalid serial EEPROM packet size: {len(bytes(payload))}"))
                else:
                    self._populate_eeprom_dialog(data)
            elif kind == "sample":
                self._handle_sample(payload)  # type: ignore[arg-type]
                got_sample = True
            elif kind == "swo-read":
                data, reason = payload  # type: ignore[misc]
                self._set_pid_fields(data)
                self._append_log(
                    f"Loaded SWO::data ({reason}): "
                    f"Kp={data.kp:.6f} Ki={data.ki:.6f} Kd={data.kd:.6f} "
                    f"AWR={anti_windup_raw_to_percent(data.anti_windup):.2f}% RPM={data.rpm} "
                    f"Current={data.input_current_limit}mA Level={data.current_limit_level} changed={int(data.changed)}",
                    "INFO",
                )
                self._set_sync_enabled(True)
                self.sync_in_progress = False
            elif kind == "swo-read-invalid":
                reason = str(payload)
                if reason == "startup":
                    self._reset_backend_after_invalid_swo_data(reason)
            elif kind == "firmware-reset-complete":
                self.reset_in_progress = False
                reset_ok = bool(payload)
                auto_restart = self._auto_restart_after_reset
                self._auto_restart_after_reset = False
                if reset_ok:
                    self.start_stop_button.configure(text="Start")
                    self.status_var.set("Stopped")
                    self.sync_in_progress = False
                    self._set_fault_indicator(self.ocp_indicator, False)
                    self._set_fault_indicator(self.driver_fault_indicator, False)
                    self._set_fault_indicator(self.driver_ocp_indicator, False)
                    self._update_control_button_states()
                    if auto_restart:
                        self._append_log("Firmware reset complete; restarting monitor", "INFO")
                        self._start_monitoring()
                    else:
                        self._append_log('Firmware ready. Press "Start" to connect and run.', "INFO")
                else:
                    self._auto_restart_after_reset = False
                    self.status_var.set("Error")
                    self._append_log("Firmware reset failed; check pyOCD/probe connection", "ERROR")
            elif kind == "sync-done":
                self.sync_in_progress = False
            elif kind == "startup-sync-retry":
                self._startup_sync_job = None
            elif kind == "eeprom-load":
                swo_data, eeprom_data = payload  # type: ignore[misc]
                self._populate_eeprom_dialog(eeprom_data, swo_data)
            elif kind == "eeprom-load-error":
                message = str(payload)
                if self._eeprom_dialog_status_var is not None:
                    self._eeprom_dialog_status_var.set(f"Failed to load EEPROM data: {message}")
                self._append_log(f"EEPROM dialog load failed: {message}", "ERROR")
                self._set_eeprom_dialog_editable(False)
            elif kind == "eeprom-commit-complete":
                committed_eeprom_data = payload if isinstance(payload, EEPROMData) else None
                if self._eeprom_dialog_status_var is not None:
                    self._eeprom_dialog_status_var.set("EEPROM committed")
                self._append_log(
                    "EEPROM data committed"
                    + (" and SWO::data.EEPROM.commit set" if self.config.transport == "swo" else " via serial"),
                    "INFO",
                )
                if committed_eeprom_data is not None:
                    # Keep UI input fields in sync with committed EEPROM values without re-reading SWO memory.
                    self._pid_fields_updating = True
                    try:
                        self.kp_var.set(f"{committed_eeprom_data.kp:.6f}")
                        self.ki_var.set(f"{committed_eeprom_data.ki:.6f}")
                        self.kd_var.set(f"{committed_eeprom_data.kd:.6f}")
                        self.anti_windup_var.set(f"{anti_windup_raw_to_percent(committed_eeprom_data.anti_windup):.2f}")
                        self.rpm_var.set(str(committed_eeprom_data.motor_rpm))
                        self.input_current_limit_var.set(str(committed_eeprom_data.input_current_limit))
                        self.current_limit_level_var.set(current_limit_level_to_name(committed_eeprom_data.current_limit_level))
                        self._pid_fields_dirty = False
                    finally:
                        self._pid_fields_updating = False

                    if self._last_loaded_swo_data is not None:
                        self._last_loaded_swo_data = SWOData(
                            kp=committed_eeprom_data.kp,
                            ki=committed_eeprom_data.ki,
                            kd=committed_eeprom_data.kd,
                            anti_windup=committed_eeprom_data.anti_windup,
                            rpm=committed_eeprom_data.motor_rpm,
                            enabled_state=self._last_loaded_swo_data.enabled_state,
                            changed=self._last_loaded_swo_data.changed,
                            eeprom_address=self._last_loaded_swo_data.eeprom_address,
                            eeprom_commit=self._last_loaded_swo_data.eeprom_commit,
                            send_screenshot=self._last_loaded_swo_data.send_screenshot,
                            input_current_limit=committed_eeprom_data.input_current_limit,
                            current_limit_level=committed_eeprom_data.current_limit_level,
                        )

                    self._append_log(
                        "Updated input fields after EEPROM commit: "
                        f"Kp={committed_eeprom_data.kp:.6f} Ki={committed_eeprom_data.ki:.6f} "
                        f"Kd={committed_eeprom_data.kd:.6f} "
                        f"AWR={anti_windup_raw_to_percent(committed_eeprom_data.anti_windup):.2f}% "
                        f"RPM={committed_eeprom_data.motor_rpm} "
                        f"Current={committed_eeprom_data.input_current_limit}mA "
                        f"Level={committed_eeprom_data.current_limit_level}",
                        "INFO",
                    )
                    self._set_sync_enabled(True)
                self._close_eeprom_dialog()
            elif kind == "eeprom-commit-error":
                message = str(payload)
                if self._eeprom_dialog_status_var is not None:
                    self._eeprom_dialog_status_var.set(f"EEPROM commit failed: {message}")
                self._append_log(f"EEPROM commit failed: {message}", "ERROR")
                self._set_eeprom_dialog_editable(True)
            elif kind == "screenshot-complete":
                filename = Path(str(payload)).name
                self.screenshot_in_progress = False
                self._screenshot_deadline = 0.0
                self._append_log(f"Saved screenshot to {filename}", "INFO")
                self._update_control_button_states()
            elif kind == "screenshot-started":
                filename = Path(str(payload)).name
                # Unsolicited screenshots should follow the same timeout/finalize path as button-triggered captures.
                self.screenshot_in_progress = True
                self._screenshot_deadline = time.monotonic() + SCREENSHOT_TIMEOUT_SECONDS
                self._append_log(f"Screenshot stream started -> {filename}", "INFO")
                self._update_control_button_states()
            elif kind == "screenshot-error":
                message = str(payload)
                self.screenshot_in_progress = False
                self._screenshot_deadline = 0.0
                self.backend.cancel_screenshot()
                self._append_log(message, "ERROR")
                self._update_control_button_states()

        now = time.monotonic()
        graph_refresh_interval = (
            EEPROM_DIALOG_GRAPH_REFRESH_INTERVAL_SECONDS
            if self._eeprom_dialog is not None
            else GRAPH_REFRESH_INTERVAL_SECONDS
        )
        if got_sample and (now - self._last_plot_refresh) >= graph_refresh_interval:
            self._refresh_plot()
            self._last_plot_refresh = now

        pending_capture = self.screenshot_in_progress or self.backend.has_pending_screenshot()

        if pending_capture and self._screenshot_deadline > 0.0 and now >= self._screenshot_deadline:
            self.screenshot_in_progress = False
            self.backend.cancel_screenshot()
            self._append_log("Screenshot request timed out", "ERROR")
            self._update_control_button_states()

        if pending_capture and self.backend.fail_screenshot_if_idle(SCREENSHOT_TIMEOUT_SECONDS):
            self.screenshot_in_progress = False
            self._screenshot_deadline = 0.0
            self._update_control_button_states()

        if self._waiting_for_first_sample and self.backend.running and now >= self._first_sample_deadline:
            self._request_firmware_reset(
                f"No PID packets received within {self.STARTUP_PACKET_TIMEOUT_SECONDS:.0f}s after Start. "
                "Resetting firmware and retrying automatically.",
                auto_restart=True,
            )

        if not self.backend.running and self.start_stop_button.cget("text") == "Stop":
            self.start_stop_button.configure(text="Start")
            self.status_var.set("Stopped")
            self._waiting_for_first_sample = False
            self._first_sample_deadline = 0.0
            self._update_control_button_states()

        self.root.after(40, self._process_events)

    def _on_close(self) -> None:
        self._save_ui_state()
        self.backend.stop()
        self.gdb_mem.close()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> AppConfig:
    defaults = _load_saved_settings_defaults()

    def saved_string(key: str, fallback: str) -> str:
        value = defaults.get(key, fallback)
        return str(value)

    def saved_int(key: str, fallback: int) -> int:
        value = defaults.get(key, fallback)
        try:
            return int(value)
        except Exception:
            return fallback

    parser = argparse.ArgumentParser(description="Motor configuration and parameter tuning tool for SWO or USB/serial")
    parser.add_argument(
        "--transport",
        choices=["serial", "swo"],
        default=saved_string("transport", "serial").strip().lower(),
        help="PID data transport",
    )
    parser.add_argument("--port", dest="serial_port", default=saved_string("serial_port", "COM10"), help="COM port or USB device id")
    parser.add_argument("--baud", dest="serial_baud", type=int, default=saved_int("serial_baud", 115200), help="Serial baud rate")
    parser.add_argument("--uid", default=saved_string("uid", ""), help="Optional pyOCD probe unique ID")
    parser.add_argument("--target", default=saved_string("target", "cortex_m"), help="pyOCD target")
    parser.add_argument("--system-clock", type=int, default=saved_int("system_clock", 72_000_000), help="System clock in Hz")
    parser.add_argument("--swo-clock", type=int, default=saved_int("swo_clock", 2_000_000), help="SWO clock in Hz")
    parser.add_argument("--swd-frequency", type=int, default=saved_int("swd_frequency", 4_000_000), help="SWD frequency in Hz")
    parser.add_argument(
        "--connect-mode",
        choices=["halt", "pre-reset", "under-reset", "attach"],
        default=saved_string("connect_mode", "attach"),
        help="pyOCD connect mode",
    )
    parser.add_argument("--raw-port", type=int, default=saved_int("raw_port", 3443), help="SWV raw TCP port")
    parser.add_argument("--pid-port", type=int, default=1, help="ITM port for PidLoopType")
    parser.add_argument("--gdb-port", type=int, default=saved_int("gdb_port", 3333), help="pyOCD gdbserver port for memory sync")

    args = parser.parse_args()
    return AppConfig(
        transport=args.transport,
        serial_port=args.serial_port,
        serial_baud=args.serial_baud,
        uid=args.uid,
        target=args.target,
        system_clock=args.system_clock,
        swo_clock=args.swo_clock,
        swd_frequency=args.swd_frequency,
        connect_mode=args.connect_mode,
        raw_port=args.raw_port,
        pid_port=args.pid_port,
        gdb_port=args.gdb_port,
    )


def main() -> int:
    config = parse_args()
    app = PIDTuningApp(config)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
