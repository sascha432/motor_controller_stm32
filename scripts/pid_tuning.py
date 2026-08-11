#!/usr/bin/env python3
"""Graphical SWO PID monitor for STM32 + pyOCD.

Author: sascha_lammers@gmx.de

*** Mostly AI generated code be aware of utter garbage that i might have missed ***

The backend connects to pyOCD gdbserver and decodes raw SWV ITM packets.
Port 1 is interpreted as PidController::PidLoopType binary frames.
"""

from __future__ import annotations

import argparse
import math
import queue
import re
import socket
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from collections import deque
from dataclasses import dataclass
from tkinter import ttk
from typing import Callable, Iterable, List, Optional, Tuple

from PIL import Image
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


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


@dataclass
class Sample:
    sequence: int
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
    uid: str
    target: str
    system_clock: int
    swo_clock: int
    swd_frequency: int
    connect_mode: str
    raw_port: int
    pid_port: int
    sample_hz: int
    gdb_port: int


PID_FRAME_MAGIC = b"PID1"
# C++ PidLoopType has 2 bytes padding before float members for 4-byte alignment.
PID_ITEM_STRUCT = "<I9H2xfffI"
PID_ITEM_SIZE = struct.calcsize(PID_ITEM_STRUCT)
SCREENSHOT_FRAME_MAGIC = b"IMG1"
SCREENSHOT_TILE_MAGIC = b"TIL1"
SCREENSHOT_END_MAGIC = b"END1"
SCREENSHOT_PIXEL_FORMAT_RGB565 = 1
# Font-safe screenshot tone mapping.
# Keep gamma moderate to preserve anti-aliased text edges.
SCREENSHOT_GAMMA = 0.20


def _build_screenshot_lut(gamma: float) -> bytes:
    gamma = max(0.1, float(gamma))
    return bytes(
        max(
            0,
            min(
                255,
                int(
                    round(
                        pow(i / 255.0, gamma)
                        * 255.0
                    )
                ),
            ),
        )
        for i in range(256)
    )


SCREENSHOT_GAMMA_LUT = _build_screenshot_lut(SCREENSHOT_GAMMA)

SCREENSHOT_PORT = 2
SCREENSHOT_FRAME_STRUCT = "<4sHHBB"
SCREENSHOT_FRAME_SIZE = struct.calcsize(SCREENSHOT_FRAME_STRUCT)
SCREENSHOT_TILE_STRUCT = "<4sHHHHI"
SCREENSHOT_TILE_SIZE = struct.calcsize(SCREENSHOT_TILE_STRUCT)
# PID_PWM_MAX_LEVEL = 3599.0
PID_PWM_MAX_LEVEL = 100.0
ANTI_WINDUP_FACTOR = 512.0
SWO_DATA_FIXED_RAM_ADDRESS = 0x2000F000
SWO_ENABLE_DISABLED = 0
SWO_ENABLE_SWO = 1
SWO_ENABLE_USB = 2
SWO_DATA_EEPROM_ADDRESS_OFFSET = 20
SWO_DATA_EEPROM_COMMIT_OFFSET = 24
SWO_DATA_SEND_SCREENSHOT_OFFSET = 28
# FOR THE AI: REMEMBER THIS IS 32BIT AND 32BIT PADDED (AND DONT REMOVE THIS COMMENT)
# C++ SWO::DataType layout:
# float Kp, float Ki, float Kd, uint16_t antiWindup, uint16_t rpm,
# uint8_t enabled, bool changed, 2 bytes padding,
# uint32_t EEPROM.address, bool EEPROM.commit, 3 bytes padding,
# bool sendScreenshot, 3 bytes padding.
SWO_DATA_STRUCT = "<fffHHB?2xI?3x?3x"
SWO_DATA_SIZE = struct.calcsize(SWO_DATA_STRUCT)
SWO_DATA_ENABLED_OFFSET = struct.calcsize("<fffHH")
EEPROM_DATA_STRUCT = "<IIIIBBHHHHHBBBBBBBBHxxfffHHH?x"
EEPROM_DATA_SIZE = struct.calcsize(EEPROM_DATA_STRUCT)


@dataclass
class SWOData:
    kp: float
    ki: float
    kd: float
    anti_windup: int
    rpm: int
    changed: bool
    eeprom_address: int
    eeprom_commit: bool
    send_screenshot: bool


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


EEPROM_FIELD_SPECS = (
    ("TFT Brightness", "tft_brightness", "int", 5, 100, None),
    ("LED Brightness", "led_brightness", "int", 0, 100, None),
    ("Input Current (mA)", "input_current_limit", "int", 500, 40000, None),
    ("Motor Current (mA)", "motor_current_limit", "int", 500, 40000, None),
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
    return float(raw_value) / ANTI_WINDUP_FACTOR


def anti_windup_percent_to_raw(percent_value: float) -> int:
    return int(round(percent_value * ANTI_WINDUP_FACTOR))


def decode_pid_item(payload: bytes) -> Optional[Sample]:
    if len(payload) != PID_ITEM_SIZE:
        return None

    sequence, rpm, pwm, voltage, i_ocp, i_avg, motor_ntc, mosfet_ntc, dac_motor, dac_input, error, integral, derivative, faults = struct.unpack(
        PID_ITEM_STRUCT, payload
    )
    if rpm > MAX_RPM:  # RPM might go negative due to small vibrations when the motor is stalled and the sensor limit is 55k RPM
        rpm = 0

    running, drv_fault, ocp_fault, snsout_fault = decode_fault_word(faults)
    return Sample(
        sequence=sequence,
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


def is_plausible_sample(sample: Sample, last_sequence: Optional[int]) -> bool:
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
            return False

    if sample.running not in (0, 1):
        return False
    if sample.drv_fault not in (0, 1):
        return False
    if sample.ocp_fault not in (0, 1):
        return False
    if sample.snsout_fault not in (0, 1):
        return False

    if last_sequence is None:
        return True

    # Only reject exact duplicates. Sequence jumps can legitimately happen
    # after partial frame loss and should not lock out future valid samples.
    return sample.sequence != last_sequence


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
        screenshot_event_callback: Callable[[str, object], None],
        sample_callback: Callable[[Sample], None],
    ) -> None:
        self.config = config
        self.log = log_callback
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
        self._screenshot_width = 0
        self._screenshot_height = 0
        self._screenshot_format = 0
        self._screenshot_last_packet_time = 0.0

    def request_screenshot(self, output_path: Path) -> None:
        with self._screenshot_lock:
            self._screenshot_output_path = output_path
            self._screenshot_buffer.clear()
            self._screenshot_image = None
            self._screenshot_width = 0
            self._screenshot_height = 0
            self._screenshot_format = 0
            self._screenshot_last_packet_time = time.monotonic()

    def cancel_screenshot(self) -> None:
        with self._screenshot_lock:
            self._screenshot_output_path = None
            self._screenshot_buffer.clear()
            self._screenshot_image = None
            self._screenshot_width = 0
            self._screenshot_height = 0
            self._screenshot_format = 0
            self._screenshot_last_packet_time = 0.0

    def has_pending_screenshot(self) -> bool:
        with self._screenshot_lock:
            return self._screenshot_output_path is not None

    def try_finalize_screenshot_if_idle(self, idle_seconds: float) -> bool:
        with self._screenshot_lock:
            if self._screenshot_output_path is None or self._screenshot_image is None:
                return False
            if self._screenshot_last_packet_time <= 0.0:
                return False
            if (time.monotonic() - self._screenshot_last_packet_time) < idle_seconds:
                return False

        self._finalize_screenshot_stream()
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
        self.log("Launching: " + " ".join(cmd))

        try:
            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=0,
            )
        except FileNotFoundError:
            self.log("pyOCD not found. Install with: pip install pyocd")
            return False
        except Exception as exc:  # pragma: no cover - defensive path
            self.log(f"Failed to launch pyOCD: {exc}")
            return False

        self.stop_event.clear()
        self.stdout_thread = threading.Thread(target=self._read_stdout, daemon=True)
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

        self.log("Resetting firmware: " + " ".join(cmd))
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
            self.log("pyOCD not found. Install with: pip install pyocd")
            return False
        except subprocess.TimeoutExpired as exc:
            self.log("Firmware reset timed out after 8s")
            if exc.stdout:
                self.log(str(exc.stdout).rstrip())
            return False
        except Exception as exc:  # pragma: no cover - defensive path
            self.log(f"Failed to reset firmware: {exc}")
            return False

        if result.returncode != 0:
            self.log(f"Firmware reset failed with exit code {result.returncode}")
            if result.stdout:
                self.log(result.stdout.rstrip())
            return False

        if result.stdout:
            self.log(result.stdout.rstrip())
        return True

    def _read_stdout(self) -> None:
        if not self.proc or self.proc.stdout is None:
            return

        buffer = bytearray()
        while not self.stop_event.is_set():
            chunk = self.proc.stdout.read(1)
            if not chunk:
                if buffer:
                    self.log(buffer.decode("utf-8", errors="replace"))
                break

            buffer.extend(chunk)
            if chunk == b"\n":
                self.log(buffer.decode("utf-8", errors="replace").rstrip("\r\n"))
                buffer.clear()

        self.running = False

    def _read_raw_swv(self) -> None:
        last_connect_log = 0.0

        while not self.stop_event.is_set():
            try:
                with socket.create_connection(("127.0.0.1", self.config.raw_port), timeout=1.0) as sock:
                    sock.settimeout(1.0)
                    self.log(f"Connected to SWV raw stream on tcp://127.0.0.1:{self.config.raw_port}")

                    accumulated = bytearray()
                    pid_bytes = bytearray()
                    last_sequence: Optional[int] = None

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
                                # Port 0 text is already mirrored by pyOCD stdout because
                                # semihost_console_type=console is enabled.
                                continue

                            if port == SCREENSHOT_PORT:
                                self._consume_screenshot_payload(payload)
                                continue

                            if port != self.config.pid_port:
                                continue

                            pid_bytes.extend(payload)

                            while True:
                                magic_pos = pid_bytes.find(PID_FRAME_MAGIC)
                                if magic_pos < 0:
                                    if len(pid_bytes) > len(PID_FRAME_MAGIC) - 1:
                                        del pid_bytes[: -(len(PID_FRAME_MAGIC) - 1)]
                                    break

                                if magic_pos > 0:
                                    del pid_bytes[:magic_pos]

                                frame_size = len(PID_FRAME_MAGIC) + PID_ITEM_SIZE
                                if len(pid_bytes) < frame_size:
                                    break

                                raw_item = bytes(pid_bytes[len(PID_FRAME_MAGIC):frame_size])
                                sample = decode_pid_item(raw_item)
                                if sample and is_plausible_sample(sample, last_sequence):
                                    del pid_bytes[:frame_size]
                                    last_sequence = sample.sequence
                                    self.sample_callback(sample)
                                    continue

                                # Invalid candidate frame: shift by one byte and search again.
                                # This recovers quickly from partial writes/split frames.
                                del pid_bytes[0]

            except OSError:
                accumulated = bytearray()
                pid_bytes = bytearray()
                now = time.monotonic()
                if now - last_connect_log > 2.0:
                    self.log(f"Waiting for SWV raw server on tcp://127.0.0.1:{self.config.raw_port}...")
                    last_connect_log = now
                time.sleep(0.2)

    @staticmethod
    def _rgb565_to_rgb_bytes(payload: bytes) -> bytes:
        if len(payload) % 2 != 0:
            raise ValueError("RGB565 payload length must be even")

        rgb = bytearray((len(payload) // 2) * 3)
        lut = SCREENSHOT_GAMMA_LUT
        dst = 0
        for src in range(0, len(payload), 2):
            value = payload[src] | (payload[src + 1] << 8)
            red = ((value >> 11) & 0x1F) * 255 // 31
            green = ((value >> 5) & 0x3F) * 255 // 63
            blue = (value & 0x1F) * 255 // 31
            rgb[dst] = lut[red]
            rgb[dst + 1] = lut[green]
            rgb[dst + 2] = lut[blue]
            dst += 3
        return bytes(rgb)

    def _reset_screenshot_stream(self) -> None:
        with self._screenshot_lock:
            self._screenshot_buffer.clear()
            self._screenshot_image = None
            self._screenshot_width = 0
            self._screenshot_height = 0
            self._screenshot_format = 0

    def _finalize_screenshot_stream(self) -> None:
        with self._screenshot_lock:
            image = self._screenshot_image
            output_path = self._screenshot_output_path
            self._screenshot_output_path = None
            self._screenshot_buffer.clear()
            self._screenshot_image = None
            self._screenshot_width = 0
            self._screenshot_height = 0
            self._screenshot_format = 0

        if image is None or output_path is None:
            return

        try:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            image.save(output_path, format="PNG")
        except Exception as exc:  # pragma: no cover - defensive path
            self._screenshot_event_callback("screenshot-error", f"Failed to save screenshot: {exc}")
            return

        self._screenshot_event_callback("screenshot-complete", str(output_path))

    def _consume_screenshot_payload(self, payload: bytes) -> None:
        with self._screenshot_lock:
            if self._screenshot_output_path is None:
                return
            self._screenshot_last_packet_time = time.monotonic()
            self._screenshot_buffer.extend(payload)

            while True:
                if self._screenshot_image is None:
                    if len(self._screenshot_buffer) < SCREENSHOT_FRAME_SIZE:
                        return

                    header_index = self._screenshot_buffer.find(SCREENSHOT_FRAME_MAGIC)
                    if header_index < 0:
                        keep = len(SCREENSHOT_FRAME_MAGIC) - 1
                        if len(self._screenshot_buffer) > keep:
                            del self._screenshot_buffer[:-keep]
                        return

                    if header_index > 0:
                        del self._screenshot_buffer[:header_index]
                        if len(self._screenshot_buffer) < SCREENSHOT_FRAME_SIZE:
                            return

                    magic, width, height, pixel_format, reserved = struct.unpack(
                        SCREENSHOT_FRAME_STRUCT,
                        self._screenshot_buffer[:SCREENSHOT_FRAME_SIZE],
                    )
                    if magic != SCREENSHOT_FRAME_MAGIC:
                        del self._screenshot_buffer[0]
                        continue
                    if pixel_format != SCREENSHOT_PIXEL_FORMAT_RGB565:
                        self._screenshot_event_callback("screenshot-error", f"Unsupported screenshot format: {pixel_format}")
                        self._reset_screenshot_stream()
                        return

                    self._screenshot_width = width
                    self._screenshot_height = height
                    self._screenshot_format = pixel_format
                    self._screenshot_image = Image.new("RGB", (width, height))
                    del self._screenshot_buffer[:SCREENSHOT_FRAME_SIZE]
                    continue

                if len(self._screenshot_buffer) < 4:
                    return

                if self._screenshot_buffer.startswith(SCREENSHOT_END_MAGIC):
                    del self._screenshot_buffer[:len(SCREENSHOT_END_MAGIC)]
                    self._finalize_screenshot_stream()
                    return

                if len(self._screenshot_buffer) < SCREENSHOT_TILE_SIZE:
                    return

                if not self._screenshot_buffer.startswith(SCREENSHOT_TILE_MAGIC):
                    del self._screenshot_buffer[0]
                    continue

                magic, x, y, width, height, byte_count = struct.unpack(
                    SCREENSHOT_TILE_STRUCT,
                    self._screenshot_buffer[:SCREENSHOT_TILE_SIZE],
                )
                if magic != SCREENSHOT_TILE_MAGIC:
                    del self._screenshot_buffer[0]
                    continue

                if len(self._screenshot_buffer) < SCREENSHOT_TILE_SIZE + byte_count:
                    return

                tile_bytes = bytes(
                    self._screenshot_buffer[SCREENSHOT_TILE_SIZE : SCREENSHOT_TILE_SIZE + byte_count]
                )
                del self._screenshot_buffer[:SCREENSHOT_TILE_SIZE + byte_count]

                if self._screenshot_image is None:
                    continue

                if width == 0 or height == 0:
                    continue

                if x + width > self._screenshot_width or y + height > self._screenshot_height:
                    self._screenshot_event_callback("screenshot-error", "Screenshot tile out of bounds")
                    self._reset_screenshot_stream()
                    return

                rgb_bytes = self._rgb565_to_rgb_bytes(tile_bytes)
                tile_image = Image.frombytes("RGB", (width, height), rgb_bytes)
                self._screenshot_image.paste(tile_image, (x, y))


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
        # Detach so target resumes, then close this short-lived RSP session.
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
    STARTUP_SYNC_DELAY_MS = 200
    STARTUP_PACKET_TIMEOUT_SECONDS = 5.0

    @staticmethod
    def _make_series(length: int) -> deque[float]:
        return deque([math.nan] * length, maxlen=length)

    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.event_queue: queue.Queue[Tuple[str, object]] = queue.Queue()
        self.backend = SWOBackend(
            config,
            log_callback=lambda msg: self.event_queue.put(("log", msg)),
            screenshot_event_callback=lambda kind, payload: self.event_queue.put((kind, payload)),
            sample_callback=lambda sample: self.event_queue.put(("sample", sample)),
        )

        self.root = tk.Tk()
        self.root.title("PID Tuning SWO Monitor")
        self.root.geometry("1920x1200")
        self.root.minsize(1000, 640)

        self.window_seconds_var = tk.StringVar(value="10")
        self.status_var = tk.StringVar(value="Stopped")
        self.kp_var = tk.StringVar(value="0.0")
        self.ki_var = tk.StringVar(value="0.0")
        self.kd_var = tk.StringVar(value="0.0")
        self.anti_windup_var = tk.StringVar(value="0.00")
        self.rpm_var = tk.StringVar(value="0")
        self._pid_fields_updating = False
        self._pid_fields_dirty = False
        self._last_loaded_swo_data: Optional[SWOData] = None
        self._target_running = False
        self._eeprom_dialog: Optional[tk.Toplevel] = None
        self._eeprom_dialog_status_var: Optional[tk.StringVar] = None
        self._eeprom_dialog_vars: dict[str, tk.StringVar] = {}
        self._eeprom_dialog_widgets: list[tuple[tk.Widget, str]] = []
        self._eeprom_dialog_summary_vars: dict[str, tk.StringVar] = {}
        self._eeprom_dialog_commit_button: Optional[ttk.Button] = None
        self._eeprom_dialog_source: Optional[EEPROMData] = None
        self._eeprom_dialog_address: Optional[int] = None
        self.screenshot_in_progress = False
        self._screenshot_deadline = 0.0
        self._install_pid_field_traces()

        self.data_address: Optional[int] = SWO_DATA_FIXED_RAM_ADDRESS
        self.pending_initial_sync = False
        self.sync_in_progress = False
        self.start_reset_retried = False
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
        self._init_buffers(window_seconds=10)
        self._build_plot_lines()
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

        pid_group = ttk.LabelFrame(outer, text="PID Parameters (SWO::DataType)")
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

        self.sync_button = ttk.Button(pid_group, text="Sync", command=self._sync_to_target, state=tk.DISABLED)
        self.sync_button.grid(
            row=5, column=0, columnspan=2, padx=6, pady=(8, 6), sticky="ew"
        )

        self.eeprom_button = ttk.Button(pid_group, text="EEPROM...", command=self._open_eeprom_dialog, state=tk.DISABLED)
        self.eeprom_button.grid(row=6, column=0, columnspan=2, padx=6, pady=(0, 6), sticky="ew")

        self.screenshot_button = ttk.Button(pid_group, text="Screenshot", command=self._request_screenshot, state=tk.DISABLED)
        self.screenshot_button.grid(row=7, column=0, columnspan=2, padx=6, pady=(0, 6), sticky="ew")

        self.reset_firmware_button = ttk.Button(
            pid_group,
            text="Reset Firmware",
            command=self._reset_firmware_manual,
            state=tk.DISABLED,
        )
        self.reset_firmware_button.grid(row=8, column=0, columnspan=2, padx=6, pady=(0, 6), sticky="ew")

        ttk.Label(
            outer,
            text="Panels are resizable; drag separators to adjust Graph / Logs / Faults / Controls.",
            justify=tk.LEFT,
        ).grid(row=3, column=0, columnspan=2, sticky="w", pady=(6, 0))

    def _pack_swo_data(
        self,
        data: SWOData,
        eeprom_address: int,
        eeprom_commit: bool,
        send_screenshot: bool = False,
        enabled_state: int = SWO_ENABLE_DISABLED,
        changed: bool = True,
    ) -> bytes:
        # C++ layout: float Kp, float Ki, float Kd, uint16_t antiWindup,
        # uint16_t rpm, enum class EnableState : uint8_t enabled, bool changed,
        # padding, then struct { uint32_t address; bool commit; bool sendScreenshot; } EEPROM.
        return struct.pack(
            SWO_DATA_STRUCT,
            data.kp,
            data.ki,
            data.kd,
            data.anti_windup,
            data.rpm,
            enabled_state & 0xFF,
            changed,
            eeprom_address,
            eeprom_commit,
            send_screenshot,
        )

    def _unpack_swo_data(self, payload: bytes) -> SWOData:
        kp, ki, kd, anti_windup_raw, rpm, _enabled_state, changed, eeprom_address, eeprom_commit, send_screenshot = struct.unpack(
            SWO_DATA_STRUCT,
            payload[:SWO_DATA_SIZE],
        )
        return SWOData(
            kp=kp,
            ki=ki,
            kd=kd,
            anti_windup=anti_windup_raw,
            rpm=rpm,
            changed=changed,
            eeprom_address=eeprom_address,
            eeprom_commit=eeprom_commit,
            send_screenshot=send_screenshot,
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
        )

    def _unpack_eeprom_data(self, payload: bytes) -> EEPROMData:
        values = struct.unpack(EEPROM_DATA_STRUCT, payload[:EEPROM_DATA_SIZE])
        return EEPROMData(*values)

    @staticmethod
    def _is_invalid_swo_default_signature(data: SWOData) -> bool:
        return (
            not data.changed
            and abs(data.kp - 1.0) < 1e-6
            and abs(data.ki - 1.0) < 1e-6
            and abs(data.kd - 0.0) < 1e-6
            and data.rpm == 0
        )

    def _validate_swo_payload(self, payload: bytes) -> Tuple[SWOData, int]:
        if len(payload) < SWO_DATA_SIZE:
            raise RuntimeError(f"SWO::data read returned too few bytes ({len(payload)} < {SWO_DATA_SIZE})")

        data = self._unpack_swo_data(payload)
        enabled_state = payload[SWO_DATA_ENABLED_OFFSET]

        if enabled_state not in (SWO_ENABLE_DISABLED, SWO_ENABLE_SWO, SWO_ENABLE_USB):
            raise RuntimeError(f"Invalid SWO::data enabled state: {enabled_state}")

        for name, value in (("Kp", data.kp), ("Ki", data.ki), ("Kd", data.kd)):
            if not math.isfinite(value):
                raise RuntimeError(f"Invalid SWO::data {name}: not finite")

        if not (0 <= data.rpm <= 65535):
            raise RuntimeError(f"Invalid SWO::data RPM: {data.rpm}")

        if not (0 <= data.anti_windup <= anti_windup_percent_to_raw(100.0)):
            raise RuntimeError(
                f"Invalid SWO::data anti-windup raw value: {data.anti_windup}"
            )

        if data.eeprom_address == 0:
            raise RuntimeError("Invalid SWO::data EEPROM address: 0")

        if self._is_invalid_swo_default_signature(data):
            raise RuntimeError("Invalid SWO::data signature detected")

        return data, enabled_state

    def _install_pid_field_traces(self) -> None:
        for var in (self.kp_var, self.ki_var, self.kd_var, self.anti_windup_var, self.rpm_var):
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
            self._last_loaded_swo_data = data
            self._pid_fields_dirty = False
        finally:
            self._pid_fields_updating = False

    def _set_sync_enabled(self, enabled: bool) -> None:
        self.sync_button.configure(state=(tk.NORMAL if enabled else tk.DISABLED))

    def _set_eeprom_button_enabled(self, enabled: bool) -> None:
        self.eeprom_button.configure(state=(tk.NORMAL if enabled else tk.DISABLED))

    def _set_screenshot_button_enabled(self, enabled: bool) -> None:
        self.screenshot_button.configure(state=(tk.NORMAL if enabled else tk.DISABLED))

    def _set_reset_firmware_button_enabled(self, enabled: bool) -> None:
        self.reset_firmware_button.configure(state=(tk.NORMAL if enabled else tk.DISABLED))

    def _update_control_button_states(self) -> None:
        eeprom_enabled = self.backend.running and not self.reset_in_progress and self._eeprom_dialog is None
        screenshot_enabled = self.backend.running and not self.reset_in_progress and not self.screenshot_in_progress
        reset_enabled = not self.reset_in_progress and self._eeprom_dialog is None
        self._set_eeprom_button_enabled(eeprom_enabled)
        self._set_screenshot_button_enabled(screenshot_enabled)
        self._set_reset_firmware_button_enabled(reset_enabled)

    def _next_screenshot_path(self) -> Path:
        script_dir = Path(__file__).resolve().parent
        index = 0
        while True:
            candidate = script_dir / f"ss{index:06d}.png"
            if not candidate.exists():
                return candidate
            index += 1

    def _request_screenshot(self) -> None:
        if self.data_address is None:
            self._append_log("Cannot take screenshot: no dataAddress received yet")
            return
        if not self.backend.running or self.reset_in_progress:
            self._append_log("Cannot take screenshot: monitor is not running")
            return
        if self.screenshot_in_progress:
            self._append_log("Screenshot already in progress")
            return

        output_path = self._next_screenshot_path()
        self.screenshot_in_progress = True
        self._screenshot_deadline = time.monotonic() + 10.0
        self._update_control_button_states()

        def worker() -> None:
            try:
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
        if self.data_address is None:
            return

        def worker() -> None:
            try:
                payload = self.gdb_mem.read_memory(self.data_address, SWO_DATA_SIZE)
                current_data, _enabled_state = self._validate_swo_payload(payload)
                data = bytearray(payload)
                data[SWO_DATA_ENABLED_OFFSET] = SWO_ENABLE_SWO if enabled else SWO_ENABLE_DISABLED
                data[SWO_DATA_EEPROM_ADDRESS_OFFSET:SWO_DATA_EEPROM_ADDRESS_OFFSET + 4] = struct.pack(
                    "<I",
                    current_data.eeprom_address,
                )
                data[SWO_DATA_EEPROM_COMMIT_OFFSET] = 1 if current_data.eeprom_commit else 0
                self.gdb_mem.write_memory(self.data_address, bytes(data))
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
                self._eeprom_dialog.destroy()
            except Exception:
                pass

        self._eeprom_dialog = None
        self._eeprom_dialog_status_var = None
        self._eeprom_dialog_vars = {}
        self._eeprom_dialog_widgets = []
        self._eeprom_dialog_summary_vars = {}
        self._eeprom_dialog_commit_button = None
        self._eeprom_dialog_source = None
        self._eeprom_dialog_address = None
        self._update_control_button_states()

    def _populate_eeprom_dialog(self, swo_data: SWOData, data: EEPROMData) -> None:
        if self._eeprom_dialog is None:
            return

        self._eeprom_dialog_source = data
        self._eeprom_dialog_address = swo_data.eeprom_address

        summary_vars = self._eeprom_dialog_summary_vars
        summary_vars["magic"].set(f"0x{data.magic:08X}")
        summary_vars["version"].set(str(data.version))
        summary_vars["sequence"].set(str(data.sequence))
        summary_vars["crc"].set(f"0x{data.crc:08X}")
        summary_vars["address"].set(f"0x{self._eeprom_dialog_address:08X}")
        summary_vars["commit"].set("set" if swo_data.eeprom_commit else "clear")

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
            self._append_log("EEPROM editor is only available while monitoring is started")
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("EEPROM Editor")
        dialog.transient(self.root)
        dialog.grab_set()
        dialog.minsize(980, 560)
        dialog.geometry("1080x640")
        dialog.protocol("WM_DELETE_WINDOW", self._close_eeprom_dialog)
        self._eeprom_dialog = dialog

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

        button_row = ttk.Frame(button_frame)
        button_row.grid(row=0, column=1, sticky="e")
        self._eeprom_dialog_commit_button = ttk.Button(button_row, text="Commit", command=self._commit_eeprom_dialog, state=tk.DISABLED)
        self._eeprom_dialog_commit_button.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(button_row, text="Cancel", command=self._close_eeprom_dialog).pack(side=tk.LEFT)

        self._set_eeprom_dialog_editable(False)

        def worker() -> None:
            try:
                swo_payload = self.gdb_mem.read_memory(self.data_address, SWO_DATA_SIZE)
                swo_data, _enabled_state = self._validate_swo_payload(swo_payload)
                eeprom_payload = self.gdb_mem.read_memory(swo_data.eeprom_address, EEPROM_DATA_SIZE)
                eeprom_data = self._unpack_eeprom_data(eeprom_payload)
                self.event_queue.put(("eeprom-load", (swo_data, eeprom_data)))
            except Exception as exc:
                self.event_queue.put(("eeprom-load-error", str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def _commit_eeprom_dialog(self) -> None:
        if self._eeprom_dialog_source is None or self._eeprom_dialog_address is None:
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

    def _run_scheduled_startup_sync(self) -> None:
        self._startup_sync_job = None
        if not self.backend.running:
            return
        if not self.pending_initial_sync:
            return
        if self.data_address is None:
            return
        self._request_read_swo_data("startup")

    def _request_read_swo_data(self, reason: str) -> None:
        if self.data_address is None:
            self._append_log("Cannot read SWO::data yet: dataAddress not available")
            return
        if self.sync_in_progress:
            return

        self.sync_in_progress = True

        def worker() -> None:
            try:
                payload = self.gdb_mem.read_memory(self.data_address, SWO_DATA_SIZE)
                data, _enabled_state = self._validate_swo_payload(payload)
                self.event_queue.put(("swo-read", (data, reason)))
            except Exception as exc:
                self.event_queue.put(("log", f"Read SWO::data failed: {exc}"))
                self.event_queue.put(("swo-read-invalid", reason))
                if reason == "startup":
                    self.event_queue.put(("startup-sync-retry", None))
                self.event_queue.put(("sync-done", None))

        threading.Thread(target=worker, daemon=True).start()

    def _sync_to_target(self) -> None:
        if self.data_address is None:
            self._append_log("Cannot sync: no dataAddress received yet")
            return
        if self.sync_in_progress:
            self._append_log("Sync already in progress")
            return

        # Read-only sync unless user edited fields. This prevents accidental
        # writes of defaults and keeps Sync useful for refresh.
        if not self._pid_fields_dirty:
            self._request_read_swo_data("manual")
            return

        try:
            kp = float(self.kp_var.get().strip())
            ki = float(self.ki_var.get().strip())
            kd = float(self.kd_var.get().strip())
            anti_windup = float(self.anti_windup_var.get().strip())
            if anti_windup < 0.0 or anti_windup > 100.0:
                raise ValueError("Anti-windup out of range (0.00..100.00)")
            rpm = int(self.rpm_var.get().strip())
            if rpm < 0 or rpm > 65535:
                raise ValueError("RPM out of range (0..65535)")
        except Exception as exc:
            self._append_log(f"Invalid PID input: {exc}")
            return

        self.sync_in_progress = True
        data = SWOData(
            kp=kp,
            ki=ki,
            kd=kd,
            anti_windup=anti_windup_percent_to_raw(anti_windup),
            rpm=rpm,
            changed=True,
            eeprom_address=0,
            eeprom_commit=False,
            send_screenshot=False,
        )

        def worker() -> None:
            try:
                current_payload = self.gdb_mem.read_memory(self.data_address, SWO_DATA_SIZE)
                current_data, enabled_state = self._validate_swo_payload(current_payload)
                self.gdb_mem.write_memory(
                    self.data_address,
                    self._pack_swo_data(
                        data,
                        enabled_state=enabled_state,
                        changed=True,
                        eeprom_address=current_data.eeprom_address,
                        eeprom_commit=current_data.eeprom_commit,
                        send_screenshot=current_data.send_screenshot,
                    ),
                )
                self.event_queue.put(("log", "Synced PID params to SWO::data (changed=true)"))
                # Read back once after write for confirmation.
                payload = self.gdb_mem.read_memory(self.data_address, SWO_DATA_SIZE)
                data_verify, _enabled_state_verify = self._validate_swo_payload(payload)
                self.event_queue.put(("swo-read", (data_verify, "after write")))
            except Exception as exc:
                self.event_queue.put(("log", f"Write SWO::data failed: {exc}"))
                self.event_queue.put(("sync-done", None))

        threading.Thread(target=worker, daemon=True).start()

    def _init_buffers(self, window_seconds: int) -> None:
        self.window_seconds = max(1, int(window_seconds))
        self.samples_per_window = max(16, self.window_seconds * self.config.sample_hz)
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

    def _build_plot_lines(self) -> None:
        ax0, ax1, ax2, ax3, ax4, ax5 = self.axes

        (self.line_rpm,) = ax0.plot(self.x_values, self.rpm, label="RPM", color="#0077B6")
        (self.line_rpm_avg,) = ax0.plot(self.x_values, self.rpm_avg, label="Avg RPM", color="#E85D04", linestyle="--")
        ax0.legend(loc="upper left")

        (self.line_pwm,) = ax1.plot(self.x_values, self.pwm, label="PWM %", color="#2A9D8F")
        (self.line_pwm_avg,) = ax1.plot(self.x_values, self.pwm_avg, label="Avg PWM %", color="#9A031E", linestyle="--")
        ax1.legend(loc="upper left")

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

        (self.line_u_mv,) = ax3.plot(self.x_values, self.voltage_mv, label="Voltage", color="#FF9F1C")
        ax3.legend(loc="upper left")

        (self.line_motor_t,) = ax4.plot(self.x_values, self.motor_temp_c, label="Motor", color="#3A86FF")
        (self.line_mosfet_t,) = ax4.plot(self.x_values, self.mosfet_temp_c, label="MOSFET", color="#FB5607")
        ax4.legend(loc="upper left")

        (self.line_error,) = ax5.plot(self.x_values, self.error, label="Error", color="#E76F51")
        (self.line_integral,) = ax5.plot(self.x_values, self.integral, label="Integral", color="#6A4C93")
        (self.line_derivative,) = ax5.plot(self.x_values, self.derivative, label="Derivative", color="#2A9D8F")
        ax5.legend(loc="upper left")

    def _append_value(self, series: List[float], value: float) -> None:
        series.append(value)

    def _handle_sample(self, sample: Sample) -> None:
        self._waiting_for_first_sample = False
        self._first_sample_deadline = 0.0
        self._target_running = bool(sample.running)
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

    def _refresh_plot(self) -> None:
        if not self._plot_dirty:
            return

        self.line_rpm.set_data(self.x_values, self.rpm)
        self.line_rpm_avg.set_data(self.x_values, self.rpm_avg)
        self.line_pwm.set_data(self.x_values, self.pwm)
        self.line_pwm_avg.set_data(self.x_values, self.pwm_avg)
        self.line_i_avg.set_data(self.x_values, self.current_avg_ma)

        self.line_i_ocp.set_data(self.x_values, self.current_ocp_ma)
        self.line_i_motor_limit.set_data(self.x_values, self.dac_motor_current_ma)
        self.line_i_input_limit.set_data(self.x_values, self.dac_input_current_ma)
        self.line_u_mv.set_data(self.x_values, self.voltage_mv)
        self.line_motor_t.set_data(self.x_values, self.motor_temp_c)
        self.line_mosfet_t.set_data(self.x_values, self.mosfet_temp_c)
        self.line_error.set_data(self.x_values, self.error)
        self.line_integral.set_data(self.x_values, self.integral)
        self.line_derivative.set_data(self.x_values, self.derivative)

        for axis in self.axes:
            axis.set_xlim(self.x_values[0], self.x_values[-1])
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)

        # Keep PWM plot in percentage range.
        self.axes[1].set_ylim(0.0, 100.0)

        self.canvas.draw_idle()
        self._plot_dirty = False

    def _append_log(self, text: str) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _request_firmware_reset(self, reason: str, auto_restart: bool = False) -> None:
        if self.reset_in_progress:
            self._append_log("Firmware reset already in progress")
            return

        self.reset_in_progress = True
        self._auto_restart_after_reset = auto_restart
        self._append_log(reason)
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
        # Backend shutdown races with this asynchronous memory write; keep stop quiet.
        self._set_target_enabled(False, quiet=True)
        self.backend.stop()
        self.start_stop_button.configure(text="Start")
        self.status_var.set("Stopped")
        if log_message:
            self._append_log(log_message)
        self.pending_initial_sync = False
        self.sync_in_progress = False
        self._set_sync_enabled(False)
        self.start_reset_retried = False
        self.screenshot_in_progress = False
        self._screenshot_deadline = 0.0
        self.backend.cancel_screenshot()
        self._set_fault_indicator(self.ocp_indicator, False)
        self._set_fault_indicator(self.driver_fault_indicator, False)
        self._set_fault_indicator(self.driver_ocp_indicator, False)
        self._target_running = False
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
        self.start_reset_retried = False
        started = self.backend.start(reset_run=False)
        if started:
            self._set_target_enabled(True)
            self.start_stop_button.configure(text="Stop")
            self.status_var.set("Running")
            self.pending_initial_sync = True
            self.sync_in_progress = False
            self.start_reset_retried = False
            self._waiting_for_first_sample = True
            self._first_sample_deadline = time.monotonic() + self.STARTUP_PACKET_TIMEOUT_SECONDS
            self._update_control_button_states()
            self._append_log("Waiting for SWO::data read...")
            self._append_log("Started")
            self._request_read_swo_data("startup")
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
            self._append_log(f"Invalid time window: {raw}")
            return

        self._init_buffers(seconds)
        self._refresh_plot()
        self._append_log(f"Applied time window: {seconds}s")

    def _process_events(self) -> None:
        got_sample = False
        while True:
            try:
                kind, payload = self.event_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "log":
                text = str(payload)
                self._append_log(text)
            elif kind == "sample":
                self._handle_sample(payload)  # type: ignore[arg-type]
                got_sample = True
            elif kind == "swo-read":
                data, reason = payload  # type: ignore[misc]
                self._set_pid_fields(data)
                self._append_log(
                    f"Loaded SWO::data ({reason}): "
                    f"Kp={data.kp:.6f} Ki={data.ki:.6f} Kd={data.kd:.6f} "
                    f"AWR={anti_windup_raw_to_percent(data.anti_windup):.2f}% RPM={data.rpm} changed={int(data.changed)}"
                )
                self._set_sync_enabled(True)
                if reason == "startup":
                    self.pending_initial_sync = False
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
                    self.start_reset_retried = False
                    self.pending_initial_sync = False
                    self.sync_in_progress = False
                    self._set_fault_indicator(self.ocp_indicator, False)
                    self._set_fault_indicator(self.driver_fault_indicator, False)
                    self._set_fault_indicator(self.driver_ocp_indicator, False)
                    self._target_running = False
                    self._update_control_button_states()
                    if auto_restart:
                        self._append_log("Firmware reset complete; restarting monitor")
                        self._start_monitoring()
                    else:
                        self._append_log('Firmware ready. Press "Start" to connect and run.')
                else:
                    self._auto_restart_after_reset = False
                    self.status_var.set("Error")
                    self._append_log("Firmware reset failed; check pyOCD/probe connection")
            elif kind == "sync-done":
                self.sync_in_progress = False
            elif kind == "startup-sync-retry":
                self._startup_sync_job = None
                self.pending_initial_sync = True
            elif kind == "eeprom-load":
                swo_data, eeprom_data = payload  # type: ignore[misc]
                self._populate_eeprom_dialog(swo_data, eeprom_data)
            elif kind == "eeprom-load-error":
                message = str(payload)
                if self._eeprom_dialog_status_var is not None:
                    self._eeprom_dialog_status_var.set(f"Failed to load EEPROM data: {message}")
                self._append_log(f"EEPROM dialog load failed: {message}")
                self._set_eeprom_dialog_editable(False)
            elif kind == "eeprom-commit-complete":
                committed_eeprom_data = payload if isinstance(payload, EEPROMData) else None
                if self._eeprom_dialog_status_var is not None:
                    self._eeprom_dialog_status_var.set("EEPROM committed")
                self._append_log("EEPROM data committed and SWO::data.EEPROM.commit set")
                if committed_eeprom_data is not None:
                    # Keep UI input fields in sync with committed EEPROM values without re-reading SWO memory.
                    self._pid_fields_updating = True
                    try:
                        self.kp_var.set(f"{committed_eeprom_data.kp:.6f}")
                        self.ki_var.set(f"{committed_eeprom_data.ki:.6f}")
                        self.kd_var.set(f"{committed_eeprom_data.kd:.6f}")
                        self.anti_windup_var.set(f"{anti_windup_raw_to_percent(committed_eeprom_data.anti_windup):.2f}")
                        self.rpm_var.set(str(committed_eeprom_data.motor_rpm))
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
                            changed=self._last_loaded_swo_data.changed,
                            eeprom_address=self._last_loaded_swo_data.eeprom_address,
                            eeprom_commit=self._last_loaded_swo_data.eeprom_commit,
                            send_screenshot=self._last_loaded_swo_data.send_screenshot,
                        )

                    self._append_log(
                        "Updated input fields after EEPROM commit: "
                        f"Kp={committed_eeprom_data.kp:.6f} Ki={committed_eeprom_data.ki:.6f} "
                        f"Kd={committed_eeprom_data.kd:.6f} "
                        f"AWR={anti_windup_raw_to_percent(committed_eeprom_data.anti_windup):.2f}% "
                        f"RPM={committed_eeprom_data.motor_rpm}"
                    )
                    self._set_sync_enabled(True)
                self._close_eeprom_dialog()
            elif kind == "eeprom-commit-error":
                message = str(payload)
                if self._eeprom_dialog_status_var is not None:
                    self._eeprom_dialog_status_var.set(f"EEPROM commit failed: {message}")
                self._append_log(f"EEPROM commit failed: {message}")
                self._set_eeprom_dialog_editable(True)
            elif kind == "screenshot-complete":
                filename = str(payload)
                self.screenshot_in_progress = False
                self._screenshot_deadline = 0.0
                self._append_log(f"Saved screenshot to {filename}")
                self._update_control_button_states()
            elif kind == "screenshot-error":
                message = str(payload)
                self.screenshot_in_progress = False
                self._screenshot_deadline = 0.0
                self.backend.cancel_screenshot()
                self._append_log(message)
                self._update_control_button_states()

        now = time.monotonic()
        if got_sample and (now - self._last_plot_refresh) >= 0.10:
            self._refresh_plot()
            self._last_plot_refresh = now

        if self.screenshot_in_progress and now >= self._screenshot_deadline:
            self.screenshot_in_progress = False
            self.backend.cancel_screenshot()
            self._append_log("Screenshot request timed out")
            self._update_control_button_states()

        if self.screenshot_in_progress and self.backend.try_finalize_screenshot_if_idle(0.75):
            self.screenshot_in_progress = False
            self._screenshot_deadline = 0.0
            self._append_log("Saved screenshot from idle stream")
            self._update_control_button_states()

        if self._waiting_for_first_sample and self.backend.running and now >= self._first_sample_deadline:
            self._request_firmware_reset(
                "No PID packets received within 1s after Start. Resetting firmware and retrying automatically.",
                auto_restart=True,
            )

        if not self.backend.running and self.start_stop_button.cget("text") == "Stop":
            self.start_stop_button.configure(text="Start")
            self.status_var.set("Stopped")
            self._target_running = False
            self._waiting_for_first_sample = False
            self._first_sample_deadline = 0.0
            self._update_control_button_states()

        self.root.after(40, self._process_events)

    def _on_close(self) -> None:
        self.backend.stop()
        self.gdb_mem.close()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> AppConfig:
    parser = argparse.ArgumentParser(description="PID SWO graphical monitor")
    parser.add_argument("--uid", default="", help="Optional pyOCD probe unique ID")
    parser.add_argument("--target", default="cortex_m", help="pyOCD target")
    parser.add_argument("--system-clock", type=int, default=72_000_000, help="System clock in Hz")
    parser.add_argument("--swo-clock", type=int, default=2_000_000, help="SWO clock in Hz")
    parser.add_argument("--swd-frequency", type=int, default=4_000_000, help="SWD frequency in Hz")
    parser.add_argument(
        "--connect-mode",
        choices=["halt", "pre-reset", "under-reset", "attach"],
        default="attach",
        help="pyOCD connect mode",
    )
    parser.add_argument("--raw-port", type=int, default=3443, help="SWV raw TCP port")
    parser.add_argument("--pid-port", type=int, default=1, help="ITM port for PidLoopType")
    parser.add_argument("--gdb-port", type=int, default=3333, help="pyOCD gdbserver port for memory sync")
    parser.add_argument(
        "--sample-hz",
        type=int,
        default=200,
        help="Expected PID sample rate used for window buffer sizing",
    )

    args = parser.parse_args()
    return AppConfig(
        uid=args.uid,
        target=args.target,
        system_clock=args.system_clock,
        swo_clock=args.swo_clock,
        swd_frequency=args.swd_frequency,
        connect_mode=args.connect_mode,
        raw_port=args.raw_port,
        pid_port=args.pid_port,
        sample_hz=max(1, args.sample_hz),
        gdb_port=args.gdb_port,
    )


def main() -> int:
    config = parse_args()
    app = PIDTuningApp(config)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
