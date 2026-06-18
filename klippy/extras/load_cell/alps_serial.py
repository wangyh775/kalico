# ALPS USB 串口压力传感器支持
#
# Copyright (C) 2024 Your Name <your.email@example.com>
#
# 本文件可能在 GNU GPLv3 许可证条款下分发。
from __future__ import annotations

import logging
import queue
import re
import threading
import time
from typing import Optional

import serial

from klippy.extras import bulk_sensor
from klippy.extras.load_cell.interfaces import (
    BulkAdcData,
    BulkAdcDataCallback,
    LoadCellSensor,
)
from klippy.mcu import MCU

# 模块级常量
ALPS_DEFAULT_BAUDRATE = 9600
ALPS_DEFAULT_TIMEOUT = 0.05
ALPS_FIXED_SPS = 2600
ALPS_UPDATE_INTERVAL = 0.10
ALPS_MAX_CONSECUTIVE_FAILURES = 100
ALPS_BUFFER_MULTIPLIER = 3


class VirtualMCU:
    """
    USB 串口传感器的虚拟 MCU 对象

    由于 ALPS 传感器通过 USB 串口直接连接到主机，
    没有真实的 Klipper MCU 固件。此类提供最小化的 MCU 接口兼容性，
    防止 McuLoadCellProbe 等组件因 None 引用而崩溃。

    注意：此虚拟 MCU 不支持实际的固件命令操作。
    任何尝试调用 MCU 固件通信方法都会抛出明确的异常。
    """

    def __init__(self, printer, sensor_name: str):
        self._printer = printer
        self._sensor_name = sensor_name
        self._oid_counter = 0

    def get_printer(self):
        return self._printer

    def get_name(self) -> str:
        return f"virtual_{self._sensor_name}"

    def create_oid(self) -> int:
        oid = self._oid_counter
        self._oid_counter += 1
        return oid

    def alloc_command_queue(self):
        raise NotImplementedError(
            f"VirtualMCU for '{self._sensor_name}' does not support command queues. "
            f"USB serial sensors cannot execute MCU firmware commands."
        )

    def register_config_callback(self, callback):
        logging.warning(
            "VirtualMCU: Ignoring config callback registration for '%s'. "
            "USB serial sensors do not support MCU firmware configuration.",
            self._sensor_name,
        )

    def add_config_cmd(self, cmd: str):
        logging.debug(
            "VirtualMCU: Ignoring config command for '%s': %s",
            self._sensor_name,
            cmd,
        )

    def lookup_command(self, msgformat: str, cq=None):
        raise NotImplementedError(
            f"VirtualMCU for '{self._sensor_name}' does not support command lookup. "
            f"Cannot execute firmware command: {msgformat}"
        )

    def lookup_query_command(
        self, msgformat: str, respformat: str, oid=None, cq=None, is_async=False
    ):
        raise NotImplementedError(
            f"VirtualMCU for '{self._sensor_name}' does not support query commands. "
            f"Cannot query firmware: {msgformat}"
        )

    def register_response(self, callback, msg_name: str, oid=None):
        logging.warning(
            "VirtualMCU: Ignoring response registration for '%s'. "
            "USB serial sensors handle data internally.",
            self._sensor_name,
        )

    def print_time_to_clock(self, print_time: float) -> int:
        """使用主机单调时间作为近似（假设 1GHz 时钟）"""
        return int(print_time * 1e9)

    def seconds_to_clock(self, seconds: float) -> int:
        """将秒转换为时钟刻度（假设 1GHz 时钟）"""
        return int(seconds * 1e9)

    def clock_to_print_time(self, clock: int) -> float:
        """将时钟刻度转换为打印时间"""
        return clock / 1e9

    def clock32_to_clock64(self, clock32: int) -> int:
        """简单的 32 位到 64 位转换"""
        return clock32 & 0xFFFFFFFF


class ALPSSerialSensor(LoadCellSensor):
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]

        # 串口配置
        self.serial_port = config.get("serial_port")
        self.baudrate = config.getint("baudrate", default=9600)
        self.timeout = config.getfloat("timeout", default=1.0)

        # 固定采样率：2.6 kHz（不可修改）
        self.sps = ALPS_FIXED_SPS

        # 数据解析模式：a=**,b=**（支持整数和小数，逗号后允许空格）
        self.data_pattern = re.compile(r"a=(-?\d+\.?\d*),\s*b=(-?\d+\.?\d*)")

        # 状态跟踪
        self.last_error_count = 0
        self.consecutive_fails = 0
        self.is_reading = False
        self.read_thread = None

        # 使用队列在线程间传递数据（线程安全）
        buffer_size = int(
            self.sps * ALPS_UPDATE_INTERVAL * ALPS_BUFFER_MULTIPLIER
        )
        self.data_queue: queue.Queue = queue.Queue(maxsize=buffer_size)

        # 客户端回调（通过 BatchBulkHelper 管理）
        self.clients = []

        # 初始化串口连接
        self.serial_conn: Optional[serial.Serial] = None

        # 使用标准批量处理助手
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer,
            self._process_batch,
            self._start_measurements,
            self._stop_measurements,
            ALPS_UPDATE_INTERVAL,
        )

        # 注册事件处理器
        printer.register_event_handler("klippy:ready", self._handle_ready)
        printer.register_event_handler("klippy:shutdown", self._handle_shutdown)

        logging.info(
            "ALPS sensor '%s' initialized on port %s at fixed 2600 Hz",
            self.name,
            self.serial_port,
        )

    def _handle_ready(self):
        """Klipper 就绪时开始读取"""
        try:
            self._open_serial()
            self._send_activation()
            self._start_reading()
            logging.info(
                "ALPS sensor '%s' started on port %s at fixed %d Hz",
                self.name,
                self.serial_port,
                self.sps,
            )
        except Exception as e:
            logging.error(
                "Failed to start ALPS sensor '%s': %s", self.name, str(e)
            )
            raise self.printer.command_error(
                f"ALPS sensor initialization failed: {str(e)}"
            )

    def _send_activation(self):
        """发送激活命令使传感器开始输出数据"""
        try:
            self.serial_conn.write(b"rt\r\n")
            time.sleep(0.3)
            # 读取 rt 响应
            if self.serial_conn.in_waiting > 0:
                resp = self.serial_conn.read(self.serial_conn.in_waiting)
                logging.info(
                    "ALPS sensor '%s' rt response: %s",
                    self.name,
                    resp.decode("utf-8", errors="ignore").strip(),
                )
            self.serial_conn.write(b"v\r\n")
            time.sleep(0.3)
            # 读取 v 响应（数据流的前几行）
            if self.serial_conn.in_waiting > 0:
                resp = self.serial_conn.read(min(self.serial_conn.in_waiting, 200))
                logging.info(
                    "ALPS sensor '%s' v response (first bytes): %s",
                    self.name,
                    resp.decode("utf-8", errors="ignore").strip()[:100],
                )
            logging.info("ALPS sensor '%s' activation commands sent", self.name)
        except Exception as e:
            raise self.printer.command_error(
                f"ALPS sensor activation failed: {str(e)}"
            )

    def _handle_shutdown(self):
        """关闭时停止读取并清理资源"""
        self._stop_reading()
        self._close_serial()

        # 清空队列
        while not self.data_queue.empty():
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                break

        # 重置状态
        self.consecutive_fails = 0

        logging.info("ALPS sensor '%s' stopped and cleaned up", self.name)

    def _open_serial(self):
        """打开串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            return

        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            logging.info(
                "Serial port %s opened successfully at %d baud",
                self.serial_port,
                self.baudrate,
            )
        except serial.SerialException as e:
            raise self.printer.command_error(
                f"Failed to open serial port {self.serial_port}: {str(e)}"
            )

    def _close_serial(self):
        """关闭串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
                logging.info("Serial port %s closed", self.serial_port)
            except Exception as e:
                logging.error("Error closing serial port: %s", str(e))

    def _start_reading(self):
        """启动后台读取线程"""
        if self.is_reading:
            return

        self.is_reading = True
        self.read_thread = threading.Thread(
            target=self._read_loop, daemon=True, name=f"ALPS-{self.name}"
        )
        self.read_thread.start()

    def _stop_reading(self):
        """停止后台读取线程"""
        self.is_reading = False
        if self.read_thread:
            self.read_thread.join(timeout=2.0)
            self.read_thread = None

    def _read_loop(self):
        """解析串口数据的主循环（运行在独立线程中）"""
        line_buffer = ""

        while self.is_reading:
            try:
                if not self.serial_conn or not self.serial_conn.is_open:
                    time.sleep(0.1)
                    continue

                # 读取可用数据
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    line_buffer += data.decode("utf-8", errors="ignore")

                    # 处理完整的行
                    while "\n" in line_buffer:
                        line, line_buffer = line_buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            self._process_line(line)
                else:
                    time.sleep(0.01)

            except Exception as e:
                logging.error("ALPS sensor read error: %s", str(e))
                self.last_error_count += 1
                time.sleep(0.1)

    def _process_line(self, line: str):
        """解析单行传感器数据并放入队列"""
        match = self.data_pattern.search(line)
        if not match:
            self.consecutive_fails += 1
            if self.consecutive_fails > ALPS_MAX_CONSECUTIVE_FAILURES:
                logging.warning(
                    "ALPS sensor '%s' has %d consecutive parse failures",
                    self.name,
                    self.consecutive_fails,
                )
            return

        try:
            # 提取 b 值（卡尔曼滤波后）
            b_value = float(match.group(2))

            # 获取当前时间戳（使用单调时间）
            eventtime = time.monotonic()

            # 将数据放入队列（只存储时间戳和原始值，2 元素元组）
            raw_value = int(b_value * 1000)
            try:
                self.data_queue.put_nowait((eventtime, raw_value))
            except queue.Full:
                # 队列满时丢弃最旧的数据
                try:
                    self.data_queue.get_nowait()
                    self.data_queue.put_nowait((eventtime, raw_value))
                except queue.Empty:
                    pass

            # 成功解析，重置失败计数
            self.consecutive_fails = 0

        except (ValueError, IndexError) as e:
            logging.debug(
                "Failed to parse ALPS data line '%s': %s", line, str(e)
            )
            self.last_error_count += 1
            self.consecutive_fails += 1

    def get_mcu(self) -> MCU:
        """
        返回虚拟 MCU 对象

        ALPS 传感器通过 USB 串口直接连接，没有真实的 Klipper MCU。
        返回 VirtualMCU 实例以符合 LoadCellSensor 接口契约，
        同时防止调用方因 None 引用而崩溃。

        返回：
            VirtualMCU: 虚拟 MCU 对象，提供最小化的接口兼容性
        """
        if not hasattr(self, "_virtual_mcu"):
            self._virtual_mcu = VirtualMCU(self.printer, self.name)
        return self._virtual_mcu

    def get_samples_per_second(self) -> int:
        """返回固定的 2600 Hz 采样率"""
        return self.sps

    def get_range(self) -> tuple[int, int]:
        """返回传感器范围（根据 ALPS 传感器规格调整）"""
        # 假设 24 位 ADC 范围，根据需要调整
        return -0x800000, 0x7FFFFF

    def get_channel_count(self) -> int:
        """返回通道数（ALPS 提供单个压力通道）"""
        return 1

    def add_client(self, callback: BulkAdcDataCallback):
        """添加客户端以接收传感器数据（委托给 BatchBulkHelper）"""
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid: int):
        """USB 串口传感器的空操作（不需要 MCU 附加）"""
        pass

    # BatchBulkHelper 回调方法
    def _start_measurements(self):
        """开始测量时的回调"""
        self.last_error_count = 0
        self.consecutive_fails = 0
        logging.info("ALPS sensor '%s' measurements started", self.name)

    def _stop_measurements(self):
        """停止测量时的回调"""
        logging.info("ALPS sensor '%s' measurements stopped", self.name)

    def _process_batch(self, eventtime) -> BulkAdcData:
        """批量处理回调，由 BatchBulkHelper 调用"""
        samples = []

        # 从队列中收集样本
        while not self.data_queue.empty():
            try:
                samples.append(self.data_queue.get_nowait())
            except queue.Empty:
                break

        if not samples:
            return None

        # 记录缓冲区使用率（用于性能监控）
        buffer_usage = self.data_queue.qsize() / self.data_queue.maxsize * 100
        if buffer_usage > 80:
            logging.warning(
                "ALPS sensor '%s' queue usage high: %.1f%%",
                self.name,
                buffer_usage,
            )

        # 返回批量数据
        msg = {"data": samples, "errors": self.last_error_count, "overflows": 0}

        # 重置错误计数
        self.last_error_count = 0

        return msg


def load_config(config):
    """加载 ALPS 传感器配置"""
    return ALPSSerialSensor(config)


ALPS_SENSOR_TYPE = {"alps_serial": ALPSSerialSensor}
