# Support for multi-channel RS485 Modbus RTU temperature transmitters
# (e.g. 16-channel PT100 modules commonly sold on Taobao)
#
# Copyright (C) 2025  Kalico Community
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import threading

from .danger_options import get_danger_options

REPORT_TIME = 1.0
MIN_REPORT_TIME = 0.3
DEFAULT_REG_START = 0x0000
DEFAULT_CHANNEL_COUNT = 16


######################################################################
# Minimal Modbus RTU master (no external dependency on pymodbus)
######################################################################

def _crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


class ModbusSerial:
    def __init__(self, serial_path, baudrate, bytesize=8, parity="N", stopbits=1):
        import serial
        self._ser = serial.Serial(
            port=serial_path,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=1.0,
        )
        self._lock = threading.Lock()

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass

    def _tx(self, slave_id, func_code, payload):
        frame = bytearray()
        frame.append(slave_id & 0xFF)
        frame.append(func_code & 0xFF)
        frame.extend(payload)
        crc = _crc16(frame)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)
        self._ser.reset_input_buffer()
        self._ser.write(bytes(frame))

    def _rx(self, expected_func, expected_data_bytes=None):
        head = self._ser.read(3)
        if len(head) < 3:
            raise Exception("Modbus timeout: no response header")
        slave_id, func = head[0], head[1]
        if func & 0x80:
            err_byte = self._ser.read(1)
            err_code = err_byte[0] if err_byte else -1
            raise Exception("Modbus exception response: code=%d" % (err_code,))
        if func != expected_func:
            raise Exception("Modbus unexpected func code: %d" % (func,))
        byte_count = head[2]
        if expected_data_bytes is not None and byte_count != expected_data_bytes:
            raise Exception("Modbus unexpected byte count: %d" % (byte_count,))
        data = self._ser.read(byte_count + 2)
        if len(data) < byte_count + 2:
            raise Exception("Modbus timeout: incomplete payload")
        raw = bytes(head) + data[:byte_count + 2]
        crc_recv = data[byte_count] | (data[byte_count + 1] << 8)
        crc_calc = _crc16(raw[:-2])
        if crc_recv != crc_calc:
            raise Exception("Modbus CRC mismatch")
        return data[:byte_count]

    def read_holding_registers(self, slave_id, reg_addr, count, func_code=0x03):
        with self._lock:
            payload = bytearray()
            payload.append((reg_addr >> 8) & 0xFF)
            payload.append(reg_addr & 0xFF)
            payload.append((count >> 8) & 0xFF)
            payload.append(count & 0xFF)
            self._tx(slave_id, func_code, payload)
            data = self._rx(func_code, expected_data_bytes=count * 2)
            regs = []
            for i in range(0, count * 2, 2):
                regs.append((data[i] << 8) | data[i + 1])
            return regs


######################################################################
# Shared bus registry (one serial port = one ModbusSerial instance)
######################################################################

class ModbusBusRegistry:
    def __init__(self):
        self._buses = {}  # key: (serial_path, baudrate, slave_id)

    def get(self, key):
        return self._buses.get(key)

    def put(self, key, bus):
        self._buses[key] = bus

    def close_all(self):
        for bus in list(self._buses.values()):
            bus.close()
        self._buses.clear()


_modbus_registry = ModbusBusRegistry()


######################################################################
# Sensor: a single channel out of N on a shared Modbus bus
######################################################################

class ModbusTemperature:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]

        # Bus parameters (shared across all channels of the same device)
        self.serial_path = config.get("modbus_serial", "/dev/ttyUSB0")
        self.baudrate = config.getint("modbus_baud", 9600)
        self.slave_id = config.getint("modbus_slave", 1, minval=1, maxval=247)

        # Data layout
        self.register_start = config.getint(
            "modbus_register_start", DEFAULT_REG_START, minval=0, maxval=0xFFFF
        )
        self.channel_count = config.getint(
            "modbus_channel_count", DEFAULT_CHANNEL_COUNT, minval=1, maxval=128
        )
        self.channel = config.getint("modbus_channel", 0, minval=0)
        self.data_scale = config.getfloat("modbus_scale", 0.1)
        self.signed = config.getboolean("modbus_signed", True)
        self.func_code = config.getint("modbus_func_code", 0x03)

        if self.channel >= self.channel_count:
            raise config.error(
                "modbus_temperature: channel %d >= channel_count %d"
                % (self.channel, self.channel_count)
            )

        self.report_time = config.getfloat(
            "modbus_report_time", REPORT_TIME, minval=MIN_REPORT_TIME
        )
        self.temp = self.min_temp = self.max_temp = 0.0
        self._last_read_time = 0.0
        self._callback = None

        bus_key = (self.serial_path, self.baudrate, self.slave_id)
        self._bus_key = bus_key

        # Debug mode: skip hardware and just return 0.0
        if self.printer.get_start_args().get("debugoutput") is not None:
            self._is_debug = True
            self._bus = None
        else:
            self._is_debug = False
            self._bus = _modbus_registry.get(bus_key)
            if self._bus is None:
                try:
                    self._bus = ModbusSerial(
                        self.serial_path, self.baudrate
                    )
                    _modbus_registry.put(bus_key, self._bus)
                except Exception as e:
                    raise config.error(
                        "modbus_temperature: Unable to open %s (%s)"
                        % (self.serial_path, e)
                    )

        self.sample_timer = self.reactor.register_timer(self._sample_temperature)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_connect(self):
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def _sample_temperature(self, eventtime):
        if self._is_debug:
            self.temp = 0.0
            if self._callback is not None:
                mcu = self.printer.lookup_object("mcu")
                measured_time = self.reactor.monotonic()
                self._callback(mcu.estimated_print_time(measured_time), self.temp)
            return eventtime + self.report_time

        try:
            regs = self._bus.read_holding_registers(
                self.slave_id,
                self.register_start,
                self.channel_count,
                self.func_code,
            )
        except Exception as e:
            logging.info("modbus_temperature: read error (%s)", e)
            return eventtime + self.report_time

        raw = regs[self.channel]
        if self.signed and raw & 0x8000:
            raw = raw - 0x10000
        self.temp = float(raw) * self.data_scale

        if (self.temp < self.min_temp or self.temp > self.max_temp) \
                and not get_danger_options().temp_ignore_limits:
            self.printer.invoke_shutdown(
                "MODBUS temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp)
            )

        if self._callback is not None:
            mcu = self.printer.lookup_object("mcu")
            measured_time = self.reactor.monotonic()
            self._callback(mcu.estimated_print_time(measured_time), self.temp)

        return measured_time + self.report_time

    def get_status(self, eventtime):
        return {"temperature": round(self.temp, 2)}


######################################################################
# Module entrypoint — register as a temperature sensor factory
######################################################################

def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("modbus_temperature", ModbusTemperature)
