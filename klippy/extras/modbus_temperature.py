# Support for multi-channel RS485 Modbus RTU temperature transmitters
# (e.g. 16-channel PT100 modules commonly sold on Taobao)
#
# Usage:
#
#   [modbus_temperature]          # or [modbus_temperature mybus]
#       serial_path: /dev/ttyUSB0
#       baudrate: 9600
#       slave_id: 1
#       register_start: 0
#       channel_count: 16
#       scale: 0.1
#       signed: True
#       func_code: 3
#
#   [heater_bed]
#       sensor_type: modbus_temperature
#       modbus_channel: 3          # one parameter and done
#
#   [temperature_sensor chamber]
#       sensor_type: modbus_temperature
#       modbus_channel: 0
#
# Copyright (C) 2025  Kalico Community
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import threading

from .danger_options import get_danger_options

REPORT_TIME = 1.0
MIN_REPORT_TIME = 0.3
DEFAULT_BUS_NAME = "default"


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
    def __init__(
        self, serial_path, baudrate, bytesize=8, parity="N", stopbits=1
    ):
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

    def _rx(self, expected_func):
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
        data = self._ser.read(byte_count + 2)
        if len(data) < byte_count + 2:
            raise Exception("Modbus timeout: incomplete payload")
        raw = bytes(head) + data[: byte_count + 2]
        crc_recv = data[byte_count] | (data[byte_count + 1] << 8)
        crc_calc = _crc16(raw[:-2])
        if crc_recv != crc_calc:
            raise Exception("Modbus CRC mismatch")
        return data[:byte_count]

    def read_registers(self, slave_id, reg_addr, count, func_code=0x03):
        with self._lock:
            payload = bytearray()
            payload.append((reg_addr >> 8) & 0xFF)
            payload.append(reg_addr & 0xFF)
            payload.append((count >> 8) & 0xFF)
            payload.append(count & 0xFF)
            self._tx(slave_id, func_code, payload)
            data = self._rx(func_code)
            regs = []
            for i in range(0, count * 2, 2):
                regs.append((data[i] << 8) | data[i + 1])
            return regs


######################################################################
# Bus configuration — registered once per [modbus_temperature] section
######################################################################


class ModbusBus:
    def __init__(self, config, bus_name):
        self.printer = config.get_printer()
        self.bus_name = bus_name

        # Shared hardware / protocol parameters
        self.serial_path = config.get("serial_path", "/dev/ttyUSB0")
        self.baudrate = config.getint("baudrate", 9600)
        self.slave_id = config.getint("slave_id", 1, minval=1, maxval=247)
        self.register_start = config.getint(
            "register_start", 0, minval=0, maxval=0xFFFF
        )
        self.channel_count = config.getint(
            "channel_count", 16, minval=1, maxval=128
        )
        self.data_scale = config.getfloat("scale", 0.1)
        self.signed = config.getboolean("signed", True)
        self.func_code = config.getint("func_code", 3, minval=1, maxval=0x7F)
        self.report_time = config.getfloat(
            "report_time", REPORT_TIME, minval=MIN_REPORT_TIME
        )

        self._bus_key = (self.serial_path, self.baudrate, self.slave_id)
        self._is_debug = (
            self.printer.get_start_args().get("debugoutput") is not None
        )

    def open_serial(self):
        """Open the Modbus serial port on first use. Returns ModbusSerial.
        Multiple ModbusBus instances with the same (path, baud, slave) share
        a single ModbusSerial instance — but typically there is only one
        bus per (path, baud, slave)."""
        registry = _modbus_get_registry(self.printer)
        bus = registry.get(self._bus_key)
        if bus is not None:
            return bus
        bus = ModbusSerial(self.serial_path, self.baudrate)
        registry.put(self._bus_key, bus)
        return bus


# Simple process-wide serial instance registry, keyed by printer object
# so tests don't bleed into each other.
_modbus_registries = {}
_modbus_registries_lock = threading.Lock()


def _modbus_get_registry(printer):
    with _modbus_registries_lock:
        key = id(printer)
        if key not in _modbus_registries:
            _modbus_registries[key] = {}
        return _modbus_registries[key]


######################################################################
# Sensor factory — registered under name "modbus_temperature"
######################################################################


class ModbusTemperatureSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self._callback = None

        # Find the bus configuration.  Try in order:
        #   1) an explicit modbus_bus: <name> in the calling section
        #   2) a bus named "default"  ([modbus_temperature] with no suffix)
        #   3) any single [modbus_temperature <any>] section if there's only one
        bus_name = config.get("modbus_bus", None)
        bus = self._lookup_bus(bus_name, config)
        self._bus = bus

        # Channel — the only parameter that must come from the calling section
        self.channel = config.getint("modbus_channel", 0, minval=0)
        if self.channel >= bus.channel_count:
            raise config.error(
                "modbus_temperature: channel %d >= channel_count %d on bus '%s'"
                % (self.channel, bus.channel_count, bus.bus_name)
            )

        # Per-instance overrides (all optional)
        self.data_scale = config.getfloat("scale", bus.data_scale)
        self.signed = config.getboolean("signed", bus.signed)
        self.func_code = config.getint(
            "func_code", bus.func_code, minval=1, maxval=0x7F
        )
        self.register_start = config.getint(
            "register_start", bus.register_start, minval=0, maxval=0xFFFF
        )
        self.channel_count = bus.channel_count
        self.slave_id = bus.slave_id
        self.report_time = config.getfloat(
            "report_time", bus.report_time, minval=MIN_REPORT_TIME
        )
        self._bus_key = bus._bus_key
        self._is_debug = bus._is_debug

        self.temp = self.min_temp = self.max_temp = 0.0
        self.sample_timer = self.reactor.register_timer(
            self._sample_temperature
        )
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )

    def _lookup_bus(self, bus_name, config):
        manager = self.printer.lookup_object("modbus_temperature", None)
        if manager is None:
            raise config.error(
                "modbus_temperature: no [modbus_temperature] section found; "
                "please add one before using sensor_type: modbus_temperature"
            )
        return manager.get_bus(bus_name, config)

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
                self._callback(
                    mcu.estimated_print_time(measured_time), self.temp
                )
            return eventtime + self.report_time

        try:
            serial = self._bus.open_serial()
            regs = serial.read_registers(
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

        if (
            self.temp < self.min_temp or self.temp > self.max_temp
        ) and not get_danger_options().temp_ignore_limits:
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
# Manager: holds the collection of named buses for this printer
######################################################################


class PrinterModbusManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self._buses = {}  # name -> ModbusBus

    def add_bus(self, bus):
        if bus.bus_name in self._buses:
            raise self.printer.config_error(
                "modbus_temperature: bus '%s' defined more than once"
                % (bus.bus_name,)
            )
        self._buses[bus.bus_name] = bus

    def get_bus(self, name, config):
        if name is None:
            # Default: try "default" first, else pick the only configured one
            if DEFAULT_BUS_NAME in self._buses:
                return self._buses[DEFAULT_BUS_NAME]
            if len(self._buses) == 1:
                return next(iter(self._buses.values()))
            if len(self._buses) == 0:
                raise config.error(
                    "modbus_temperature: no bus configured; add a "
                    "[modbus_temperature] section first"
                )
            raise config.error(
                "modbus_temperature: multiple buses configured; please "
                "specify 'modbus_bus: <name>' to choose one"
            )
        if name not in self._buses:
            raise config.error(
                "modbus_temperature: bus '%s' not found; available: %s"
                % (name, ", ".join(sorted(self._buses.keys())))
            )
        return self._buses[name]


######################################################################
# Module entry points
######################################################################


def _get_manager(config):
    printer = config.get_printer()
    manager = printer.lookup_object("modbus_temperature", None)
    if manager is None:
        manager = PrinterModbusManager(config)
        printer.add_object("modbus_temperature", manager)
        # Register the sensor factory once — on first bus section load.
        pheaters = printer.load_object(config, "heaters")
        pheaters.add_sensor_factory(
            "modbus_temperature", ModbusTemperatureSensor
        )
    return manager


def _section_has_user_config(config):
    # True if the section contains real user-provided values.  The empty
    # [modbus_temperature] stub in temperature_sensors.cfg would fail this
    # so we only register the factory without creating a default bus.
    known_keys = {
        "serial_path",
        "baudrate",
        "slave_id",
        "register_start",
        "channel_count",
        "scale",
        "signed",
        "func_code",
        "report_time",
    }
    raw = config.getsection(config.get_name()).get_options()
    return any(k in known_keys for k in raw)


def load_config(config):
    # Handles the bare [modbus_temperature] section (no suffix).
    # Always ensure manager exists and factory is registered.  Only create the
    # "default" bus if the section actually contains user parameters —
    # the temperature_sensors.cfg stub is intentionally empty and must not
    # silently create a default-bus entry.
    manager = _get_manager(config)
    if _section_has_user_config(config):
        manager.add_bus(ModbusBus(config, DEFAULT_BUS_NAME))
    return manager


def load_config_prefix(config):
    # Handles [modbus_temperature mybus] → named bus
    manager = _get_manager(config)
    bus_name = config.get_name().split(" ", 1)[1]
    manager.add_bus(ModbusBus(config, bus_name))
    return manager
