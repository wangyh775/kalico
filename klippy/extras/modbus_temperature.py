# Support for multi-channel RS485 Modbus RTU temperature transmitters
# (e.g. 16-channel PT100 modules commonly sold on Taobao).
#
# -------------------------------------------------------------------
# Configuration flow
# -------------------------------------------------------------------
# 1. temperature_sensors.cfg ships a stub named-prefix section:
#       [modbus_temperature factory]
#    It triggers load_config_prefix() which only registers
#    the sensor factory with heaters. It does NOT claim the plain
#    "modbus_temperature" name in printer.objects, so the user's own
#    [modbus_temperature] section is processed normally.
#
# 2. User defines at least one real bus in printer.cfg:
#       [modbus_temperature]           # default bus
#       serial_path: /dev/ttyUSB0
#       baudrate: 9600
#       slave_id: 1
#       register_start: 0
#       channel_count: 16
#       scale: 0.1
#       signed: True
#       func_code: 3                    # 3 = holding regs, 4 = input regs
#       report_time: 1.0
#    A second bus uses a named prefix:
#       [modbus_temperature second_bus]
#       serial_path: /dev/ttyUSB1
#       ...
#
# 3. Route a heater/sensor to a bus and channel:
#       [heater_bed]
#       sensor_type: modbus_temperature
#       modbus_channel: 3               # 4th channel (0-based)
#       # modbus_bus: second_bus        # optional: needed only with >1 bus
#
#    Pure monitoring (no heater loop):
#       [temperature_sensor chamber]
#       sensor_type: modbus_temperature
#       modbus_channel: 1
# -------------------------------------------------------------------
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
MANAGER_OBJECT_NAME = "modbus_temperature_manager"
FACTORY_LOADER_SECTION = "modbus_temperature factory"
MAX_CONSECUTIVE_ERRORS = 5
BACKOFF_STEP = 1.0  # seconds added to report_time after a failed read
REGISTER_START_DEFAULT = 0
CHANNEL_COUNT_DEFAULT = 16
DATA_SCALE_DEFAULT = 0.1
READ_FUNCS = frozenset({0x03, 0x04})

# Keys that distinguish a bus configuration from a mere factory-loader stub
BUS_CONFIG_KEYS = frozenset(
    {
        "serial_path",
        "baudrate",
        "slave_id",
        "register_start",
        "channel_count",
        "scale",
        "signed",
        "func_code",
        "report_time",
        "bytesize",
        "parity",
        "stopbits",
    }
)


######################################################################
# Minimal Modbus RTU master (no pymodbus dependency)
######################################################################


def _crc16(data):
    """Standard Modbus CRC-16 (polynomial 0xA001, init 0xFFFF)."""
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
    """Thread-safe wrapper around a single serial port, exposing
    locked register reads. Callers share a ModbusSerial through
    _SerialRegistry when their bus parameters match."""

    def __init__(
        self, serial_path, baudrate, bytesize=8, parity="N", stopbits=1
    ):
        import serial

        self._serial_path = serial_path
        self._ser = serial.Serial(
            port=serial_path,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=1.0,
        )
        self._lock = threading.Lock()

    @property
    def serial_path(self):
        return self._serial_path

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass

    # Low-level framing --------------------------------------------------

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

    def _rx(self, expected_slave_id, expected_func):
        head = self._ser.read(3)
        if len(head) < 3:
            raise Exception("Modbus timeout: no response header")
        slave_id, func = head[0], head[1]
        if slave_id != expected_slave_id:
            raise Exception(
                "Modbus unexpected slave id: expected=%d got=%d"
                % (expected_slave_id, slave_id)
            )
        if func & 0x80:
            err_byte = self._ser.read(1)
            err_code = err_byte[0] if err_byte else -1
            raise Exception(
                "Modbus exception response: slave=%d code=%d"
                % (slave_id, err_code)
            )
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
        """Read `count` 16-bit registers starting at `reg_addr`."""
        if func_code not in READ_FUNCS:
            raise Exception(
                "Modbus unsupported function code: 0x%02x" % (func_code,)
            )
        with self._lock:
            payload = bytearray()
            payload.append((reg_addr >> 8) & 0xFF)
            payload.append(reg_addr & 0xFF)
            payload.append((count >> 8) & 0xFF)
            payload.append(count & 0xFF)
            self._tx(slave_id, func_code, payload)
            data = self._rx(slave_id, func_code)
            return [
                (data[i] << 8) | data[i + 1] for i in range(0, count * 2, 2)
            ]


######################################################################
# Per-printer serial registry (avoids opening the same port twice)
######################################################################


class _SerialRegistry:
    """Keyed by bus_key. Serializes open() so racing callers share
    the single ModbusSerial instance."""

    def __init__(self):
        self._buses = {}
        self._lock = threading.Lock()

    def get_or_open(self, bus_key, build_fn):
        with self._lock:
            bus = self._buses.get(bus_key)
            if bus is not None:
                return bus
            bus = build_fn()
            self._buses[bus_key] = bus
            return bus

    def close_all(self):
        with self._lock:
            for bus in self._buses.values():
                bus.close()
            self._buses.clear()


# printer.id() -> _SerialRegistry
_registries = {}
_registries_lock = threading.Lock()


def _serial_registry_for(printer):
    with _registries_lock:
        key = id(printer)
        if key not in _registries:
            _registries[key] = _SerialRegistry()
        return _registries[key]


######################################################################
# Bus configuration (one per [modbus_temperature] or
# [modbus_temperature name] section)
######################################################################


class ModbusBus:
    """Parameters for one physical RTU bus.

    The serial port is opened lazily on first sample — so config-time
    errors surface cleanly and the debugoutput path never touches
    hardware."""

    def __init__(self, config, bus_name):
        self.printer = config.get_printer()
        self.bus_name = bus_name

        # serial / transport
        self.serial_path = config.get("serial_path", "/dev/ttyUSB0")
        self.baudrate = config.getint("baudrate", 9600)
        self.bytesize = config.getint("bytesize", 8)
        self.parity = config.get("parity", "N").upper()
        self.stopbits = config.getint("stopbits", 1)

        # slave / register layout
        self.slave_id = config.getint("slave_id", 1, minval=1, maxval=247)
        self.register_start = config.getint(
            "register_start",
            REGISTER_START_DEFAULT,
            minval=0,
            maxval=0xFFFF,
        )
        self.channel_count = config.getint(
            "channel_count",
            CHANNEL_COUNT_DEFAULT,
            minval=1,
            maxval=128,
        )
        self.data_scale = config.getfloat("scale", DATA_SCALE_DEFAULT)
        self.signed = config.getboolean("signed", True)
        self.func_code = config.getint("func_code", 3, minval=1, maxval=0x7F)
        if self.func_code not in READ_FUNCS:
            raise config.error(
                "modbus_temperature: func_code must be 3 or 4"
                " (other function codes are not supported)"
            )
        self.report_time = config.getfloat(
            "report_time",
            REPORT_TIME,
            minval=MIN_REPORT_TIME,
        )

        # Identifies a unique physical bus. Used for port sharing.
        self.bus_key = (
            self.serial_path,
            self.baudrate,
            self.bytesize,
            self.parity,
            self.stopbits,
            self.slave_id,
        )

        # Debug path: avoid any hardware access when klippy is invoked
        # with --debug-output (for tests / dry-runs).
        self.is_debug = (
            self.printer.get_start_args().get("debugoutput") is not None
        )

    def open_serial(self):
        registry = _serial_registry_for(self.printer)

        def _build():
            return ModbusSerial(
                self.serial_path,
                self.baudrate,
                self.bytesize,
                self.parity,
                self.stopbits,
            )

        return registry.get_or_open(self.bus_key, _build)

    def __repr__(self):
        return "<ModbusBus name=%s slave=%d path=%s>" % (
            self.bus_name,
            self.slave_id,
            self.serial_path,
        )


######################################################################
# Sensor — one per heater/temperature_sensor section using
# sensor_type: modbus_temperature
######################################################################


class ModbusTemperatureSensor:
    """Temperature source for a single channel on a shared bus."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self._callback = None

        # Resolve the bus
        bus_name = config.get("modbus_bus", None)
        bus = self._lookup_bus(bus_name, config)
        self._bus = bus

        # Channel selection
        self.channel = config.getint("modbus_channel", 0, minval=0)
        if self.channel >= bus.channel_count:
            raise config.error(
                "modbus_temperature: channel %d >= channel_count %d on bus '%s'"
                % (self.channel, bus.channel_count, bus.bus_name)
            )

        # Per-sensor overrides (optional — fall back to the bus defaults)
        self.data_scale = config.getfloat("scale", bus.data_scale)
        self.signed = config.getboolean("signed", bus.signed)
        self.func_code = config.getint(
            "func_code",
            bus.func_code,
            minval=1,
            maxval=0x7F,
        )
        self.register_start = config.getint(
            "register_start",
            bus.register_start,
            minval=0,
            maxval=0xFFFF,
        )
        self.channel_count = bus.channel_count
        self.slave_id = bus.slave_id
        self.report_time = config.getfloat(
            "report_time",
            bus.report_time,
            minval=MIN_REPORT_TIME,
        )
        if self.func_code not in READ_FUNCS:
            raise config.error(
                "modbus_temperature: per-sensor func_code must be 3 or 4"
            )
        self._is_debug = bus.is_debug
        self._bus_key = bus.bus_key

        # Runtime state
        self.temp = self.min_temp = self.max_temp = 0.0
        self._consecutive_errors = 0

        self.sample_timer = self.reactor.register_timer(
            self._sample_temperature
        )
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )

    # helpers ------------------------------------------------------------

    def _lookup_bus(self, bus_name, config):
        manager = self.printer.lookup_object(MANAGER_OBJECT_NAME, None)
        if manager is None:
            raise config.error(
                "modbus_temperature: no bus section loaded; add a "
                "[modbus_temperature] section to your config."
            )
        return manager.get_bus(bus_name, config)

    def _handle_connect(self):
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    # Heater protocol ----------------------------------------------------

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    # sampling -----------------------------------------------------------

    def _log_read_error(self, msg):
        # First few failures are visible at info level so users
        # see what's happening; after several consecutive failures we
        # throttle the log by downgrading repeated entries to
        # debug-level and bump report_time.
        if self._consecutive_errors in (1, MAX_CONSECUTIVE_ERRORS):
            logging.warning(
                "modbus_temperature: read error on channel %d of bus '%s': %s",
                self.channel,
                self._bus.bus_name,
                msg,
            )
        else:
            logging.info(
                "modbus_temperature: read error on channel %d of bus '%s': %s",
                self.channel,
                self._bus.bus_name,
                msg,
            )

    def _sample_temperature(self, eventtime):
        if self._is_debug:
            self.temp = 0.0
            if self._callback is not None:
                mcu = self.printer.lookup_object("mcu")
                now = self.reactor.monotonic()
                self._callback(mcu.estimated_print_time(now), self.temp)
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
            self._consecutive_errors += 1
            next_time = self.report_time
            if self._consecutive_errors > MAX_CONSECUTIVE_ERRORS:
                next_time = self.report_time + BACKOFF_STEP
            self._log_read_error(str(e))
            return eventtime + next_time

        # Happy path
        self._consecutive_errors = 0
        raw = regs[self.channel]
        if self.signed and raw & 0x8000:
            raw = raw - 0x10000
        self.temp = float(raw) * self.data_scale

        if (
            self.temp < self.min_temp or self.temp > self.max_temp
        ) and not get_danger_options().temp_ignore_limits:
            self.printer.invoke_shutdown(
                "MODBUS temperature %0.1f outside range %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp)
            )

        if self._callback is not None:
            mcu = self.printer.lookup_object("mcu")
            now = self.reactor.monotonic()
            self._callback(mcu.estimated_print_time(now), self.temp)
        return now + self.report_time

    def get_status(self, eventtime):
        return {"temperature": round(self.temp, 2)}


######################################################################
# Manager: holds all named buses configured by the user
######################################################################


class PrinterModbusManager:
    """Container for named ModbusBus instances.

    Initialized lazily — either from the factory-loader stub in
    temperature_sensors.cfg, or from the first user bus section.
    """

    def __init__(self, printer):
        self.printer = printer
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
            # Implicit routing: prefer the default bus; if there is only
            # one configured bus of any name, use it; otherwise require
            # the user to disambiguate.
            if DEFAULT_BUS_NAME in self._buses:
                return self._buses[DEFAULT_BUS_NAME]
            if len(self._buses) == 1:
                return next(iter(self._buses.values()))
            if len(self._buses) == 0:
                raise config.error(
                    "modbus_temperature: no bus configured; add a "
                    "[modbus_temperature] section first."
                )
            raise config.error(
                "modbus_temperature: %d buses configured; please "
                "specify 'modbus_bus: <name>' to choose one of: %s"
                % (len(self._buses), ", ".join(sorted(self._buses)))
            )
        if name not in self._buses:
            raise config.error(
                "modbus_temperature: bus '%s' not found; available: %s"
                % (name, ", ".join(sorted(self._buses)))
            )
        return self._buses[name]


######################################################################
# Module entry points
######################################################################


def _get_manager(printer, config):
    """Return the printer-level manager, creating it (and registering
    the sensor factory into heaters) on first call."""
    manager = printer.lookup_object(MANAGER_OBJECT_NAME, None)
    if manager is not None:
        return manager
    manager = PrinterModbusManager(printer)
    printer.add_object(MANAGER_OBJECT_NAME, manager)
    # Register factory into heaters so later calls to setup_sensor()
    # find it. This runs very early in startup, from
    # [modbus_temperature factory] which heaters.load_config pulls in
    # before any user section is processed.
    pheaters = printer.load_object(config, "heaters")
    pheaters.add_sensor_factory("modbus_temperature", ModbusTemperatureSensor)
    return manager


def _section_has_bus_config(config):
    """True if this config section contains at least one real bus-
    configuration key. The empty [modbus_temperature factory] stub
    always returns False."""
    if config.get_name() == FACTORY_LOADER_SECTION:
        return False
    raw = config.get_options()
    return any(k in BUS_CONFIG_KEYS for k in raw)


def load_config(config):
    """Called for the bare [modbus_temperature] section (default bus)."""
    printer = config.get_printer()
    manager = _get_manager(printer, config)
    if _section_has_bus_config(config):
        manager.add_bus(ModbusBus(config, DEFAULT_BUS_NAME))
    return manager


def load_config_prefix(config):
    """Called for any [modbus_temperature <suffix>] section.

    The special suffix "factory" is reserved for the temperature_sensors.cfg
    stub. Any other suffix creates a normal named bus."""
    printer = config.get_printer()
    manager = _get_manager(printer, config)
    section_name = config.get_name()
    if section_name == FACTORY_LOADER_SECTION:
        # Pure factory-loader — no bus creation.
        return manager
    # Regular named bus.
    bus_name = section_name.split(" ", 1)[1]
    manager.add_bus(ModbusBus(config, bus_name))
    return manager
