# Virtual temperature sensor that fuses multiple Modbus channels into
# one representative temperature for heater control loops.
#
# Typical use case: a heated chamber with 12 sensors spread across
# thermal zones.  The fused output feeds [heater_generic chamber]'s
# PID controller as a single, stable, confidence-weighted reading.
#
# Module shape: sensor_type: temperature_fusion factory registration,
# isomorphic to temperature_combined.  Input comes only from direct
# Modbus bus reads (via modbus_temperature_manager).
#
# Copyright (C) 2025  Kalico Community
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .danger_options import get_danger_options

REPORT_TIME = 1.0
MIN_REPORT_TIME = 0.3
MANAGER_OBJECT_NAME = "modbus_temperature_manager"

# Confidence formula weight: alpha * valid_ratio + (1-alpha) * consistency
CONFIDENCE_ALPHA = 0.7

# Default modified-Z-score threshold (Iglewicz & Hoaglin)
DEFAULT_OUTLIER_ZSCORE = 3.5


######################################################################
# Data structures
######################################################################


@dataclass
class SensorSample:
    channel: int
    raw_value: int
    temperature: float
    weight: float
    zone: str | None = None
    position: tuple | None = None
    is_valid: bool = True
    timestamp: float = 0.0


@dataclass
class FusionResult:
    temperature: float
    confidence: float
    valid_samples: int
    excluded_samples: list = field(default_factory=list)


######################################################################
# Strategy base class and built-in strategies
######################################################################


class FusionStrategy:
    """Abstract base class for all fusion strategies."""

    STRATEGY_CONFIG_KEYS: list = []

    def __init__(self, strategy_config: dict, num_channels: int):
        self.num_channels = num_channels

    def update(self, samples, eventtime):
        raise NotImplementedError

    def fuse(self):
        raise NotImplementedError

    def get_diagnostics(self):
        return {}

    def reset(self):
        pass


def _median(values):
    """Return the median of a list of numbers (list is sorted copy)."""
    s = sorted(values)
    n = len(s)
    if n == 0:
        return 0.0
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return (s[mid - 1] + s[mid]) / 2.0


def _compute_confidence(valid_count, total_count, consistency):
    """Unified confidence formula: alpha*valid_ratio + (1-alpha)*consistency."""
    if total_count <= 0:
        return 0.0
    valid_ratio = valid_count / total_count
    return max(
        0.0,
        min(
            1.0,
            CONFIDENCE_ALPHA * valid_ratio
            + (1.0 - CONFIDENCE_ALPHA) * max(0.0, consistency),
        ),
    )


class WeightedMeanStrategy(FusionStrategy):
    """Weighted mean with MAD-based outlier rejection."""

    STRATEGY_CONFIG_KEYS = ["outlier_zscore"]

    def __init__(self, strategy_config, num_channels):
        super().__init__(strategy_config, num_channels)
        self.outlier_zscore = strategy_config.get(
            "outlier_zscore", DEFAULT_OUTLIER_ZSCORE
        )
        self._last_samples = []
        self._excluded = []
        self._mad = 0.0
        self._weighted_mean = 0.0
        self._z_scores = []

    def update(self, samples, eventtime):
        self._last_samples = samples
        self._excluded = []
        self._z_scores = []

        if not samples:
            self._weighted_mean = 0.0
            self._mad = 0.0
            return

        total_weight = sum(s.weight for s in samples)
        if total_weight <= 0:
            self._weighted_mean = (
                sum(s.temperature for s in samples) / len(samples)
                if samples
                else 0.0
            )
        else:
            self._weighted_mean = (
                sum(s.temperature * s.weight for s in samples) / total_weight
            )

        deviations = [abs(s.temperature - self._weighted_mean) for s in samples]
        self._mad = _median(deviations)

        for i, s in enumerate(samples):
            if self._mad > 0:
                z = 0.6745 * deviations[i] / self._mad
            else:
                z = 0.0
            self._z_scores.append(z)
            if z > self.outlier_zscore:
                self._excluded.append(
                    {"channel": s.channel, "reason": "outlier", "z_score": z}
                )

        # Recompute weighted mean with non-excluded samples
        remaining = [
            s
            for i, s in enumerate(samples)
            if self._z_scores[i] <= self.outlier_zscore
        ]
        if remaining:
            rw = sum(s.weight for s in remaining)
            if rw > 0:
                self._weighted_mean = (
                    sum(s.temperature * s.weight for s in remaining) / rw
                )
            else:
                self._weighted_mean = sum(
                    s.temperature for s in remaining
                ) / len(remaining)

    def fuse(self):
        total = len(self._last_samples) if self._last_samples else 0
        valid = total - len(self._excluded)
        # Consistency: 1 - mad/temperature_range (guard against zero range)
        temps = (
            [s.temperature for s in self._last_samples]
            if self._last_samples
            else []
        )
        if temps:
            trange = max(temps) - min(temps)
            consistency = 1.0 - (self._mad / trange) if trange > 0 else 1.0
        else:
            consistency = 0.0
        confidence = _compute_confidence(valid, total, consistency)
        return FusionResult(
            temperature=self._weighted_mean,
            confidence=confidence,
            valid_samples=valid,
            excluded_samples=list(self._excluded),
        )

    def get_diagnostics(self):
        return {
            "excluded": list(self._excluded),
            "mad": round(self._mad, 4),
            "weighted_mean": round(self._weighted_mean, 4),
            "z_scores": [round(z, 3) for z in self._z_scores],
        }

    def reset(self):
        self._last_samples = []
        self._excluded = []
        self._mad = 0.0
        self._weighted_mean = 0.0
        self._z_scores = []


class RobustMedianStrategy(FusionStrategy):
    """Weighted median with IQR-based outlier rejection."""

    STRATEGY_CONFIG_KEYS = ["iqr_multiplier"]

    def __init__(self, strategy_config, num_channels):
        super().__init__(strategy_config, num_channels)
        self.iqr_multiplier = strategy_config.get("iqr_multiplier", 1.5)
        self._last_samples = []
        self._excluded = []
        self._q1 = 0.0
        self._q3 = 0.0
        self._iqr = 0.0
        self._weighted_median = 0.0

    def _percentile(self, sorted_vals, pct):
        """Compute the pct-th percentile (0-100) of a sorted list."""
        n = len(sorted_vals)
        if n == 0:
            return 0.0
        if n == 1:
            return sorted_vals[0]
        rank = (pct / 100.0) * (n - 1)
        lo = int(math.floor(rank))
        hi = int(math.ceil(rank))
        if lo == hi:
            return sorted_vals[lo]
        return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (
            rank - lo
        )

    def update(self, samples, eventtime):
        self._last_samples = samples
        self._excluded = []

        if not samples:
            self._q1 = self._q3 = self._iqr = 0.0
            self._weighted_median = 0.0
            return

        temps = sorted(s.temperature for s in samples)
        self._q1 = self._percentile(temps, 25)
        self._q3 = self._percentile(temps, 75)
        self._iqr = self._q3 - self._q1
        if self._iqr > 0:
            lower = self._q1 - self.iqr_multiplier * self._iqr
            upper = self._q3 + self.iqr_multiplier * self._iqr
        else:
            # IQR == 0 means most samples are identical; cannot
            # meaningfully reject outliers, so accept everything.
            lower = float("-inf")
            upper = float("inf")

        remaining = []
        for s in samples:
            if s.temperature < lower or s.temperature > upper:
                self._excluded.append(
                    {
                        "channel": s.channel,
                        "reason": "outlier",
                        "value": s.temperature,
                    }
                )
            else:
                remaining.append(s)

        if not remaining:
            remaining = list(samples)
            self._excluded = []

        # Weighted median: sort by temperature, find where cumulative weight
        # first exceeds half of total weight.
        remaining_sorted = sorted(remaining, key=lambda s: s.temperature)
        total_w = sum(s.weight for s in remaining_sorted)
        if total_w <= 0:
            # Fall back to plain median
            self._weighted_median = _median(
                [s.temperature for s in remaining_sorted]
            )
        else:
            half = total_w / 2.0
            cum = 0.0
            for s in remaining_sorted:
                cum += s.weight
                if cum >= half:
                    self._weighted_median = s.temperature
                    break
            else:
                self._weighted_median = remaining_sorted[-1].temperature

    def fuse(self):
        total = len(self._last_samples) if self._last_samples else 0
        valid = total - len(self._excluded)
        # Consistency from IQR
        temps = (
            [s.temperature for s in self._last_samples]
            if self._last_samples
            else []
        )
        if temps:
            trange = max(temps) - min(temps)
            consistency = 1.0 - (self._iqr / trange) if trange > 0 else 1.0
        else:
            consistency = 0.0
        confidence = _compute_confidence(valid, total, consistency)
        return FusionResult(
            temperature=self._weighted_median,
            confidence=confidence,
            valid_samples=valid,
            excluded_samples=list(self._excluded),
        )

    def get_diagnostics(self):
        return {
            "excluded": list(self._excluded),
            "q1": round(self._q1, 4),
            "q3": round(self._q3, 4),
            "iqr": round(self._iqr, 4),
            "weighted_median": round(self._weighted_median, 4),
        }

    def reset(self):
        self._last_samples = []
        self._excluded = []
        self._q1 = self._q3 = self._iqr = 0.0
        self._weighted_median = 0.0


class KalmanFusionStrategy(FusionStrategy):
    """Constant-state Kalman filter with sequential multi-sensor fusion."""

    STRATEGY_CONFIG_KEYS = ["q", "r_default", "init_p"]

    def __init__(self, strategy_config, num_channels):
        super().__init__(strategy_config, num_channels)
        self.q = strategy_config.get("q", 0.01)
        self.r_default = strategy_config.get("r_default", 0.1)
        self.init_p = strategy_config.get("init_p", 10.0)
        self._state = None  # x_hat
        self._covar = self.init_p  # P
        self._innovations = []
        self._gains = []
        self._last_samples = []
        self._noise_variances = []  # per-channel R, set by PrinterSensorFusion

    def set_noise_variances(self, noise_variances):
        """Called by PrinterSensorFusion to inject per-channel R values."""
        self._noise_variances = noise_variances

    def update(self, samples, eventtime):
        self._last_samples = samples
        self._innovations = []
        self._gains = []

        if not samples:
            return

        # Predict
        if self._state is None:
            # Initialize from first valid observation
            self._state = samples[0].temperature
            self._covar = self.init_p

        # Predict step: x^- = F * x (F=1), P^- = P + Q
        self._covar = self._covar + self.q

        # Sequential update for each observation
        for i, s in enumerate(samples):
            if i < len(self._noise_variances):
                r = self._noise_variances[i]
            else:
                r = self.r_default
            if r <= 0:
                r = self.r_default

            innovation = s.temperature - self._state
            gain = self._covar / (self._covar + r)
            self._state = self._state + gain * innovation
            self._covar = (1.0 - gain) * self._covar

            self._innovations.append(round(innovation, 4))
            self._gains.append(round(gain, 4))

    def fuse(self):
        total = len(self._last_samples) if self._last_samples else 0
        valid = total  # all samples passed basic filtering
        # Consistency from covariance: 1 - P/P0
        consistency = max(0.0, 1.0 - (self._covar / self.init_p))
        confidence = _compute_confidence(valid, total, consistency)
        return FusionResult(
            temperature=self._state if self._state is not None else 0.0,
            confidence=confidence,
            valid_samples=valid,
            excluded_samples=[],
        )

    def get_diagnostics(self):
        return {
            "state_estimate": round(self._state, 4)
            if self._state is not None
            else None,
            "covariance": round(self._covar, 4),
            "innovation": list(self._innovations),
            "kalman_gains": list(self._gains),
        }

    def reset(self):
        self._state = None
        self._covar = self.init_p
        self._innovations = []
        self._gains = []
        self._last_samples = []


######################################################################
# Strategy registry
######################################################################

_fusion_strategies = {}


def register_fusion_strategy(name, factory_class):
    """Register a fusion strategy class under the given name.

    This is the public API for user-supplied strategy modules.
    """
    _fusion_strategies[name] = factory_class


def _register_builtin_strategies():
    register_fusion_strategy("weighted_mean", WeightedMeanStrategy)
    register_fusion_strategy("robust_median", RobustMedianStrategy)
    register_fusion_strategy("kalman", KalmanFusionStrategy)


_register_builtin_strategies()


######################################################################
# Instance registry (module-level, for G-code command lookup)
######################################################################

_fusion_instances = {}


def register_fusion_instance(name, instance):
    _fusion_instances[name] = instance


def get_fusion_instance(name):
    return _fusion_instances.get(name)


######################################################################
# Main sensor class
######################################################################


class PrinterSensorFusion:
    """Virtual temperature sensor that fuses multiple Modbus channels."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self._config = config
        self._callback = None

        # --- Configuration ---
        self.modbus_bus_name = config.get("modbus_bus", None)
        self.modbus_channels = config.getintlist("modbus_channels", [])
        if not self.modbus_channels:
            raise config.error(
                "temperature_fusion[%s]: 'modbus_channels' is required"
                % (self.name,)
            )

        num_channels = len(self.modbus_channels)

        # Weights (default: all 1.0)
        weights_raw = config.getfloatlist("weights", None)
        if weights_raw is not None:
            self.weights = list(weights_raw)
            if len(self.weights) != num_channels:
                raise config.error(
                    "temperature_fusion[%s]: weights length %d != "
                    "modbus_channels length %d"
                    % (self.name, len(self.weights), num_channels)
                )
        else:
            self.weights = [1.0] * num_channels

        # Zones (optional, diagnostic only)
        zones_raw = config.getlist("zones", None)
        if zones_raw is not None:
            self.zones = list(zones_raw)
            if len(self.zones) != num_channels:
                raise config.error(
                    "temperature_fusion[%s]: zones length %d != "
                    "modbus_channels length %d"
                    % (self.name, len(self.zones), num_channels)
                )
        else:
            self.zones = [None] * num_channels

        # Positions (optional, reserved for future use)
        positions_raw = config.getlist("positions", None)
        if positions_raw is not None:
            if len(positions_raw) % 3 != 0:
                raise config.error(
                    "temperature_fusion[%s]: positions must be groups of "
                    "3 (x,y,z)" % (self.name,)
                )
            num_positions = len(positions_raw) // 3
            if num_positions != num_channels:
                raise config.error(
                    "temperature_fusion[%s]: positions count %d != "
                    "modbus_channels count %d"
                    % (self.name, num_positions, num_channels)
                )
            self.positions = [
                (
                    float(positions_raw[i * 3]),
                    float(positions_raw[i * 3 + 1]),
                    float(positions_raw[i * 3 + 2]),
                )
                for i in range(num_channels)
            ]
        else:
            self.positions = [None] * num_channels

        # Noise variances (for Kalman strategy)
        noise_raw = config.getfloatlist("noise_variance", None)
        if noise_raw is not None:
            self.noise_variance = list(noise_raw)
            if len(self.noise_variance) != num_channels:
                raise config.error(
                    "temperature_fusion[%s]: noise_variance length %d != "
                    "modbus_channels length %d"
                    % (self.name, len(self.noise_variance), num_channels)
                )
        else:
            self.noise_variance = []

        # Fusion strategy
        strategy_name = config.get("fusion_strategy", "weighted_mean")
        if strategy_name not in _fusion_strategies:
            raise config.error(
                "temperature_fusion[%s]: unknown strategy '%s'; "
                "available: %s"
                % (
                    self.name,
                    strategy_name,
                    ", ".join(sorted(_fusion_strategies)),
                )
            )
        strategy_class = _fusion_strategies[strategy_name]

        # Build strategy_config dict from fusion_* prefixed keys.
        # All strategy config values are floats.
        strategy_config = {}
        for key in strategy_class.STRATEGY_CONFIG_KEYS:
            val = config.getfloat("fusion_" + key, None)
            if val is not None:
                strategy_config[key] = val

        self._strategy_name = strategy_name
        self._strategy = strategy_class(strategy_config, num_channels)

        # Inject noise variances into Kalman strategy
        if (
            isinstance(self._strategy, KalmanFusionStrategy)
            and self.noise_variance
        ):
            self._strategy.set_noise_variances(self.noise_variance)

        # Timing
        self.report_time = config.getfloat(
            "report_time", REPORT_TIME, minval=MIN_REPORT_TIME
        )

        # Temperature limits
        self.min_temp = config.getfloat("min_temp", 0.0)
        self.max_temp = config.getfloat(
            "max_temp", 99999999.9, above=self.min_temp
        )
        self.maximum_deviation = config.getfloat("maximum_deviation", 999.0)

        # G-code ID for M105
        self.gcode_id = config.get("gcode_id", None)

        # --- Runtime state ---
        self.last_temp = 0.0
        self.last_confidence = 0.0
        self.last_result = None
        self.last_valid_count = 0
        self.last_raw_temps = [0.0] * num_channels
        self.last_valid_flags = [False] * num_channels
        self.last_excluded = []
        self.measured_min = 99999999.0
        self.measured_max = 0.0

        # Debug mode
        self._is_debug = (
            self.printer.get_start_args().get("debugoutput") is not None
        )

        # Bus reference (resolved lazily at connect time)
        self._bus = None

        # Register timer
        self.sample_timer = self.reactor.register_timer(self._sample_timer)

        # Register event handlers
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        # Register G-code commands
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "TEMP_FUSION_STATUS",
            self.cmd_TEMP_FUSION_STATUS,
            desc=self.cmd_TEMP_FUSION_STATUS_help,
        )
        self.gcode.register_command(
            "TEMP_FUSION_LIST_STRATEGIES",
            self.cmd_TEMP_FUSION_LIST_STRATEGIES,
            desc=self.cmd_TEMP_FUSION_LIST_STRATEGIES_help,
        )
        self.gcode.register_command(
            "TEMP_FUSION_RESET",
            self.cmd_TEMP_FUSION_RESET,
            desc=self.cmd_TEMP_FUSION_RESET_help,
        )

        # Register this instance in the module-level table
        register_fusion_instance(self.name, self)

        # Add as printer object for status queries
        self.printer.add_object("temperature_fusion " + self.name, self)

    # --- Event handlers ---

    def _handle_connect(self):
        manager = self.printer.lookup_object(MANAGER_OBJECT_NAME, None)
        if manager is None:
            raise self.printer.config_error(
                "temperature_fusion[%s]: requires [modbus_temperature] section"
                % (self.name,)
            )
        bus = manager.get_bus(self.modbus_bus_name, self._config)
        if bus is None:
            raise self.printer.config_error(
                "temperature_fusion[%s]: could not resolve modbus bus '%s'"
                % (self.name, self.modbus_bus_name)
            )
        # Validate channels
        for ch in self.modbus_channels:
            if ch < 0 or ch >= bus.channel_count:
                raise self.printer.config_error(
                    "temperature_fusion[%s]: channel %d out of range "
                    "[0, %d) on bus '%s'"
                    % (self.name, ch, bus.channel_count, bus.bus_name)
                )
        self._bus = bus

    def _handle_ready(self):
        # Delay timer start by 1s to avoid race with other sensor timers
        self.reactor.update_timer(
            self.sample_timer, self.reactor.monotonic() + 1.0
        )

    # --- Heater/sensor protocol ---

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def get_temp(self, eventtime):
        return (self.last_temp, 0.0)

    # --- Core sampling loop ---

    def _sample_timer(self, eventtime):
        if self._is_debug:
            self.last_temp = 0.0
            if self._callback is not None:
                mcu = self.printer.lookup_object("mcu")
                now = self.reactor.monotonic()
                self._callback(mcu.estimated_print_time(now), 0.0)
            return eventtime + self.report_time

        # Step 1: Batch read modbus
        try:
            serial = self._bus.open_serial()
            regs = serial.read_registers(
                self._bus.slave_id,
                self._bus.register_start,
                self._bus.channel_count,
                self._bus.func_code,
            )
        except Exception as e:
            self.printer.invoke_shutdown(
                "temperature_fusion[%s]: modbus read failed on bus '%s': %s"
                % (self.name, self._bus.bus_name, e)
            )
            return eventtime + self.report_time

        # Step 2 & 3: Decode channels + basic validity
        samples = []
        excluded = []
        temps_for_deviation = []

        for idx, ch in enumerate(self.modbus_channels):
            raw = regs[ch]

            # Track raw temps for status
            self.last_raw_temps[idx] = 0.0
            self.last_valid_flags[idx] = False

            if raw == self._bus.disconnected_raw:
                self.printer.invoke_shutdown(
                    "temperature_fusion[%s]: channel %d on bus '%s' "
                    "reports disconnected (raw=%d)"
                    % (self.name, ch, self._bus.bus_name, raw)
                )
                return eventtime + self.report_time

            # Decode
            if self._bus.signed and raw & 0x8000:
                raw = raw - 0x10000
            temp = float(raw) * self._bus.data_scale

            # NaN / Inf check
            if math.isnan(temp) or math.isinf(temp):
                self.printer.invoke_shutdown(
                    "temperature_fusion[%s]: channel %d decoded "
                    "NaN/Inf (raw=%d)" % (self.name, ch, regs[ch])
                )
                return eventtime + self.report_time

            self.last_raw_temps[idx] = temp
            self.last_valid_flags[idx] = True

            sample = SensorSample(
                channel=ch,
                raw_value=regs[ch],
                temperature=temp,
                weight=self.weights[idx],
                zone=self.zones[idx],
                position=self.positions[idx],
                is_valid=True,
                timestamp=eventtime,
            )
            samples.append(sample)
            temps_for_deviation.append(temp)

        # Step 4: Global consistency check
        if self.maximum_deviation < 999.0 and len(temps_for_deviation) >= 2:
            tmin = min(temps_for_deviation)
            tmax = max(temps_for_deviation)
            deviation = tmax - tmin
            if deviation > self.maximum_deviation:
                # Find the two channels responsible
                min_ch = self.modbus_channels[temps_for_deviation.index(tmin)]
                max_ch = self.modbus_channels[temps_for_deviation.index(tmax)]
                self.printer.invoke_shutdown(
                    "temperature_fusion[%s]: deviation %.1f > max %.1f "
                    "between channels %d (%.1f) and %d (%.1f)"
                    % (
                        self.name,
                        deviation,
                        self.maximum_deviation,
                        min_ch,
                        tmin,
                        max_ch,
                        tmax,
                    )
                )
                return eventtime + self.report_time

        # Step 5: Strategy update + fuse
        try:
            self._strategy.update(samples, eventtime)
            result = self._strategy.fuse()
        except Exception as e:
            self.printer.invoke_shutdown(
                "temperature_fusion[%s]: strategy '%s' raised: %s"
                % (self.name, self._strategy_name, e)
            )
            return eventtime + self.report_time

        # Step 6: Out-of-range check (respects temp_ignore_limits)
        if (
            result.temperature < self.min_temp
            or result.temperature > self.max_temp
        ) and not get_danger_options().temp_ignore_limits:
            self.printer.invoke_shutdown(
                "temperature_fusion[%s]: fused temp %.1f outside %.1f:%.1f"
                % (
                    self.name,
                    result.temperature,
                    self.min_temp,
                    self.max_temp,
                )
            )

        # Step 7: Cache + push
        self.last_temp = result.temperature
        self.last_confidence = result.confidence
        self.last_result = result
        self.last_valid_count = result.valid_samples
        self.last_excluded = result.excluded_samples

        if result.temperature:
            if self.measured_min > self.measured_max:
                # First valid reading — initialize both
                self.measured_min = self.measured_max = result.temperature
            else:
                self.measured_min = min(self.measured_min, result.temperature)
                self.measured_max = max(self.measured_max, result.temperature)

        if self._callback is not None:
            mcu = self.printer.lookup_object("mcu")
            self._callback(
                mcu.estimated_print_time(eventtime), result.temperature
            )

        # Step 8: Schedule next
        return eventtime + self.report_time

    # --- Status & diagnostics ---

    def get_status(self, eventtime):
        return {
            "temperature": round(self.last_temp, 2),
            "confidence": round(self.last_confidence, 3),
            "valid_samples": self.last_valid_count,
            "total_samples": len(self.modbus_channels),
            "samples": [
                {
                    "channel": ch,
                    "temperature": round(t, 2),
                    "weight": w,
                    "zone": z,
                    "valid": v,
                }
                for ch, t, w, z, v in zip(
                    self.modbus_channels,
                    self.last_raw_temps,
                    self.weights,
                    self.zones,
                    self.last_valid_flags,
                )
            ],
            "excluded": self.last_excluded,
            "strategy": self._strategy.get_diagnostics(),
            "measured_min_temp": round(self.measured_min, 2),
            "measured_max_temp": round(self.measured_max, 2),
        }

    def stats(self, eventtime):
        channel_str = " ".join(
            "ch%d=%.2f" % (ch, t)
            for ch, t in zip(self.modbus_channels, self.last_raw_temps)
        )
        return (
            False,
            "temperature_fusion: temp=%.2f conf=%.2f valid=%d/%d %s"
            % (
                self.last_temp,
                self.last_confidence,
                self.last_valid_count,
                len(self.modbus_channels),
                channel_str,
            ),
        )

    # --- G-code commands ---

    cmd_TEMP_FUSION_STATUS_help = (
        "Show current temperature fusion status for a sensor"
    )
    cmd_TEMP_FUSION_LIST_STRATEGIES_help = (
        "List all registered fusion strategies"
    )
    cmd_TEMP_FUSION_RESET_help = "Reset the internal state of a fusion strategy"

    def cmd_TEMP_FUSION_STATUS(self, gcmd):
        sensor_name = gcmd.get("SENSOR", self.name)
        instance = get_fusion_instance(sensor_name)
        if instance is None:
            gcmd.respond_info(
                "temperature_fusion: no instance named '%s'" % (sensor_name,)
            )
            return
        diag = instance._strategy.get_diagnostics()
        channel_str = " ".join(
            "ch%d=%.1f" % (ch, t)
            for ch, t in zip(instance.modbus_channels, instance.last_raw_temps)
        )
        gcmd.respond_info(
            "temperature_fusion %s:\n"
            "  fused_temp=%.1f  confidence=%.2f  valid=%d/%d\n"
            "  strategy=%s  %s\n"
            "  channel temps: %s"
            % (
                instance.name,
                instance.last_temp,
                instance.last_confidence,
                instance.last_valid_count,
                len(instance.modbus_channels),
                instance._strategy_name,
                diag,
                channel_str,
            )
        )

    def cmd_TEMP_FUSION_LIST_STRATEGIES(self, gcmd):
        lines = ["Available fusion strategies:"]
        for name in sorted(_fusion_strategies):
            cls = _fusion_strategies[name]
            builtin = (
                "(built-in)"
                if cls
                in (
                    WeightedMeanStrategy,
                    RobustMedianStrategy,
                    KalmanFusionStrategy,
                )
                else "(custom)"
            )
            lines.append("  %-20s %s" % (name, builtin))
        gcmd.respond_info("\n".join(lines))

    def cmd_TEMP_FUSION_RESET(self, gcmd):
        sensor_name = gcmd.get("SENSOR", self.name)
        instance = get_fusion_instance(sensor_name)
        if instance is None:
            gcmd.respond_info(
                "temperature_fusion: no instance named '%s'" % (sensor_name,)
            )
            return
        instance._strategy.reset()
        gcmd.respond_info(
            "temperature_fusion[%s]: strategy '%s' state reset"
            % (instance.name, instance._strategy_name)
        )


######################################################################
# Module entry point
######################################################################


def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("temperature_fusion", PrinterSensorFusion)
