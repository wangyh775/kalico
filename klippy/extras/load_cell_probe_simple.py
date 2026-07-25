# Simplified Load Cell Probe
#
# 一个轻量级的力传感器探针实现：当 ADC 读数偏离 tare 的绝对值
# 超过阈值时即视为触发。不做 SOS 滤波、不做 tap 几何分析、不做 pullback。
#
# 复用现有 MCU 端 `load_cell_probe` 命令，通过把 SOS 滤波器配成 0 段直通、
# 用固定的 grams_per_count=0.5 让 MCU 端的 grams 比较等价于 counts 比较。
#
# 依赖 [load_cell] 模块（传感器硬件初始化、collector、tare 逻辑）。
# 仅支持 MCU 直连的传感器（hx71x / ads1220 / ads131m0x），不支持 alps_serial。
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

import numpy as np

from klippy import mcu
from klippy.configfile import ConfigWrapper
from klippy.extras.homing import PrinterHoming
from klippy.extras.load_cell.load_cell import LoadCell, LoadCellSampleCollector
from klippy.extras.load_cell.sos_filter import (
    FixedPointSosFilter,
    SosFilter,
    to_fixed_32,
)
from klippy.extras.probe import PrinterProbe
from klippy.gcode import GCodeCommand

# Q2.29 定点：与现有 load_cell_probe 模块一致
Q2_INT_BITS = 2
# 虚拟 grams_per_count = 0.5，使 MCU 端的 grams 比较等价于 counts 比较：
#   trigger_grams = trigger_counts // 2
#   MCU 比较 abs(delta * 0.5) >= trigger_grams  等价于  abs(delta) >= trigger_counts


def _check_sensor_errors(results, printer):
    samples, errors = results
    if errors:
        raise printer.command_error(
            "Load cell sensor reported errors while"
            " probing: %i errors, %i overflows" % (errors[0], errors[1])
        )
    return samples


class SimpleConfigHelper:
    """读取简化版探针的配置参数，全部以 raw ADC counts 为单位。"""

    def __init__(self, config: ConfigWrapper, load_cell: LoadCell):
        self._printer = config.get_printer()
        self._load_cell = load_cell
        self._sensor = load_cell.get_sensor()
        self._rest_time = 1.0 / float(self._sensor.get_samples_per_second())
        # tare 采样时长，默认 0.1 秒
        self._tare_time = config.getfloat(
            "tare_time", default=0.1, minval=0.01, maxval=1.0
        )
        # 触发阈值：raw counts 偏离 tare 的绝对值
        self._trigger_counts = config.getint(
            "trigger_counts", minval=1, maxval=100000
        )
        # 安全限值：tare 前静止力若超此值则报错（counts 单位，0=禁用）
        self._force_safety_limit = config.getint(
            "force_safety_limit", default=50000, minval=0
        )
        # drift 安全限值：探测过程中漂移超此值则报错（counts 单位，0=禁用）
        self._drift_safety_limit = config.getint(
            "drift_safety_limit", default=10000, minval=0
        )

    def get_tare_samples(self, gcmd: GCodeCommand = None) -> int:
        tare_time = (
            gcmd.get_float(
                "TARE_TIME", self._tare_time, minval=0.01, maxval=1.0
            )
            if gcmd
            else self._tare_time
        )
        sps = self._sensor.get_samples_per_second()
        return max(2, math.ceil(tare_time * sps))

    def get_trigger_counts(self, gcmd: GCodeCommand = None) -> int:
        return (
            gcmd.get_int(
                "TRIGGER_COUNTS", self._trigger_counts, minval=1, maxval=100000
            )
            if gcmd
            else self._trigger_counts
        )

    def get_safety_limit_counts(self, gcmd: GCodeCommand = None) -> int:
        return self._force_safety_limit

    def get_drift_safety_limit(self, gcmd: GCodeCommand = None) -> int:
        return self._drift_safety_limit

    def get_rest_time(self) -> float:
        return self._rest_time

    # 下发到 MCU 的虚拟 grams 值 = trigger_counts // 2
    def get_trigger_grams(self, gcmd: GCodeCommand = None) -> int:
        return self.get_trigger_counts(gcmd) // 2

    # Q2 定点表示的 0.5
    def get_grams_per_count(self) -> int:
        return to_fixed_32(0.5, Q2_INT_BITS)

    # 基于 sensor range 中点和 force_safety_limit 计算安全带。
    # 用 sensor 中点作为 zero（简化版无校准基准），检查 tare 值是否
    # 落在合理范围内——这能检测传感器上电偏置异常或外部异常拉力。
    def get_reference_safety_range(
        self, gcmd: GCodeCommand = None
    ) -> tuple[int, int]:
        sensor_min, sensor_max = self._sensor.get_range()
        limit = self.get_safety_limit_counts(gcmd)
        if limit == 0:
            return sensor_min, sensor_max
        zero = (sensor_min + sensor_max) // 2
        safety_min = int(zero - limit)
        safety_max = int(zero + limit)
        if safety_min <= sensor_min or safety_max >= sensor_max:
            raise self._printer.command_error(
                "Load Cell Probe Error: force_safety_limit exceeds"
                " sensor range!"
            )
        return safety_min, safety_max

    def assert_force_safety_limit(self, tare_counts, gcmd: GCodeCommand = None):
        limit = self.get_safety_limit_counts(gcmd)
        if limit == 0:
            return
        safety_min, safety_max = self.get_reference_safety_range(gcmd)
        if tare_counts <= safety_min or tare_counts >= safety_max:
            raise self._printer.command_error(
                "Load Cell Probe Error: tare counts {} exceeds "
                "force_safety_limit ({} counts) before probing!".format(
                    tare_counts, limit
                )
            )

    def get_probe_drift_range(
        self, tare_counts, gcmd: GCodeCommand = None
    ) -> tuple[int, int]:
        drift_min = -(2**31)
        drift_max = 2**31 - 1
        drift_force = self.get_drift_safety_limit(gcmd)
        if drift_force > 0:
            drift_min = int(tare_counts - drift_force)
            drift_max = int(tare_counts + drift_force)
            sensor_min, sensor_max = self._sensor.get_range()
            if drift_min <= sensor_min or drift_max >= sensor_max:
                raise self._printer.command_error(
                    "Load Cell Probe Error: drift_safety_limit exceeds"
                    " sensor range!"
                )
        return drift_min, drift_max


class SimplePassthroughFilter:
    """构造一个 0 段 SOS 滤波器，MCU 端直接返回原值（见 sos_filter.c）。"""

    def __init__(self, mcu_obj: mcu.MCU, cmd_queue):
        # max_sections=1 避免 oid_alloc size=0；n_sections=0 实现直通
        fixed = FixedPointSosFilter(filter_sections=[], initial_state=[])
        self._sos_filter = SosFilter(mcu_obj, cmd_queue, fixed, max_sections=1)

    def create_filter(self):
        self._sos_filter.create_filter()

    def get_oid(self) -> int:
        return self._sos_filter.get_oid()

    def reset_filter(self):
        # 0 段滤波器无状态可重置
        pass


class SimpleMcuProbe:
    """封装 MCU 端 `load_cell_probe` 命令，与现有模块的命令接口一致。"""

    WATCHDOG_MAX = 3
    ERROR_SAFETY_RANGE = mcu.MCU_trsync.REASON_COMMS_TIMEOUT + 1
    ERROR_OVERFLOW = mcu.MCU_trsync.REASON_COMMS_TIMEOUT + 2
    ERROR_WATCHDOG = mcu.MCU_trsync.REASON_COMMS_TIMEOUT + 3

    def __init__(
        self,
        config: ConfigWrapper,
        load_cell: LoadCell,
        passthrough_filter: SimplePassthroughFilter,
        config_helper: SimpleConfigHelper,
        trigger_dispatch: mcu.TriggerDispatch,
    ):
        self._printer = config.get_printer()
        self._load_cell = load_cell
        self._filter = passthrough_filter
        self._config_helper = config_helper
        self._sensor = load_cell.get_sensor()
        self._mcu: mcu.MCU = self._sensor.get_mcu()
        self._dispatch = trigger_dispatch
        self._cmd_queue = self._dispatch.get_command_queue()
        self._oid = self._mcu.create_oid()
        self._config_commands()
        self._home_cmd = None
        self._query_cmd = None
        self._set_range_cmd = None
        self._mcu.register_config_callback(self._build_config)
        self._printer.register_event_handler("klippy:connect", self._on_connect)

    def _config_commands(self):
        self._filter.create_filter()
        self._mcu.add_config_cmd(
            "config_load_cell_probe oid=%d sos_filter_oid=%d"
            % (self._oid, self._filter.get_oid())
        )

    def _build_config(self):
        self._query_cmd = self._mcu.lookup_query_command(
            "load_cell_probe_query_state oid=%c",
            "load_cell_probe_state oid=%c is_homing_trigger=%c "
            "trigger_ticks=%u",
            oid=self._oid,
            cq=self._cmd_queue,
        )
        self._set_range_cmd = self._mcu.lookup_command(
            "load_cell_probe_set_range"
            " oid=%c safety_counts_min=%i safety_counts_max=%i tare_counts=%i"
            " trigger_grams=%u grams_per_count=%i",
            cq=self._cmd_queue,
        )
        self._home_cmd = self._mcu.lookup_command(
            "load_cell_probe_home oid=%c trsync_oid=%c trigger_reason=%c"
            " error_reason=%c clock=%u rest_ticks=%u timeout=%u",
            cq=self._cmd_queue,
        )

    def _on_connect(self):
        self._sensor.attach_load_cell_probe(self._oid)

    def get_oid(self):
        return self._oid

    def get_mcu(self):
        return self._mcu

    def get_load_cell(self) -> LoadCell:
        return self._load_cell

    def get_dispatch(self):
        return self._dispatch

    def set_endstop_range(self, tare_counts: int, gcmd: GCodeCommand = None):
        self._load_cell.tare(tare_counts)
        safety_min, safety_max = self._config_helper.get_probe_drift_range(
            tare_counts, gcmd
        )
        args = [
            self._oid,
            safety_min,
            safety_max,
            tare_counts,
            self._config_helper.get_trigger_grams(gcmd),
            self._config_helper.get_grams_per_count(),
        ]
        self._set_range_cmd.send(args)
        self._filter.reset_filter()

    def home_start(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        rest_time = self._config_helper.get_rest_time()
        rest_ticks = self._mcu.seconds_to_clock(rest_time)
        self._home_cmd.send(
            [
                self._oid,
                self._dispatch.get_oid(),
                mcu.MCU_trsync.REASON_ENDSTOP_HIT,
                self.ERROR_SAFETY_RANGE,
                clock,
                rest_ticks,
                self.WATCHDOG_MAX,
            ],
            reqclock=clock,
        )

    def clear_home(self):
        params = self._query_cmd.send([self._oid])
        trigger_ticks = self._mcu.clock32_to_clock64(params["trigger_ticks"])
        self._home_cmd.send([self._oid, 0, 0, 0, 0, 0, 0, 0])
        return self._mcu.clock_to_print_time(trigger_ticks)


class SimplePrimitives:
    """精简版探测原语：tare + probing_move，无 tap 分析、无 pullback。"""

    ERROR_MAP = {
        mcu.MCU_trsync.REASON_COMMS_TIMEOUT: "Communication timeout during "
        "homing",
        SimpleMcuProbe.ERROR_SAFETY_RANGE: "Load Cell Probe Error: force "
        "exceeded drift_safety_limit before triggering!",
        SimpleMcuProbe.ERROR_OVERFLOW: "Load Cell Probe Error: fixed point "
        "math overflow",
        SimpleMcuProbe.ERROR_WATCHDOG: "Load Cell Probe Error: timed out "
        "waiting for sensor data",
    }

    def __init__(
        self,
        config: ConfigWrapper,
        mcu_probe: SimpleMcuProbe,
        config_helper: SimpleConfigHelper,
    ):
        self._printer = config.get_printer()
        self._mcu_probe = mcu_probe
        self._config_helper = config_helper
        self._load_cell = mcu_probe.get_load_cell()
        self._dispatch = mcu_probe.get_dispatch()
        self._last_trigger_time = 0.0

    def get_mcu(self):
        return self._mcu_probe.get_mcu()

    def get_dispatch(self):
        return self._dispatch

    def _start_collector(self) -> LoadCellSampleCollector:
        toolhead = self._printer.lookup_object("toolhead")
        print_time = toolhead.get_last_move_time()
        collector = self._load_cell.get_collector()
        collector.start_collecting(min_time=print_time)
        return collector

    def tare(self, gcmd: GCodeCommand = None):
        collector = self._start_collector()
        num_samples = self._config_helper.get_tare_samples(gcmd)
        results = collector.collect_min(num_samples)
        tare_samples = _check_sensor_errors(results, self._printer)
        tare_counts = int(
            np.average(np.array(tare_samples)[:, 2].astype(float))
        )
        self._config_helper.assert_force_safety_limit(tare_counts, gcmd)
        self._mcu_probe.set_endstop_range(tare_counts, gcmd)

    # MCU_endstop 接口：home_start(print_time, sample_time, sample_count,
    # rest_time, triggered=True)。简化版忽略 sample_time/sample_count/rest_time，
    # 因为触发逻辑由 MCU 端 load_cell_probe 自己管理。
    # G28 路径：归零前先 tare（与原版 HomingMove.home_start 行为一致）。
    def home_start(
        self, print_time, sample_time, sample_count, rest_time, triggered=True
    ):
        self.tare()
        trigger_completion = self._dispatch.start(print_time)
        self._mcu_probe.home_start(print_time)
        return trigger_completion

    def home_wait(self, home_end_time):
        self._dispatch.wait_end(home_end_time)
        res = self._dispatch.stop()
        self._last_trigger_time = self._mcu_probe.clear_home()
        if res >= mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
            error = "Load Cell Probe Error: unknown reason code %i" % (res,)
            if res in self.ERROR_MAP:
                error = self.ERROR_MAP[res]
            raise self._printer.command_error(error)
        if res != mcu.MCU_trsync.REASON_ENDSTOP_HIT:
            return 0.0
        return self._last_trigger_time

    def add_stepper(self, stepper):
        self._dispatch.add_stepper(stepper)

    def get_steppers(self):
        return self.get_dispatch().get_steppers()

    def query_endstop(self, print_time):
        return False

    def probing_move(
        self, mcu_probe, pos, speed, gcmd: GCodeCommand
    ) -> list[float]:
        # 简化版不做 tap 分析，probing_move 阶段不需要持续采集样本。
        # tare 已经在内部启动并停止了它自己的 collector，这里不再启动。
        self.tare(gcmd)
        printer_homing: PrinterHoming = self._printer.lookup_object("homing")
        return printer_homing.probing_move(mcu_probe, pos, speed)

    def get_status(self, eventtime):
        return {
            "last_trigger_time": self._last_trigger_time,
            "name": self._load_cell.name,
        }


class SimpleEndstopWrapper:
    """实现 ProbeEndstopWrapper 接口，被 PrinterProbe 使用。"""

    def __init__(
        self,
        config: ConfigWrapper,
        primitives: SimplePrimitives,
    ):
        self._printer = config.get_printer()
        self._z_offset = config.getfloat("z_offset", 0.0)
        self._primitives = primitives
        # MCU_identify 后注册 Z 轴 stepper
        self._printer.register_event_handler(
            "klippy:mcu_identify", self._handle_mcu_identify
        )
        # 委托 primitives 实现 MCU_endstop 接口
        self.get_mcu = primitives.get_mcu
        self.add_stepper = primitives.add_stepper
        self.get_steppers = primitives.get_steppers
        self.home_wait = primitives.home_wait
        self.home_start = primitives.home_start
        self.query_endstop = primitives.query_endstop
        # probe 激活/去活 gcode 支持
        from klippy.extras.load_cell.load_cell_probe import (
            ProbeActivationHelper,
        )

        self._activation_helper = ProbeActivationHelper(config)
        self.probe_prepare = self._activation_helper.probe_prepare
        self.probe_finish = self._activation_helper.probe_finish
        self.multi_probe_begin = self._activation_helper.multi_probe_begin
        self.multi_probe_end = self._activation_helper.multi_probe_end

    def _handle_mcu_identify(self):
        kin = self._printer.lookup_object("toolhead").get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis("z"):
                self.add_stepper(stepper)

    # PrinterProbe.probing_move 调用 self.mcu_probe.probing_move(pos, speed, gcmd)
    # 返回 epos 或 (epos, is_good) 元组。简化版不做 tap 校验，is_good 恒为 True。
    def probing_move(self, pos, speed, gcmd: GCodeCommand):
        epos = self._primitives.probing_move(self, pos, speed, gcmd)
        return epos, True

    def get_position_endstop(self):
        return self._z_offset

    def get_status(self, eventtime):
        return self._primitives.get_status(eventtime)


def load_config(config: ConfigWrapper):
    printer = config.get_printer()
    # 依赖 [load_cell] 已加载的 LoadCell 对象
    load_cell: LoadCell = printer.load_object(config, "load_cell")
    config_helper = SimpleConfigHelper(config, load_cell)
    mcu_obj = load_cell.get_sensor().get_mcu()
    trigger_dispatch = mcu.TriggerDispatch(mcu_obj)
    passthrough_filter = SimplePassthroughFilter(
        mcu_obj, trigger_dispatch.get_command_queue()
    )
    mcu_probe = SimpleMcuProbe(
        config, load_cell, passthrough_filter, config_helper, trigger_dispatch
    )
    primitives = SimplePrimitives(config, mcu_probe, config_helper)
    wrapper = SimpleEndstopWrapper(config, primitives)
    printer_probe = PrinterProbe(config, wrapper)
    printer.add_object("probe", printer_probe)
    return printer_probe
