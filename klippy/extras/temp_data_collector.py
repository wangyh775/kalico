# =============================================================================
# 温度数据采集模块 - 用于热参数辨识
# Temperature Data Collector for Thermal Parameter Identification
# =============================================================================
#
# Copyright (C) 2024
#
# 本文件依据 GNU GPLv3 许可证分发。
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# =============================================================================
# 模块概述 (Module Overview)
# =============================================================================
#
# 本模块用于3D打印机加热系统的热参数辨识，提供两种实验方法：
#
# 1. 稳态阶梯响应实验 (Steady-State Staircase Characterization)
#    - 目的：分离并量化系统的静态热损耗参数
#    - 辨识参数：线性散热系数 K_lin（对流+传导）、辐射系数 K_rad
#    - 原理：在热平衡状态下，输入功率等于散热功率
#
# 2. 高频动态激励实验 (PRBS Dynamic Excitation)
#    - 目的：辨识系统的动态惯性参数
#    - 辨识参数：热容 C、接触热阻 R
#    - 原理：通过伪随机二进制序列激发系统的频率响应特性
#
# =============================================================================
# 热学模型 (Thermal Model)
# =============================================================================
#
# 热损耗模型：
#   P_loss = K_lin × (T - T_amb) + K_rad × (T⁴ - T_amb⁴)
#
# 多节点热网络模型：
#   [加热芯] --R_hb--> [加热块] --R_bs--> [传感器]
#       |                  |                  |
#     C_h                C_b                C_s
#
# 其中：
#   - K_lin: 线性散热系数 (W/K)，包含对流和喉管传导
#   - K_rad: 辐射系数 (W/K⁴)，等于 εσA
#   - C_h, C_b, C_s: 各节点的热容 (J/K)
#   - R_hb, R_bs: 节点间的接触热阻 (K/W)
#
# =============================================================================

import csv
import logging
import os
import random
import threading
import time

# =============================================================================
# 全局常量定义 (Global Constants)
# =============================================================================

# PWM引脚最小时间间隔（秒），确保MCU有足够时间处理命令
PIN_MIN_TIME = 0.100

# 默认环境温度（摄氏度），用于热损耗计算
AMBIENT_TEMP = 25.0

# 稳态确认后开始挤出前的等待时间（秒）
STABILITY_BEFORE_EXTRUDE = 15.0


# =============================================================================
# 核心类：温度数据采集器
# Core Class: Temperature Data Collector
# =============================================================================


class TemperatureDataCollector:
    """
    温度数据采集器类

    该类实现温度数据的实时采集、存储和实验管理功能。支持：
    - 手动数据采集模式
    - 稳态阶梯响应自动实验
    - PRBS动态激励自动实验
    - CSV格式数据存储

    属性：
        printer: 打印机主对象引用
        reactor: 事件反应器，用于定时器管理
        gcode: G-code命令处理器
        data_dir: 数据存储目录路径
        default_sample_rate: 默认采样率 (Hz)
        max_heater_power: 加热器最大功率 (W)
        is_collecting: 是否正在采集数据
        data_buffer: 数据缓存列表
        heater: 当前监控的加热器对象
    """

    def __init__(self, config):
        """
        初始化温度数据采集器

        参数：
            config: 配置对象，包含printer.cfg中的配置参数

        配置参数说明：
            data_dir: 数据存储目录，默认 ~/printer_data/temp_data
            sample_rate: 默认采样率 (Hz)，默认 10.0
            max_heater_power: 加热器最大功率 (W)，默认 60.0
        """
        # 获取打印机核心对象
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")

        # 从配置文件读取参数
        # data_dir: 数据存储目录，支持 ~ 展开为用户主目录
        self.data_dir = os.path.expanduser(
            config.get("data_dir", "~/printer_data/temp_data")
        )
        # sample_rate: 采样频率，单位 Hz，影响数据采集的时间分辨率
        self.default_sample_rate = config.getfloat(
            "sample_rate", 10.0, above=0.0
        )
        # max_heater_power: 加热器额定功率，用于将PWM占空比转换为实际功率(W)
        self.max_heater_power = config.getfloat(
            "max_heater_power", 60.0, above=0.0
        )

        # 采集状态变量
        self.is_collecting = False  # 是否正在采集
        self.collection_lock = threading.Lock()  # 线程锁，保护数据缓冲区
        self.current_experiment = None  # 当前实验名称
        self.current_phase = "heating"  # 当前阶段：heating/cooling
        self.data_buffer = []  # 数据缓存列表
        self.sample_timer = None  # 采样定时器句柄
        self.start_time = 0.0  # 实验开始时间

        # 加热器相关对象
        self.heater = None  # 加热器对象引用
        self.heater_name = None  # 加热器名称
        self.pwm_pin = None  # PWM引脚对象（保留扩展用）
        self.sensor = None  # 温度传感器对象（保留扩展用）

        # 开环控制相关变量
        self._open_loop_control = None  # 开环控制器实例
        self._original_control = None  # 保存的原控制器（用于恢复）
        self._control_switched = False  # 控制器是否已切换

        # 注册事件处理器
        # klippy:ready - 打印机初始化完成时触发
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        # klippy:shutdown - 打印机关闭时触发，用于清理资源
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )

        # 注册G-code命令
        # 每个命令对应一个处理方法，desc属性提供命令帮助信息
        self.gcode.register_command(
            "TEMP_DATA_COLLECT",
            self.cmd_TEMP_DATA_COLLECT,
            desc=self.cmd_TEMP_DATA_COLLECT_help,
        )
        self.gcode.register_command(
            "TEMP_DATA_STOP",
            self.cmd_TEMP_DATA_STOP,
            desc=self.cmd_TEMP_DATA_STOP_help,
        )
        self.gcode.register_command(
            "STEADY_STATE_CALIBRATE",
            self.cmd_STEADY_STATE_CALIBRATE,
            desc=self.cmd_STEADY_STATE_CALIBRATE_help,
        )
        self.gcode.register_command(
            "PRBS_CALIBRATE",
            self.cmd_PRBS_CALIBRATE,
            desc=self.cmd_PRBS_CALIBRATE_help,
        )
        self.gcode.register_command(
            "TEMP_DATA_STATUS",
            self.cmd_TEMP_DATA_STATUS,
            desc=self.cmd_TEMP_DATA_STATUS_help,
        )
        self.gcode.register_command(
            "THERMAL_ID_CALIBRATE",
            self.cmd_THERMAL_ID_CALIBRATE,
            desc=self.cmd_THERMAL_ID_CALIBRATE_help,
        )

        # 创建数据存储目录
        # 如果目录不存在则自动创建
        if not os.path.exists(self.data_dir):
            try:
                os.makedirs(self.data_dir)
            except OSError as e:
                logging.warning("无法创建数据目录: %s", e)

    # =========================================================================
    # 事件处理方法 (Event Handlers)
    # =========================================================================

    def _handle_ready(self):
        """
        打印机就绪事件处理器

        在打印机初始化完成后调用，可用于执行需要其他模块已加载的初始化操作。
        当前为空实现，保留用于未来扩展。
        """
        pass

    def _handle_shutdown(self):
        """
        打打印机关闭事件处理器

        在打印机关闭时调用，确保停止正在进行的采集任务，
        防止数据丢失或加热器异常。
        """
        self._stop_collection()

    # =========================================================================
    # 辅助方法 (Helper Methods)
    # =========================================================================

    def _get_heater(self, heater_name):
        """
        获取加热器对象

        参数：
            heater_name: 加热器名称，如 "extruder" 或 "heater_bed"

        返回：
            加热器对象引用

        异常：
            如果加热器不存在，将抛出配置错误异常
        """
        pheaters = self.printer.lookup_object("heaters")
        return pheaters.lookup_heater(heater_name)

    def _get_sensor_temp(self):
        """
        获取当前传感器温度

        返回：
            当前温度值（摄氏度），如果加热器未初始化则返回 0.0
        """
        if self.heater is None:
            return 0.0
        # get_temp() 返回 (当前温度, 目标温度) 元组
        return self.heater.get_temp(self.reactor.monotonic())[0]

    def _get_pwm_value(self):
        """
        获取当前PWM占空比

        返回：
            当前PWM占空比 (0.0-1.0)，如果加热器未初始化则返回 0.0
        """
        if self.heater is None:
            return 0.0
        return getattr(self.heater, "last_pwm_value", 0.0)

    def _set_heater_power(self, power, target):
        """
        设置加热器功率（开环控制）

        该方法通过开环控制器设置PWM占空比，用于开环功率控制。
        必须先调用 _switch_to_open_loop_control() 切换到开环控制器。

        参数：
            power: PWM占空比 (0.0-1.0)，表示最大功率的百分比
            target: 目标温度 (°C)，用于控制器状态管理

        注意：
            此方法用于开环控制，不经过PID调节，
            使用时需确保有适当的安全保护措施。
        """
        if self.heater is None or self._open_loop_control is None:
            return
        self._open_loop_control.set_output(power, target)

    def _switch_to_open_loop_control(self):
        """
        切换到开环控制模式

        使用OpenLoopControl替换当前控制器，实现开环功率控制。
        OpenLoopControl会在temperature_update回调中维持设置的PWM值，
        防止被原控制器覆盖。

        必须在开环控制前调用此方法，结束后调用 _restore_original_control() 恢复。
        """
        if self.heater is None:
            return

        class OpenLoopControl:
            """
            开环控制器 - 用于校准和参数辨识

            该控制器在temperature_update回调中维持设置的PWM值，
            而不是根据温度误差计算输出，从而实现开环控制。
            """

            def __init__(self, heater):
                self.value = 0.0
                self.target = None
                self.heater = heater
                self.log = []
                self.logging = False

            def temperature_update(self, read_time, temp, target_temp):
                if self.logging:
                    self.log.append((read_time, temp))
                self.heater.set_pwm(read_time, self.value)

            def check_busy(self, eventtime, smoothed_temp, target_temp):
                return self.value != 0.0 or self.target != 0

            def set_output(self, value, target):
                self.value = value
                self.target = target
                self.heater.set_temp(target)

            def get_profile(self):
                return {"name": "open_loop"}

            def get_type(self):
                return "open_loop"

        self._open_loop_control = OpenLoopControl(self.heater)
        self._original_control = self.heater.set_control(
            self._open_loop_control
        )
        self._control_switched = True

    def _restore_original_control(self):
        """
        恢复原控制器

        将之前保存的控制器恢复，从开环控制切换回原控制模式。
        """
        if self.heater is None or self._original_control is None:
            return

        self._open_loop_control.set_output(0.0, 0.0)
        self.heater.set_control(self._original_control)
        self._control_switched = False
        self._open_loop_control = None

    # =========================================================================
    # 数据采集核心方法 (Data Collection Core Methods)
    # =========================================================================

    def _sample_callback(self, eventtime):
        """
        采样回调函数 - 采集单次数据样本

        该方法在每次采样时刻被调用，采集温度、PWM、目标温度等数据，
        并将样本存入数据缓冲区。

        参数：
            eventtime: 事件时间戳（reactor单调时间）

        采集的数据字段：
            - time: 采样时间戳
            - temperature: 当前温度 (°C)
            - pwm: PWM占空比 (0.0-1.0)
            - target: 目标温度 (°C)
            - power_watts: 计算功率 (W) = pwm × max_heater_power
            - experiment: 实验名称（可选）
            - phase: 当前阶段（heating/cooling）
        """
        if not self.is_collecting:
            return

        temp = self._get_sensor_temp()
        pwm = self._get_pwm_value()
        target = (
            getattr(self.heater, "target_temp", 0.0) if self.heater else 0.0
        )

        sample = {
            "time": eventtime,
            "temperature": temp,
            "pwm": pwm,
            "target": target,
            "power_watts": pwm * self.max_heater_power,
            "phase": self.current_phase,
        }

        if self.current_experiment is not None:
            sample["experiment"] = self.current_experiment

        with self.collection_lock:
            self.data_buffer.append(sample)

    def _start_collection(self, experiment_name=None):
        """
        启动数据采集

        初始化采集状态，清空数据缓冲区，启动采样定时器。

        参数：
            experiment_name: 实验名称，用于标识数据来源

        返回：
            bool: True 表示成功启动，False 表示已有采集任务在进行

        注意：
            该方法使用线程锁确保状态变更的原子性。
        """
        with self.collection_lock:
            if self.is_collecting:
                return False

            self.is_collecting = True
            self.current_experiment = experiment_name
            self.current_phase = "heating"
            self.data_buffer = []
            self.start_time = self.reactor.monotonic()

        self.sample_timer = self.reactor.register_timer(
            self._sample_callback_timer, self.reactor.NOW
        )
        return True

    def _sample_callback_timer(self, eventtime):
        """
        采样定时器回调函数

        该方法由reactor定时器调用，执行采样并返回下次调用时间。

        参数：
            eventtime: 当前事件时间

        返回：
            float: 下次调用的时间戳，实现周期性采样
        """
        self._sample_callback(eventtime)
        # 计算下次采样时间 = 当前时间 + 采样周期
        return eventtime + (1.0 / self.default_sample_rate)

    def _stop_collection(self):
        """
        停止数据采集

        停止采样定时器，设置采集状态为False。
        注意：该方法不保存数据，需调用 _save_data_to_csv() 单独保存。
        """
        self.is_collecting = False
        if self.sample_timer is not None:
            self.reactor.unregister_timer(self.sample_timer)
            self.sample_timer = None

    def _save_data_to_csv(self, filename):
        """
        将采集数据保存到CSV文件

        参数：
            filename: 输出文件名（不含路径）

        返回：
            bool: True 表示保存成功，False 表示保存失败或无数据

        输出格式：
            CSV文件，包含以下列：
            - time: 相对时间（秒，从实验开始计时）
            - temperature: 温度 (°C)
            - pwm: PWM占空比
            - target: 目标温度 (°C)
            - power_watts: 功率 (W)
            - experiment: 实验名称
            - phase: 当前阶段（heating/cooling）
        """
        if not self.data_buffer:
            return False

        filepath = os.path.join(self.data_dir, filename)
        try:
            with open(filepath, "w", newline="") as csvfile:
                fieldnames = [
                    "time",
                    "temperature",
                    "pwm",
                    "target",
                    "power_watts",
                    "experiment",
                    "phase",
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

                start_time = self.data_buffer[0]["time"]
                for sample in self.data_buffer:
                    row = {
                        "time": sample["time"] - start_time,
                        "temperature": sample["temperature"],
                        "pwm": sample["pwm"],
                        "target": sample["target"],
                        "power_watts": sample["power_watts"],
                        "experiment": sample.get("experiment", ""),
                        "phase": sample.get("phase", "heating"),
                    }
                    writer.writerow(row)
            return True
        except Exception as e:
            logging.error("保存CSV数据失败: %s", e)
            return False

    # =========================================================================
    # G-code 命令处理方法 (G-code Command Handlers)
    # =========================================================================

    # 命令帮助文本
    cmd_TEMP_DATA_COLLECT_help = "启动温度数据采集"

    def cmd_TEMP_DATA_COLLECT(self, gcmd):
        """
        处理 TEMP_DATA_COLLECT 命令 - 启动手动数据采集

        用法：
            TEMP_DATA_COLLECT [HEATER=<加热器名>] [SAMPLE_RATE=<采样率>] [EXPERIMENT=<实验名>]

        参数：
            HEATER: 加热器名称，默认 "extruder"
            SAMPLE_RATE: 采样频率 (Hz)，默认使用配置值
            EXPERIMENT: 实验名称，用于数据标识

        示例：
            TEMP_DATA_COLLECT HEATER=extruder SAMPLE_RATE=20 EXPERIMENT=manual_test
        """
        # 解析命令参数
        heater_name = gcmd.get("HEATER", "extruder")
        sample_rate = gcmd.get_float(
            "SAMPLE_RATE", self.default_sample_rate, above=0.0
        )
        experiment_name = gcmd.get("EXPERIMENT", "manual_collection")

        # 获取加热器对象
        try:
            self.heater = self._get_heater(heater_name)
            self.heater_name = heater_name
        except Exception as e:
            raise gcmd.error(f"未找到加热器 '{heater_name}': {e}")

        # 更新采样率
        self.default_sample_rate = sample_rate

        # 启动采集
        if self._start_collection(experiment_name):
            gcmd.respond_info(
                f"已启动 '{heater_name}' 的温度数据采集，"
                f"采样率 {sample_rate} Hz，实验名称: {experiment_name}"
            )
        else:
            gcmd.respond_info("数据采集已在进行中。")

    cmd_TEMP_DATA_STOP_help = "停止温度数据采集并保存到文件"

    def cmd_TEMP_DATA_STOP(self, gcmd):
        """
        处理 TEMP_DATA_STOP 命令 - 停止采集并保存数据

        用法：
            TEMP_DATA_STOP [FILENAME=<文件名>]

        参数：
            FILENAME: 输出文件名，默认自动生成（temp_data_<时间戳>.csv）

        示例：
            TEMP_DATA_STOP FILENAME=my_experiment.csv
        """
        filename = gcmd.get(
            "FILENAME", f"temp_data_{time.strftime('%Y%m%d%H%M')}.csv"
        )

        # 停止采集
        self._stop_collection()

        # 保存数据
        if self._save_data_to_csv(filename):
            filepath = os.path.join(self.data_dir, filename)
            gcmd.respond_info(
                f"数据采集已停止。已保存 {len(self.data_buffer)} 个样本到 {filepath}"
            )
        else:
            gcmd.respond_info("无数据可保存或保存失败。")

        # 清理状态
        with self.collection_lock:
            self.data_buffer = []
            self.current_experiment = None

    cmd_TEMP_DATA_STATUS_help = "获取温度数据采集状态"

    def cmd_TEMP_DATA_STATUS(self, gcmd):
        """
        处理 TEMP_DATA_STATUS 命令 - 显示当前采集状态

        用法：
            TEMP_DATA_STATUS

        输出信息：
            - 当前状态（采集中/空闲）
            - 当前实验名称
            - 已采集样本数
            - 采样率
            - 数据存储目录
        """
        status = "采集中" if self.is_collecting else "空闲"
        experiment = self.current_experiment or "无"
        samples = len(self.data_buffer)

        gcmd.respond_info(
            f"温度数据采集器状态:\n"
            f"  状态: {status}\n"
            f"  当前实验: {experiment}\n"
            f"  已采集样本: {samples}\n"
            f"  采样率: {self.default_sample_rate} Hz\n"
            f"  数据目录: {self.data_dir}"
        )

    cmd_STEADY_STATE_CALIBRATE_help = "运行稳态阶梯响应校准实验"

    def cmd_STEADY_STATE_CALIBRATE(self, gcmd):
        """
        处理 STEADY_STATE_CALIBRATE 命令 - 稳态阶梯响应实验

        该实验通过测量不同温度点下的平衡功率，辨识热损耗参数。

        原理说明：
            在热平衡状态下，dT/dt = 0，输入功率等于散热功率：
            P_in = P_loss(T) = K_lin × (T - T_amb) + K_rad × (T⁴ - T_amb⁴)

        用法：
            STEADY_STATE_CALIBRATE [HEATER=<加热器>] [TEMP_POINTS=<温度点>]
                                   [TOLERANCE=<容差>] [DURATION=<持续时间>]
                                   [FILENAME=<文件名>]
                                   [COOLING_ENABLED=<0|1>]
                                   [COOLING_MODE=<duration|target_temp>]
                                   [COOLING_DURATION=<秒>]
                                   [COOLING_TARGET_TEMP=<温度>]
                                   [F=<挤出速度>] [EXTRUDE_DURATION=<挤出时长>]

        参数：
            HEATER: 加热器名称，默认 "extruder"
            TEMP_POINTS: 温度设定点序列，逗号分隔，默认 "50,100,150,200,250,300"
            TOLERANCE: 稳定性判断容差 (°C)，默认 0.1
            DURATION: 稳定持续时间要求 (秒)，默认 180
            FILENAME: 输出文件名
            COOLING_ENABLED: 是否启用冷却数据采集，默认 1（启用）
            COOLING_MODE: 冷却模式，"duration"（固定时长）或 "target_temp"（目标温度），默认 "target_temp"
            COOLING_DURATION: 冷却持续时间（秒），当 COOLING_MODE=duration 时有效，默认 600
            COOLING_TARGET_TEMP: 冷却目标温度 (°C)，当 COOLING_MODE=target_temp 时有效，默认 30
            F: 挤出速度 (mm/min)，默认 0（不挤出），设置大于0则启用挤出
            EXTRUDE_DURATION: 挤出持续时间 (秒)，默认 0（挤出到稳态采集结束）

        示例：
            STEADY_STATE_CALIBRATE HEATER=extruder TEMP_POINTS=50,100,150,200,250,300
            STEADY_STATE_CALIBRATE HEATER=extruder COOLING_MODE=duration COOLING_DURATION=300
            STEADY_STATE_CALIBRATE HEATER=extruder COOLING_ENABLED=0
            STEADY_STATE_CALIBRATE HEATER=extruder F=300 EXTRUDE_DURATION=60
        """
        heater_name = gcmd.get("HEATER", "extruder")
        temp_points = gcmd.get("TEMP_POINTS", "50,100,150,200,250,300")
        stability_tolerance = gcmd.get_float("TOLERANCE", 1, above=0.0)
        stability_duration = gcmd.get_float("DURATION", 180.0, above=0.0)
        filename = gcmd.get(
            "FILENAME", f"steady_state_{time.strftime('%Y%m%d%H%M')}.csv"
        )

        cooling_enabled = gcmd.get_int("COOLING_ENABLED", 1, minval=0, maxval=1)
        cooling_mode = gcmd.get("COOLING_MODE", "target_temp")
        cooling_duration = gcmd.get_float("COOLING_DURATION", 600.0, above=0.0)
        cooling_target_temp = gcmd.get_float(
            "COOLING_TARGET_TEMP", 30.0, above=0.0
        )

        extrude_speed = gcmd.get_float("F", 0.0, minval=0.0)
        extrude_duration = gcmd.get_float("EXTRUDE_DURATION", 0.0, minval=0.0)

        try:
            self.heater = self._get_heater(heater_name)
            self.heater_name = heater_name
        except Exception as e:
            raise gcmd.error(f"未找到加热器 '{heater_name}': {e}")

        try:
            temps = [float(t.strip()) for t in temp_points.split(",")]
        except ValueError:
            raise gcmd.error("TEMP_POINTS 格式无效。请使用逗号分隔的数值。")

        if cooling_mode not in ("duration", "target_temp"):
            raise gcmd.error("COOLING_MODE 必须是 'duration' 或 'target_temp'")

        gcmd.respond_info(
            f"启动 '{heater_name}' 的稳态阶梯响应校准\n"
            f"温度设定点: {temps}\n"
            f"稳定性容差: {stability_tolerance}°C\n"
            f"稳定持续时间: {stability_duration}s\n"
            f"冷却数据采集: {'启用' if cooling_enabled else '禁用'}\n"
            f"挤出: {'启用' if extrude_speed > 0 else '禁用'}"
            + (
                f" (速度: {extrude_speed:.0f}mm/min, "
                f"时长: {'至采集结束' if extrude_duration <= 0 else f'{extrude_duration:.0f}s'})"
                if extrude_speed > 0
                else ""
            )
        )

        if cooling_enabled:
            if cooling_mode == "duration":
                gcmd.respond_info(f"冷却模式: 固定时长 ({cooling_duration}s)")
            else:
                gcmd.respond_info(
                    f"冷却模式: 目标温度 ({cooling_target_temp}°C)"
                )

        self._start_collection("steady_state_staircase")

        results = []
        cooling_data = None
        try:
            for target_temp in temps:
                gcmd.respond_info(f"加热至 {target_temp}°C...")

                result = self._run_steady_state_measurement(
                    target_temp,
                    stability_duration,
                    gcmd,
                    tolerance=stability_tolerance,
                    extrude_speed=extrude_speed,
                    extrude_duration=extrude_duration,
                )

                results.append(
                    {
                        "target_temp": result["target_temp"],
                        "avg_temp": result["avg_temp"],
                        "avg_power": result["avg_power"],
                    }
                )
                gcmd.respond_info(
                    f"温度 {target_temp}°C 稳态结果: "
                    f"平均温度={result['avg_temp']:.2f}°C, 平均功率={result['avg_power']:.2f}W"
                )

            if cooling_enabled:
                gcmd.respond_info("开始冷却数据采集阶段...")
                cooling_data = self._run_cooling_phase(
                    cooling_mode, cooling_duration, cooling_target_temp, gcmd
                )

        except Exception as e:
            gcmd.respond_raw(f"!! 校准中断: {e}")
        finally:
            self._set_heater_power(0.0, 0.0)
            self._restore_original_control()
            self._stop_collection()
            self._save_data_to_csv(filename)

        if results:
            self._save_steady_state_results(results, filename)

        if cooling_data:
            self._save_cooling_results(cooling_data, filename)

        summary = f"稳态阶梯响应校准完成。数据已保存到 {filename}"
        if cooling_enabled and cooling_data:
            summary += f"\n冷却阶段: 从 {cooling_data['start_temp']:.1f}°C 降至 {cooling_data['end_temp']:.1f}°C，耗时 {cooling_data['duration']:.1f}s"
        gcmd.respond_info(summary)

    def _run_steady_state_measurement(
        self,
        target_temp,
        stable_duration,
        gcmd,
        tolerance=1.0,
        extrude_speed=0.0,
        extrude_duration=0.0,
    ):
        """
        执行单温度点稳态测量

        使用PID闭环控制达到目标温度，采集稳态数据。
        支持两种模式：
        1. 稳定性监测模式：监测温度波动，达到稳定后采集数据
        2. 固定时长模式：达到目标温度后直接采集固定时长数据

        参数：
            target_temp: 目标温度 (°C)
            stable_duration: 稳态数据采集时长 (秒)
            gcmd: G-code命令对象
            tolerance: 温度波动容差 (°C)，默认值为 1.0
                      - None: 固定时长模式，不进行稳定性监测
                      - 数值: 稳定性监测模式，温度波动 ≤ tolerance 时认为稳定
            extrude_speed: 挤出速度 (mm/min)，0表示不挤出
            extrude_duration: 挤出持续时间 (秒)，0表示挤出到稳态采集结束

        返回：
            dict: 稳态测量结果，包含：
                - target_temp: 目标温度 (°C)
                - avg_temp: 平均温度 (°C)
                - avg_power: 平均功率 (W)
                - stable_duration: 稳态采集时长 (秒)
                - samples_count: 采集样本数
        """
        pheaters = self.printer.lookup_object("heaters")

        start_temp = self._get_sensor_temp()
        gcmd.respond_info(
            f"设置目标温度: {target_temp}°C (当前: {start_temp:.1f}°C)"
        )

        pheaters.set_temperature(self.heater, target_temp, wait=True)

        if tolerance is not None:
            gcmd.respond_info(
                f"等待 {target_temp}°C 温度稳定 (容差: {tolerance}°C)..."
            )

            stability_window = 5
            stable_start = None
            stable_confirmed_time = None
            check_interval = 1.0
            last_temps = []

            while True:
                current_temp = self._get_sensor_temp()

                last_temps.append(current_temp)
                if len(last_temps) > int(stability_window / check_interval):
                    last_temps.pop(0)

                if len(last_temps) >= 3:
                    temp_range = max(last_temps) - min(last_temps)
                    if temp_range <= tolerance:
                        if stable_start is None:
                            stable_start = self.reactor.monotonic()
                            stable_confirmed_time = self.reactor.monotonic()
                        elif (
                            self.reactor.monotonic() - stable_start
                        ) >= stability_window:
                            gcmd.respond_info(
                                f"温度 {target_temp}°C 已达到稳定状态"
                            )
                            break
                    else:
                        stable_start = None
                        stable_confirmed_time = None

                self.reactor.pause(self.reactor.monotonic() + check_interval)
        else:
            gcmd.respond_info(
                f"已达到目标温度，开始稳态数据采集 ({stable_duration}秒)"
            )
            stable_confirmed_time = None

        samples_at_start = len(self.data_buffer)
        measurement_start = self.reactor.monotonic()
        check_interval = 1.0
        last_report_time = measurement_start
        report_interval = 15.0
        extrusion_started = False

        while True:
            current_time = self.reactor.monotonic()
            elapsed = current_time - measurement_start

            if elapsed >= stable_duration:
                break

            time_since_stable = (
                current_time - stable_confirmed_time
                if stable_confirmed_time is not None
                else 0.0
            )
            if (
                not extrusion_started
                and extrude_speed > 0
                and time_since_stable >= STABILITY_BEFORE_EXTRUDE
            ):
                extrude_time = (
                    stable_duration
                    - STABILITY_BEFORE_EXTRUDE
                    - (measurement_start - stable_confirmed_time)
                    if extrude_duration <= 0
                    else min(
                        extrude_duration,
                        stable_duration
                        - STABILITY_BEFORE_EXTRUDE
                        - (measurement_start - stable_confirmed_time),
                    )
                )
                distance = extrude_speed * extrude_time / 60.0
                try:
                    gcode_move = self.printer.lookup_object("gcode_move")
                    was_absolute = gcode_move.absolute_extrude
                    self.gcode.run_script_from_command("M83")
                    self.gcode.run_script_from_command(
                        f"G1 E{distance:.1f} F{extrude_speed:.0f}"
                    )
                    if was_absolute:
                        self.gcode.run_script_from_command("M82")
                    extrusion_started = True
                    gcmd.respond_info(
                        f"开始挤出: 速度{extrude_speed:.0f}mm/min, "
                        f"距离{distance:.1f}mm, 时长{extrude_time:.0f}s"
                    )
                except Exception as e:
                    gcmd.respond_info(f"挤出启动失败: {e}")

            if current_time - last_report_time >= report_interval:
                current_temp = self._get_sensor_temp()
                remaining = stable_duration - elapsed
                gcmd.respond_info(
                    f"  稳态采集中... {elapsed:.0f}s/{stable_duration:.0f}s, "
                    f"当前温度: {current_temp:.1f}°C"
                )
                last_report_time = current_time

            self.reactor.pause(current_time + check_interval)

        samples_collected = len(self.data_buffer) - samples_at_start
        avg_temp = self._calculate_average_temp(stable_duration)
        avg_power = self._calculate_average_power(stable_duration)

        results = {
            "target_temp": target_temp,
            "avg_temp": avg_temp,
            "avg_power": avg_power,
            "stable_duration": stable_duration,
            "samples_count": samples_collected,
        }

        gcmd.respond_info(
            f"稳态测量完成:\n"
            f"  目标温度: {target_temp}°C\n"
            f"  平均温度: {avg_temp:.2f}°C\n"
            f"  平均功率: {avg_power:.2f}W (散热功率)\n"
            f"  采集样本: {samples_collected}"
        )

        return results

    def _calculate_average_power(self, duration):
        """
        计算指定时间窗口内的平均功率

        参数：
            duration: 时间窗口长度（秒）

        返回：
            float: 平均功率 (W)
        """
        if not self.data_buffer:
            return 0.0

        # 获取最近N个样本（N = duration × sample_rate）
        recent_samples = self.data_buffer[
            -int(duration * self.default_sample_rate) :
        ]
        if not recent_samples:
            return 0.0

        return sum(s["power_watts"] for s in recent_samples) / len(
            recent_samples
        )

    def _calculate_average_temp(self, duration):
        """
        计算指定时间窗口内的平均温度

        参数：
            duration: 时间窗口长度（秒）

        返回：
            float: 平均温度 (°C)
        """
        if not self.data_buffer:
            return 0.0

        recent_samples = self.data_buffer[
            -int(duration * self.default_sample_rate) :
        ]
        if not recent_samples:
            return 0.0

        return sum(s["temperature"] for s in recent_samples) / len(
            recent_samples
        )

    def _save_steady_state_results(self, results, base_filename):
        """
        保存稳态实验汇总结果

        将各温度点的平均温度和功率保存到单独的结果文件。

        参数：
            results: 结果列表，每项包含 target_temp, avg_temp, avg_power
            base_filename: 基础文件名，结果文件名在此基础上添加 _results 后缀
        """
        filename = base_filename.replace(".csv", "_results.csv")
        filepath = os.path.join(self.data_dir, filename)

        try:
            with open(filepath, "w", newline="") as csvfile:
                fieldnames = ["target_temp", "avg_temp", "avg_power"]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for r in results:
                    writer.writerow(r)
        except Exception as e:
            logging.error("保存稳态结果失败: %s", e)

    def _run_cooling_phase(self, mode, duration, target_temp, gcmd):
        """
        执行冷却数据采集阶段

        在完成所有稳态温度点测量后，关闭加热器并采集自然冷却数据。
        支持两种模式：固定时长模式和目标温度模式。

        参数：
            mode: 冷却模式，"duration" 或 "target_temp"
            duration: 冷却持续时间（秒），仅在 duration 模式下使用
            target_temp: 冷却目标温度 (°C)，仅在 target_temp 模式下使用
            gcmd: G-code命令对象，用于输出信息

        返回：
            dict: 冷却阶段统计信息，包含：
                - start_temp: 冷却开始温度 (°C)
                - end_temp: 冷却结束温度 (°C)
                - duration: 冷却持续时间 (秒)
                - mode: 使用的冷却模式
                - samples: 采集的样本数
        """
        self.current_phase = "cooling"

        self._set_heater_power(0.0, 0.0)

        pheaters = self.printer.lookup_object("heaters")
        pheaters.set_temperature(self.heater, 0.0, wait=False)

        start_temp = self._get_sensor_temp()
        cooling_start_time = self.reactor.monotonic()
        samples_at_start = len(self.data_buffer)

        gcmd.respond_info(f"冷却阶段开始 - 当前温度: {start_temp:.1f}°C")

        check_interval = 1.0
        last_progress_time = cooling_start_time
        progress_interval = 30.0

        cooling_end_temp = start_temp

        if mode == "duration":
            gcmd.respond_info(f"冷却模式: 固定时长 {duration}s")

            while True:
                if not self.is_collecting:
                    break

                current_temp = self._get_sensor_temp()
                elapsed = self.reactor.monotonic() - cooling_start_time

                if elapsed >= duration:
                    cooling_end_temp = current_temp
                    break

                if (
                    self.reactor.monotonic() - last_progress_time
                    >= progress_interval
                ):
                    remaining = duration - elapsed
                    gcmd.respond_info(
                        f"冷却中... 当前: {current_temp:.1f}°C, "
                        f"已耗时: {elapsed:.0f}s, 剩余: {remaining:.0f}s"
                    )
                    last_progress_time = self.reactor.monotonic()

                self.reactor.pause(self.reactor.monotonic() + check_interval)

        else:
            gcmd.respond_info(f"冷却模式: 目标温度 {target_temp}°C")

            while True:
                if not self.is_collecting:
                    break

                current_temp = self._get_sensor_temp()
                elapsed = self.reactor.monotonic() - cooling_start_time

                if current_temp <= target_temp:
                    cooling_end_temp = current_temp
                    gcmd.respond_info(
                        f"已达到目标温度 {target_temp}°C (实际: {current_temp:.1f}°C)"
                    )
                    break

                if (
                    self.reactor.monotonic() - last_progress_time
                    >= progress_interval
                ):
                    temp_diff = current_temp - target_temp
                    gcmd.respond_info(
                        f"冷却中... 当前: {current_temp:.1f}°C, "
                        f"距目标: {temp_diff:.1f}°C, 已耗时: {elapsed:.0f}s"
                    )
                    last_progress_time = self.reactor.monotonic()

                self.reactor.pause(self.reactor.monotonic() + check_interval)

        cooling_end_time = self.reactor.monotonic()
        actual_duration = cooling_end_time - cooling_start_time
        samples_collected = len(self.data_buffer) - samples_at_start

        cooling_data = {
            "start_temp": start_temp,
            "end_temp": cooling_end_temp,
            "duration": actual_duration,
            "mode": mode,
            "samples": samples_collected,
        }

        if mode == "duration":
            cooling_data["target_duration"] = duration
        else:
            cooling_data["target_temp"] = target_temp

        gcmd.respond_info(
            f"冷却阶段完成\n"
            f"  起始温度: {start_temp:.1f}°C\n"
            f"  结束温度: {cooling_end_temp:.1f}°C\n"
            f"  温降: {start_temp - cooling_end_temp:.1f}°C\n"
            f"  耗时: {actual_duration:.1f}s\n"
            f"  采集样本: {samples_collected}"
        )

        return cooling_data

    def _save_cooling_results(self, cooling_data, base_filename):
        """
        保存冷却阶段汇总结果

        将冷却阶段的统计信息保存到单独的结果文件。

        参数：
            cooling_data: 冷却阶段数据字典
            base_filename: 基础文件名，结果文件名在此基础上添加 _cooling 后缀
        """
        filename = base_filename.replace(".csv", "_cooling.csv")
        filepath = os.path.join(self.data_dir, filename)

        try:
            with open(filepath, "w", newline="") as csvfile:
                fieldnames = [
                    "start_temp",
                    "end_temp",
                    "duration",
                    "mode",
                    "samples",
                    "target_duration",
                    "target_temp",
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerow(cooling_data)
            logging.info("冷却结果已保存到: %s", filepath)
        except Exception as e:
            logging.error("保存冷却结果失败: %s", e)

    cmd_THERMAL_ID_CALIBRATE_help = "运行综合热参数辨识实验"

    def cmd_THERMAL_ID_CALIBRATE(self, gcmd):
        """
        处理 THERMAL_ID_CALIBRATE 命令 - 综合热参数辨识实验

        该实验包含4个顺序阶段：

        时间轴（秒）：
        ├── 0-50 ────┼── 50-240 ────┼── 240-稳态后 ───┼── 冷却30秒 ──┤
        │ 阶段1      │   阶段2      │     阶段3       │    阶段4     │
        │ 开环阶跃   │  PRBS激励    │   450°C稳态     │    冷却      │
        │ (50秒)     │  (190秒)     │  (稳定后30秒)   │   (30秒)     │

        辨识目标：
        - 阶段1：辨识θ_8和整体热容（开环满功率阶跃响应）
        - 阶段2：辨识动态参数（PRBS伪随机序列激励）
        - 阶段3：辨识散热参数（450°C稳态数据采集）
        - 阶段4：辨识冷却特性（自然冷却温度衰减）

        用法：
            THERMAL_ID_CALIBRATE [HEATER=<加热器>] [SAMPLE_RATE=<采样率>]
                                [MAX_TEMP=<最高温度>] [FILENAME=<文件名>]
                                [STEP1_DURATION=<秒>] [STEP1_TARGET_TEMP=<温度>]
                                [STEP2_DURATION=<秒>]
                                [STEP3_TEMP=<温度1,温度2,...>] [STEP3_STABLE_DURATION=<秒>]
                                [STEP4_DURATION=<秒>]
                                [PRBS_LOW_POWER=<比例>] [PRBS_HIGH_POWER=<比例>]

        参数：
            HEATER: 加热器名称，默认 "extruder"
            SAMPLE_RATE: 采样频率 (Hz)，默认 20.0
            MAX_TEMP: 最高温度限制 (°C)，默认 500.0
            FILENAME: 输出文件名

            阶段参数（可自定义）：
            STEP1_DURATION: 阶段1开环阶跃最大持续时间，默认 50秒
            STEP1_TARGET_TEMP: 阶段1目标温度，默认 200°C
            STEP2_DURATION: 阶段2 PRBS激励持续时间，默认 190秒
            STEP3_TEMP: 阶段3稳态目标温度，支持多个温度值（逗号分隔），默认 450°C
                        例如: STEP3_TEMP=300,400 或 STEP3_TEMP=200,300,400
            STEP3_STABLE_DURATION: 阶段3每个温度点稳态数据采集时长，默认 30秒
            STEP4_DURATION: 阶段4冷却持续时间，默认 30秒

            PRBS参数：
            PRBS_LOW_POWER: PRBS低功率电平，默认 0.2 (20%)
            PRBS_HIGH_POWER: PRBS高功率电平，默认 0.6 (60%)

        示例：
            THERMAL_ID_CALIBRATE HEATER=extruder
            THERMAL_ID_CALIBRATE HEATER=extruder MAX_TEMP=480 STEP3_TEMP=420
            THERMAL_ID_CALIBRATE HEATER=extruder STEP3_TEMP=300,400
            THERMAL_ID_CALIBRATE HEATER=extruder STEP3_TEMP=200,300,400,500
        """
        heater_name = gcmd.get("HEATER", "extruder")
        sample_rate = gcmd.get_float("SAMPLE_RATE", 20.0, above=0.0)
        max_temp = gcmd.get_float("MAX_TEMP", 500.0, above=0.0)
        filename = gcmd.get(
            "FILENAME", f"thermal_id_{time.strftime('%Y%m%d%H%M')}.csv"
        )

        step1_duration = gcmd.get_float("STEP1_DURATION", 50.0, above=0.0)
        step1_target_temp = gcmd.get_float(
            "STEP1_TARGET_TEMP", 200.0, above=0.0
        )
        step2_duration = gcmd.get_float("STEP2_DURATION", 190.0, above=0.0)

        step3_temp_raw = gcmd.get("STEP3_TEMP", "450")
        step3_temps = []
        for temp_str in step3_temp_raw.split(","):
            temp_str = temp_str.strip()
            if temp_str:
                try:
                    temp_val = float(temp_str)
                    if temp_val <= 0:
                        raise gcmd.error(
                            f"STEP3_TEMP 温度值必须大于0: {temp_val}"
                        )
                    step3_temps.append(temp_val)
                except ValueError:
                    raise gcmd.error(
                        f"无法解析 STEP3_TEMP 温度值: '{temp_str}'"
                    )
        if not step3_temps:
            step3_temps = [450.0]

        step3_stable_duration = gcmd.get_float(
            "STEP3_STABLE_DURATION", 30.0, above=0.0
        )
        step4_duration = gcmd.get_float("STEP4_DURATION", 30.0, above=0.0)

        prbs_low_power = gcmd.get_float(
            "PRBS_LOW_POWER", 0.2, minval=0.0, maxval=1.0
        )
        prbs_high_power = gcmd.get_float(
            "PRBS_HIGH_POWER", 0.6, minval=0.0, maxval=1.0
        )

        try:
            self.heater = self._get_heater(heater_name)
            self.heater_name = heater_name
        except Exception as e:
            raise gcmd.error(f"未找到加热器 '{heater_name}': {e}")

        total_duration = (
            step1_duration
            + step2_duration
            + step3_stable_duration * len(step3_temps)
            + step4_duration
        )

        gcmd.respond_info(
            f"启动综合热参数辨识实验\n"
            f"加热器: {heater_name}\n"
            f"预计总时长: {total_duration:.0f}秒 ({total_duration / 60:.1f}分钟)\n"
            f"采样率: {sample_rate} Hz\n"
            f"最高温度限制: {max_temp}°C\n"
            f"\n阶段配置:\n"
            f"  阶段1: 开环阶跃响应 {step1_duration:.0f}秒 (满功率升温至{step1_target_temp}°C)\n"
            f"  阶段2: PRBS动态激励 {step2_duration:.0f}秒 ({prbs_low_power * 100:.0f}%-{prbs_high_power * 100:.0f}%功率)\n"
            f"  阶段3: 稳态实验 {len(step3_temps)}个温度点 ({', '.join([f'{t:.0f}°C' for t in step3_temps])})\n"
            f"         每个温度点稳定后采集{step3_stable_duration:.0f}秒\n"
            f"  阶段4: 冷却阶段 {step4_duration:.0f}秒 (关闭加热，记录温度衰减)"
        )

        self.default_sample_rate = sample_rate
        self._start_collection("thermal_identification")

        experiment_results = {
            "step1": {},
            "step2": {},
            "step3": {},
            "step4": {},
        }

        try:
            experiment_start = self.reactor.monotonic()

            # 阶段1: 开环阶跃响应 (50秒满功率升温至目标温度)
            gcmd.respond_info("\n" + "=" * 50)
            gcmd.respond_info(
                f"阶段1: 开环阶跃响应 - 满功率升温至{step1_target_temp}°C"
            )
            gcmd.respond_info("=" * 50)
            self.current_phase = "open_loop_step"
            step1_results = self._run_open_loop_step(
                step1_duration, step1_target_temp, max_temp, gcmd
            )
            experiment_results["step1"] = step1_results

            # 阶段2: PRBS动态激励 (190秒)
            gcmd.respond_info("\n" + "=" * 50)
            gcmd.respond_info("阶段2: PRBS动态激励")
            gcmd.respond_info("=" * 50)
            self.current_phase = "prbs_excitation"
            step2_results = self._run_step5_safe_prbs(
                step2_duration, prbs_low_power, prbs_high_power, max_temp, gcmd
            )
            experiment_results["step2"] = step2_results

            step3_all_results = []
            for i, step3_temp in enumerate(step3_temps):
                gcmd.respond_info("\n" + "=" * 50)
                gcmd.respond_info(
                    f"阶段3.{i + 1}: {step3_temp}°C稳态实验 ({i + 1}/{len(step3_temps)})"
                )
                gcmd.respond_info("=" * 50)
                self.current_phase = f"steady_state_{i + 1}"
                step3_result = self._run_steady_state_measurement(
                    step3_temp, step3_stable_duration, gcmd
                )
                step3_result["temp_index"] = i + 1
                step3_result["temp_count"] = len(step3_temps)
                step3_all_results.append(step3_result)
            experiment_results["step3"] = step3_all_results

            # 阶段4: 冷却阶段 (30秒)
            gcmd.respond_info("\n" + "=" * 50)
            gcmd.respond_info("阶段4: 冷却阶段")
            gcmd.respond_info("=" * 50)
            self.current_phase = "cooling"
            step4_results = self._run_cooling_phase(
                "duration", step4_duration, 0.0, gcmd
            )
            experiment_results["step4"] = step4_results

        except Exception as e:
            gcmd.respond_raw(f"!! 实验中断: {e}")
        finally:
            self._set_heater_power(0.0, 0.0)
            self._stop_collection()
            self._save_data_to_csv(filename)

        self._save_thermal_id_results(experiment_results, filename)

        actual_duration = self.reactor.monotonic() - experiment_start

        gcmd.respond_info("\n" + "=" * 50)
        gcmd.respond_info("综合热参数辨识实验完成")
        gcmd.respond_info("=" * 50)
        gcmd.respond_info(
            f"总耗时: {actual_duration:.1f}秒 ({actual_duration / 60:.1f}分钟)\n"
            f"数据已保存到: {filename}\n"
            f"结果汇总已保存到: {filename.replace('.csv', '_results.csv')}"
        )

        self._print_thermal_id_summary(experiment_results, gcmd)

    def _run_open_loop_step(self, duration, target_temp, max_temp, gcmd):
        """
        执行开环阶跃实验

        开环满功率升温，用于辨识θ_8和整体热容。
        该方法直接控制加热器功率，不使用PID闭环控制。

        参数：
            duration: 最大持续时间（秒）
            target_temp: 目标温度（达到后停止）
            max_temp: 最高温度限制 (°C)
            gcmd: G-code命令对象

        返回：
            dict: 阶段结果，包含：
                - start_temp: 起始温度 (°C)
                - end_temp: 结束温度 (°C)
                - duration: 实际升温时长 (秒)
                - reached_target: 是否达到目标温度
                - initial_slope: 初始升温斜率 (°C/s)
                - samples_count: 采集样本数
        """
        self._switch_to_open_loop_control()

        start_time = self.reactor.monotonic()
        start_temp = self._get_sensor_temp()

        gcmd.respond_info(f"开始满功率升温，目标温度: {target_temp}°C")

        self._set_heater_power(self.heater.get_max_power(), target_temp)

        initial_samples = []
        all_samples = []
        initial_period = 5.0
        sample_interval = 0.1
        last_sample_time = start_time

        reached_target = False
        actual_duration = 0.0

        while True:
            current_time = self.reactor.monotonic()
            elapsed = current_time - start_time
            current_temp = self._get_sensor_temp()

            if current_time - last_sample_time >= sample_interval:
                sample_data = {
                    "elapsed": elapsed,
                    "temp": current_temp,
                }
                all_samples.append(sample_data)

                if elapsed <= initial_period:
                    initial_samples.append(sample_data)

                last_sample_time = current_time

            if current_temp >= target_temp:
                reached_target = True
                actual_duration = elapsed
                gcmd.respond_info(
                    f"已达到目标温度 {current_temp:.1f}°C (耗时 {elapsed:.1f}秒)"
                )
                break

            if current_temp >= max_temp:
                gcmd.respond_info(
                    f"温度达到上限 {current_temp:.1f}°C，停止升温"
                )
                actual_duration = elapsed
                break

            if elapsed >= duration:
                gcmd.respond_info(
                    f"达到最大时长 {duration}秒，当前温度 {current_temp:.1f}°C"
                )
                actual_duration = duration
                break

            if elapsed % 10.0 < 0.2:
                gcmd.respond_info(
                    f"  升温中... {elapsed:.0f}s: {current_temp:.1f}°C"
                )

            self.reactor.pause(current_time + 0.05)

        initial_slope = 0.0
        if len(initial_samples) >= 3:
            temps = [s["temp"] for s in initial_samples]
            times = [s["elapsed"] for s in initial_samples]
            if times[-1] > times[0]:
                initial_slope = (temps[-1] - temps[0]) / (times[-1] - times[0])

        end_temp = self._get_sensor_temp()

        self._set_heater_power(0.0, 0.0)
        self._restore_original_control()

        results = {
            "start_temp": start_temp,
            "end_temp": end_temp,
            "duration": actual_duration,
            "reached_target": reached_target,
            "initial_slope": initial_slope,
            "samples_count": len(all_samples),
        }

        gcmd.respond_info(
            f"阶段1完成:\n"
            f"  起始温度: {start_temp:.1f}°C\n"
            f"  结束温度: {end_temp:.1f}°C\n"
            f"  升温时长: {actual_duration:.1f}秒\n"
            f"  初始升温斜率: {initial_slope:.2f}°C/s (用于辨识θ_8)\n"
            f"  采集样本: {len(all_samples)}"
        )

        return results

    def _run_prbs_experiment(
        self,
        duration,
        min_pulse,
        max_pulse,
        power_levels,
        max_temp,
        gcmd,
        adaptive_power=True,
        resume_pulse=True,
    ):
        """
        执行PRBS动态激励实验

        开环功率控制，带温度保护机制。支持自适应功率调整和脉冲恢复。

        参数：
            duration: 实验总时长 (秒)
            min_pulse: 最小脉冲宽度 (秒)
            max_pulse: 最大脉冲宽度 (秒)
            power_levels: 功率电平列表 [p1, p2, ...]
            max_temp: 最高温度限制 (°C)
            gcmd: G-code命令对象
            adaptive_power: 是否启用自适应功率调整 (默认True)
                           - True: 根据温度自动选择高低功率
                           - False: 严格按PRBS序列执行
            resume_pulse: 是否启用脉冲恢复 (默认True)
                         - True: 温度恢复后继续当前脉冲
                         - False: 温度恢复后进入下一脉冲

        返回：
            dict: PRBS实验结果，包含：
                - duration: 实际运行时长 (秒)
                - effective_duration: 有效实验时长 (秒)
                - protection_time: 保护暂停时长 (秒)
                - start_temp: 起始温度 (°C)
                - end_temp: 结束温度 (°C)
                - samples_count: 采集样本数
                - pulse_count: 脉冲数量
        """
        self._switch_to_open_loop_control()

        temp_high_limit = max_temp - 10.0
        temp_low_limit = 50

        gcmd.respond_info(
            f"PRBS参数:\n"
            f"  功率电平: {[f'{p * 100:.0f}%' for p in power_levels]}\n"
            f"  脉冲宽度: {min_pulse}s - {max_pulse}s\n"
            f"  温度限制: {temp_low_limit}°C - {temp_high_limit}°C (上限: {max_temp}°C)"
        )

        prbs_sequence = self._generate_prbs_sequence(
            duration, min_pulse, max_pulse, power_levels
        )

        # 输出完整PRBS序列
        prbs_str = "\n".join(
            [
                f"    功率: {p * 100:.0f}%, 时长: {d:.1f}s"
                for p, d in prbs_sequence
            ]
        )
        gcmd.respond_info(
            f"生成PRBS序列:\n"
            f"  总脉冲数: {len(prbs_sequence)}\n"
            f"  完整序列:\n{prbs_str}"
        )

        start_time = self.reactor.monotonic()
        start_temp = self._get_sensor_temp()
        samples_at_start = len(self.data_buffer)

        sequence_index = 0
        total_protection_time = 0.0
        current_pulse_remaining = 0.0
        saved_pulse_power = 0.0
        pulse_logged = False  # 标记当前脉冲是否已输出日志

        try:
            while sequence_index < len(prbs_sequence):
                if not self.is_collecting:
                    break

                pulse_power, pulse_duration = prbs_sequence[sequence_index]

                if current_pulse_remaining > 0 and resume_pulse:
                    pulse_duration = current_pulse_remaining
                    pulse_power = saved_pulse_power
                else:
                    current_pulse_remaining = pulse_duration
                    saved_pulse_power = pulse_power

                pulse_start = self.reactor.monotonic()
                current_temp = self._get_sensor_temp()

                if adaptive_power and len(power_levels) >= 2:
                    low_power = min(power_levels)
                    high_power = max(power_levels)

                    if current_temp >= temp_high_limit:
                        pulse_power = low_power
                        gcmd.respond_info(
                            f"温度 {current_temp:.1f}°C 接近上限，使用低功率"
                        )
                    elif current_temp <= temp_low_limit:
                        pulse_power = high_power
                        gcmd.respond_info(
                            f"温度 {current_temp:.1f}°C 较低，使用高功率"
                        )

                target_temp = current_temp + 15
                self._set_heater_power(pulse_power, target_temp)

                # 脉冲开始时输出一次状态信息
                gcmd.respond_info(
                    f"PRBS脉冲 {sequence_index + 1}/{len(prbs_sequence)}: "
                    f"功率{pulse_power * 100:.0f}%, "
                    f"时长{pulse_duration:.1f}s, "
                    f"温度{current_temp:.1f}°C"
                )
                pulse_logged = True

                while True:
                    if not self.is_collecting:
                        break

                    current_time = self.reactor.monotonic()
                    elapsed_in_pulse = current_time - pulse_start
                    current_temp = self._get_sensor_temp()

                    if current_temp >= max_temp:
                        gcmd.respond_info(
                            f"温度超限 ({current_temp:.1f}°C >= {max_temp}°C)，暂停加热"
                        )
                        self._set_heater_power(0.0, 0.0)

                        protection_start = current_time
                        recovery_temp = max(temp_low_limit, 50.0)

                        while self._get_sensor_temp() >= temp_high_limit:
                            if not self.is_collecting:
                                break
                            self.reactor.pause(self.reactor.monotonic() + 0.1)

                        protection_end = self.reactor.monotonic()
                        protection_duration = protection_end - protection_start
                        total_protection_time += protection_duration

                        gcmd.respond_info(
                            f"温度恢复至 {self._get_sensor_temp():.1f}°C，"
                            f"暂停时长: {protection_duration:.1f}s"
                        )

                        if resume_pulse:
                            elapsed_in_pulse = (
                                current_time - pulse_start - protection_duration
                            )
                            current_pulse_remaining = (
                                pulse_duration - elapsed_in_pulse
                            )
                            saved_pulse_power = pulse_power

                            if current_pulse_remaining <= 0:
                                sequence_index += 1
                                current_pulse_remaining = 0.0
                                pulse_logged = False
                        break

                    if elapsed_in_pulse >= pulse_duration:
                        current_pulse_remaining = 0.0
                        break

                    self.reactor.pause(current_time + 0.1)

                if current_temp < max_temp and current_pulse_remaining <= 0:
                    sequence_index += 1
                    pulse_logged = False

        finally:
            self._set_heater_power(0.0, 0.0)
            self._restore_original_control()

        end_temp = self._get_sensor_temp()
        actual_duration = self.reactor.monotonic() - start_time
        samples_collected = len(self.data_buffer) - samples_at_start

        results = {
            "duration": actual_duration,
            "effective_duration": actual_duration - total_protection_time,
            "protection_time": total_protection_time,
            "start_temp": start_temp,
            "end_temp": end_temp,
            "samples_count": samples_collected,
            "pulse_count": len(prbs_sequence),
        }

        gcmd.respond_info(
            f"PRBS实验完成:\n"
            f"  实际时长: {actual_duration:.1f}秒\n"
            f"  有效时长: {actual_duration - total_protection_time:.1f}秒\n"
            f"  保护暂停: {total_protection_time:.1f}秒\n"
            f"  脉冲数量: {len(prbs_sequence)}\n"
            f"  采集样本: {samples_collected}"
        )

        return results

    def _run_step5_safe_prbs(
        self, duration, low_power, high_power, max_temp, gcmd
    ):
        """
        执行阶段5：安全PRBS动态激励（综合实验专用）

        该方法是 _run_prbs_experiment 的简化封装，用于综合热参数辨识实验。
        使用固定脉冲宽度 (4-15秒) 和自适应功率调整。

        参数：
            duration: 持续时间 (秒)
            low_power: 低功率电平 (0.0-1.0)
            high_power: 高功率电平 (0.0-1.0)
            max_temp: 最高温度限制 (°C)
            gcmd: G-code命令对象

        返回：
            dict: PRBS阶段结果
        """
        return self._run_prbs_experiment(
            duration=duration,
            min_pulse=5.0,
            max_pulse=15.0,
            power_levels=[0, 0, 0, 0, 0.25, 0.5, 0.75],
            max_temp=max_temp,
            gcmd=gcmd,
            adaptive_power=True,
            resume_pulse=False,
        )

    def _save_thermal_id_results(self, results, base_filename):
        """
        保存综合热参数辨识实验结果

        参数：
            results: 各阶段结果字典
            base_filename: 基础文件名
        """
        filename = base_filename.replace(".csv", "_results.csv")
        filepath = os.path.join(self.data_dir, filename)

        try:
            with open(filepath, "w", newline="") as csvfile:
                fieldnames = [
                    "phase",
                    "parameter",
                    "value",
                    "unit",
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

                step1 = results.get("step1", {})
                writer.writerow(
                    {
                        "phase": "step1",
                        "parameter": "start_temp",
                        "value": step1.get("start_temp", 0),
                        "unit": "°C",
                    }
                )
                writer.writerow(
                    {
                        "phase": "step1",
                        "parameter": "end_temp",
                        "value": step1.get("end_temp", 0),
                        "unit": "°C",
                    }
                )
                writer.writerow(
                    {
                        "phase": "step1",
                        "parameter": "duration",
                        "value": step1.get("duration", 0),
                        "unit": "s",
                    }
                )
                writer.writerow(
                    {
                        "phase": "step1",
                        "parameter": "initial_slope",
                        "value": step1.get("initial_slope", 0),
                        "unit": "°C/s",
                    }
                )
                writer.writerow(
                    {
                        "phase": "step1",
                        "parameter": "reached_target",
                        "value": step1.get("reached_target", False),
                        "unit": "",
                    }
                )

                step2 = results.get("step2", {})
                writer.writerow(
                    {
                        "phase": "step2",
                        "parameter": "target_temp",
                        "value": step2.get("target_temp", 0),
                        "unit": "°C",
                    }
                )
                writer.writerow(
                    {
                        "phase": "step2",
                        "parameter": "avg_temp",
                        "value": step2.get("avg_temp", 0),
                        "unit": "°C",
                    }
                )
                writer.writerow(
                    {
                        "phase": "step2",
                        "parameter": "avg_power",
                        "value": step2.get("avg_power", 0),
                        "unit": "W",
                    }
                )

                step3_results = results.get("step3", [])
                if isinstance(step3_results, list):
                    for i, step3_data in enumerate(step3_results):
                        phase_name = f"step3_{i + 1}"
                        writer.writerow(
                            {
                                "phase": phase_name,
                                "parameter": "target_temp",
                                "value": step3_data.get("target_temp", 0),
                                "unit": "°C",
                            }
                        )
                        writer.writerow(
                            {
                                "phase": phase_name,
                                "parameter": "avg_temp",
                                "value": step3_data.get("avg_temp", 0),
                                "unit": "°C",
                            }
                        )
                        writer.writerow(
                            {
                                "phase": phase_name,
                                "parameter": "avg_power",
                                "value": step3_data.get("avg_power", 0),
                                "unit": "W",
                            }
                        )
                        writer.writerow(
                            {
                                "phase": phase_name,
                                "parameter": "stable_duration",
                                "value": step3_data.get("stable_duration", 0),
                                "unit": "s",
                            }
                        )
                else:
                    step3_data = step3_results
                    writer.writerow(
                        {
                            "phase": "step3",
                            "parameter": "target_temp",
                            "value": step3_data.get("target_temp", 0),
                            "unit": "°C",
                        }
                    )
                    writer.writerow(
                        {
                            "phase": "step3",
                            "parameter": "avg_temp",
                            "value": step3_data.get("avg_temp", 0),
                            "unit": "°C",
                        }
                    )
                    writer.writerow(
                        {
                            "phase": "step3",
                            "parameter": "avg_power",
                            "value": step3_data.get("avg_power", 0),
                            "unit": "W",
                        }
                    )

                step4 = results.get("step4", {})
                writer.writerow(
                    {
                        "phase": "step4",
                        "parameter": "target_temp",
                        "value": step4.get("target_temp", 0),
                        "unit": "°C",
                    }
                )
                writer.writerow(
                    {
                        "phase": "step4",
                        "parameter": "avg_temp",
                        "value": step4.get("avg_temp", 0),
                        "unit": "°C",
                    }
                )
                writer.writerow(
                    {
                        "phase": "step4",
                        "parameter": "avg_power",
                        "value": step4.get("avg_power", 0),
                        "unit": "W",
                    }
                )

                step5 = results.get("step5", {})
                writer.writerow(
                    {
                        "phase": "step5",
                        "parameter": "duration",
                        "value": step5.get("duration", 0),
                        "unit": "s",
                    }
                )
                writer.writerow(
                    {
                        "phase": "step5",
                        "parameter": "effective_duration",
                        "value": step5.get("effective_duration", 0),
                        "unit": "s",
                    }
                )
                writer.writerow(
                    {
                        "phase": "step5",
                        "parameter": "pulse_count",
                        "value": step5.get("pulse_count", 0),
                        "unit": "",
                    }
                )

            logging.info("热参数辨识结果已保存到: %s", filepath)
        except Exception as e:
            logging.error("保存热参数辨识结果失败: %s", e)

    def _print_thermal_id_summary(self, results, gcmd):
        """
        打印实验结果摘要

        参数：
            results: 各阶段结果字典
            gcmd: G-code命令对象
        """
        gcmd.respond_info("\n实验结果摘要:")
        gcmd.respond_info("-" * 40)

        step1 = results.get("step1", {})
        if step1.get("initial_slope"):
            gcmd.respond_info(
                f"θ_8估计 (初始升温斜率): {step1.get('initial_slope', 0):.3f} °C/s"
            )

        gcmd.respond_info("\n稳态散热功率:")
        step2 = results.get("step2", {})
        if step2.get("avg_power"):
            gcmd.respond_info(
                f"  PRBS阶段: P = {step2.get('avg_power', 0):.2f}W "
                f"(T_avg = {step2.get('avg_temp', 0):.1f}°C)"
            )

        step3_results = results.get("step3", [])
        if isinstance(step3_results, list):
            for i, step3_data in enumerate(step3_results):
                if step3_data.get("avg_power"):
                    gcmd.respond_info(
                        f"  稳态{i + 1} ({step3_data.get('target_temp', 0):.0f}°C): "
                        f"P = {step3_data.get('avg_power', 0):.2f}W "
                        f"(T_avg = {step3_data.get('avg_temp', 0):.1f}°C)"
                    )
        else:
            step3_data = step3_results
            if step3_data.get("avg_power"):
                gcmd.respond_info(
                    f"  稳态 ({step3_data.get('target_temp', 0):.0f}°C): "
                    f"P = {step3_data.get('avg_power', 0):.2f}W "
                    f"(T_avg = {step3_data.get('avg_temp', 0):.1f}°C)"
                )

        step4 = results.get("step4", {})
        if step4.get("avg_power"):
            gcmd.respond_info(
                f"  冷却阶段: P = {step4.get('avg_power', 0):.2f}W "
                f"(T_avg = {step4.get('avg_temp', 0):.1f}°C)"
            )

        step5 = results.get("step5", {})
        if step5.get("pulse_count"):
            gcmd.respond_info(
                f"\nPRBS动态激励: {step5.get('pulse_count', 0)}个脉冲, "
                f"有效时长 {step5.get('effective_duration', 0):.1f}s"
            )

        gcmd.respond_info("-" * 40)

    cmd_PRBS_CALIBRATE_help = "运行PRBS动态激励校准实验"

    def cmd_PRBS_CALIBRATE(self, gcmd):
        """
        处理 PRBS_CALIBRATE 命令 - PRBS动态激励实验

        该实验通过伪随机二进制序列功率信号，辨识系统的动态热参数。

        原理说明：
            PRBS信号包含丰富的频率成分，能激发系统的动态响应特性：
            - 短脉冲（0.2-1s）：激发小热容节点（加热芯）的快速响应
            - 长脉冲（5-20s）：反映整体热容和传导滞后

        用法：
            PRBS_CALIBRATE [HEATER=<加热器>] [DURATION=<总时长>]
                          [MIN_PULSE=<最小脉宽>] [MAX_PULSE=<最大脉宽>]
                          [POWER_LEVELS=<功率电平>] [SAMPLE_RATE=<采样率>]
                          [MAX_TEMP=<最高温度>] [FILENAME=<文件名>]
                          [ADAPTIVE_POWER=<0|1>]

        参数：
            HEATER: 加热器名称，默认 "extruder"
            DURATION: 实验总时长 (秒)，默认 300
            MIN_PULSE: 最小脉冲宽度 (秒)，默认 0.2
            MAX_PULSE: 最大脉冲宽度 (秒)，默认 10.0
            POWER_LEVELS: 功率电平序列，逗号分隔，默认 "0.0,1.0"
            SAMPLE_RATE: 采样频率 (Hz)，默认 20.0
            MAX_TEMP: 最高温度限制 (°C)，默认 300.0
            FILENAME: 输出文件名
            ADAPTIVE_POWER: 是否启用自适应功率调整，默认 1

        示例：
            PRBS_CALIBRATE HEATER=extruder DURATION=600 MIN_PULSE=0.2 MAX_PULSE=15 POWER_LEVELS=0.0,0.5,1.0 MAX_TEMP=280
        """
        heater_name = gcmd.get("HEATER", "extruder")
        duration = gcmd.get_float("DURATION", 300.0, above=0.0)
        min_pulse = gcmd.get_float("MIN_PULSE", 0.2, above=0.0)
        max_pulse = gcmd.get_float(
            "MAX_PULSE", 10.0, above=0.0, minval=min_pulse
        )
        power_levels_str = gcmd.get("POWER_LEVELS", "0.0,1.0")
        sample_rate = gcmd.get_float("SAMPLE_RATE", 20.0, above=0.0)
        max_temp = gcmd.get_float("MAX_TEMP", 300.0, above=0.0)
        filename = gcmd.get(
            "FILENAME", f"prbs_data_{time.strftime('%Y%m%d%H%M')}.csv"
        )
        adaptive_power = (
            gcmd.get_int("ADAPTIVE_POWER", 1, minval=0, maxval=1) == 1
        )

        try:
            self.heater = self._get_heater(heater_name)
            self.heater_name = heater_name
        except Exception as e:
            raise gcmd.error(f"未找到加热器 '{heater_name}': {e}")

        try:
            powers = [float(p.strip()) for p in power_levels_str.split(",")]
        except ValueError:
            raise gcmd.error("POWER_LEVELS 格式无效。请使用逗号分隔的数值。")

        gcmd.respond_info(
            f"启动 '{heater_name}' 的PRBS动态激励校准\n"
            f"实验时长: {duration}s\n"
            f"脉冲宽度范围: {min_pulse}s - {max_pulse}s\n"
            f"功率电平: {powers}\n"
            f"采样率: {sample_rate} Hz\n"
            f"最高温度限制: {max_temp}°C\n"
            f"自适应功率: {'启用' if adaptive_power else '禁用'}"
        )

        self.default_sample_rate = sample_rate
        self._start_collection("prbs_dynamic")

        try:
            results = self._run_prbs_experiment(
                duration=duration,
                min_pulse=min_pulse,
                max_pulse=max_pulse,
                power_levels=powers,
                max_temp=max_temp,
                gcmd=gcmd,
                adaptive_power=adaptive_power,
                resume_pulse=True,
            )
        except Exception as e:
            gcmd.respond_raw(f"!! PRBS校准中断: {e}")
            results = None
        finally:
            self._stop_collection()
            self._save_data_to_csv(filename)

        if results:
            gcmd.respond_info(
                f"PRBS动态激励校准完成。\n"
                f"  实际运行时长: {results['duration']:.1f}s\n"
                f"  暂停时长: {results['protection_time']:.1f}s\n"
                f"  有效实验时长: {results['effective_duration']:.1f}s\n"
                f"  数据已保存到 {filename}"
            )

    def _generate_prbs_sequence(self, duration, min_pulse, max_pulse, powers):
        """
        生成伪随机二进制序列 (PRBS)

        生成一个随机的功率脉冲序列，用于系统辨识。

        参数：
            duration: 序列总时长（秒）
            min_pulse: 最小脉冲宽度（秒）
            max_pulse: 最大脉冲宽度（秒）
            powers: 可选功率电平列表

        返回：
            list: [(功率, 持续时间), ...] 元组列表

        注意：
            该序列满足"持续激励条件"，保证辨识结果收敛且唯一。
        """
        sequence = []
        total_time = 0.0

        while total_time < duration:
            # 随机生成脉冲宽度
            pulse_duration = random.uniform(min_pulse, max_pulse)
            # 确保不超过总时长
            pulse_duration = min(pulse_duration, duration - total_time)

            # 随机选择功率电平
            pulse_power = random.choice(powers)

            sequence.append((pulse_power, pulse_duration))
            total_time += pulse_duration

        return sequence

    def get_status(self, eventtime):
        """
        获取模块状态信息

        该方法供API Server和宏调用，返回当前采集状态。

        参数：
            eventtime: 事件时间（未使用，但必须保留以符合接口规范）

        返回：
            dict: 状态信息字典，包含：
                - is_collecting: 是否正在采集
                - current_experiment: 当前实验名称
                - current_phase: 当前阶段（heating/cooling）
                - samples_collected: 已采集样本数
                - sample_rate: 当前采样率
                - data_directory: 数据存储目录
        """
        return {
            "is_collecting": self.is_collecting,
            "current_experiment": self.current_experiment or "",
            "current_phase": self.current_phase,
            "samples_collected": len(self.data_buffer),
            "sample_rate": self.default_sample_rate,
            "data_directory": self.data_dir,
        }


# =============================================================================
# 热参数估计器类
# Thermal Parameter Estimator Class
# =============================================================================


class ThermalParameterEstimator:
    """
    热参数估计器类

    该类提供从采集数据估计热参数的功能，支持：
    - 稳态数据分析：估计线性散热系数和辐射系数
    - PRBS数据分析：提供外部工具分析建议

    数学模型：
        P_loss = K_lin × (T - T_amb) + K_rad × (T⁴ - T_amb⁴)

    其中：
        K_lin: 线性散热系数 (W/K)
        K_rad: 辐射系数 (W/K⁴)
    """

    def __init__(self, config):
        """
        初始化热参数估计器

        参数：
            config: 配置对象
        """
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")

        # 注册G-code命令
        self.gcode.register_command(
            "ESTIMATE_THERMAL_PARAMS",
            self.cmd_ESTIMATE_THERMAL_PARAMS,
            desc=self.cmd_ESTIMATE_THERMAL_PARAMS_help,
        )

    cmd_ESTIMATE_THERMAL_PARAMS_help = "从采集数据估计热参数"

    def cmd_ESTIMATE_THERMAL_PARAMS(self, gcmd):
        """
        处理 ESTIMATE_THERMAL_PARAMS 命令 - 估计热参数

        用法：
            ESTIMATE_THERMAL_PARAMS DATA_FILE=<数据文件> [METHOD=<方法>]

        参数：
            DATA_FILE: 数据文件路径
            METHOD: 估计方法，可选 "steady_state" 或 "prbs"，默认 "steady_state"

        示例：
            ESTIMATE_THERMAL_PARAMS DATA_FILE=~/printer_data/temp_data/steady_state_xxx.csv METHOD=steady_state
        """
        data_file = gcmd.get("DATA_FILE")
        method = gcmd.get("METHOD", "steady_state")

        if method == "steady_state":
            self._estimate_from_steady_state(data_file, gcmd)
        elif method == "prbs":
            self._estimate_from_prbs(data_file, gcmd)
        else:
            raise gcmd.error(f"未知的估计方法: {method}")

    def _estimate_from_steady_state(self, data_file, gcmd):
        """
        从稳态数据估计热参数

        使用最小二乘法拟合热损耗模型：
        P_loss = K_lin × (T - T_amb) + K_rad × (T⁴ - T_amb⁴)

        参数：
            data_file: 稳态实验结果文件路径
            gcmd: G-code命令对象
        """
        # 加载稳态结果文件
        try:
            results_file = data_file.replace(".csv", "_results.csv")
            results = self._load_csv(results_file)
        except Exception as e:
            raise gcmd.error(f"加载数据文件失败: {e}")

        if not results:
            raise gcmd.error("数据文件中未找到稳态结果")

        # 提取温度和功率数据
        temps = [float(r["avg_temp"]) for r in results]
        powers = [float(r["avg_power"]) for r in results]

        # 拟合热损耗模型
        k_lin, k_rad = self._fit_heat_loss_model(temps, powers)

        gcmd.respond_info(
            f"热参数估计结果:\n"
            f"  线性散热系数 (K_lin): {k_lin:.6f} W/K\n"
            f"  辐射系数 (K_rad): {k_rad:.9f} W/K^4\n"
            f"  热损耗模型: P_loss = K_lin × (T - T_amb) + K_rad × (T⁴ - T_amb⁴)"
        )

    def _fit_heat_loss_model(self, temps, powers):
        """
        拟合热损耗模型参数

        使用最小二乘法求解二元线性回归问题：
        y = a×x1 + b×x2

        其中：
            y = P_loss (功率)
            x1 = T - T_amb (线性项)
            x2 = T⁴ - T_amb⁴ (辐射项)
            a = K_lin
            b = K_rad

        参数：
            temps: 温度列表 (°C)
            powers: 功率列表 (W)

        返回：
            tuple: (K_lin, K_rad) 热损耗系数
        """
        n = len(temps)
        if n < 2:
            return 0.0, 0.0

        # 转换为开尔文温度
        T_amb = AMBIENT_TEMP + 273.15
        T_kelvin = [t + 273.15 for t in temps]

        # 计算回归所需的各项求和
        # 线性项：x1 = T - T_amb
        sum_x1 = sum(T - T_amb for T in T_kelvin)
        # 辐射项：x2 = T⁴ - T_amb⁴
        sum_x2 = sum(T**4 - T_amb**4 for T in T_kelvin)
        # 功率：y = P
        sum_y = sum(powers)
        # 平方项
        sum_x1x1 = sum((T - T_amb) ** 2 for T in T_kelvin)
        sum_x2x2 = sum((T**4 - T_amb**4) ** 2 for T in T_kelvin)
        # 交叉项
        sum_x1x2 = sum((T - T_amb) * (T**4 - T_amb**4) for T in T_kelvin)
        # 与功率的乘积
        sum_x1y = sum((T_kelvin[i] - T_amb) * powers[i] for i in range(n))
        sum_x2y = sum(
            (T_kelvin[i] ** 4 - T_amb**4) * powers[i] for i in range(n)
        )

        # 求解正规方程组
        # 行列式
        det = sum_x1x1 * sum_x2x2 - sum_x1x2**2
        if abs(det) < 1e-15:
            # 矩阵奇异，退化为单参数估计
            return sum_y / sum_x1 if sum_x1 != 0 else 0.0, 0.0

        # 克拉默法则求解
        k_lin = (sum_x1y * sum_x2x2 - sum_x2y * sum_x1x2) / det
        k_rad = (sum_x2y * sum_x1x1 - sum_x1y * sum_x1x2) / det

        return k_lin, k_rad

    def _estimate_from_prbs(self, data_file, gcmd):
        """
        从PRBS数据估计热参数

        PRBS数据分析需要更复杂的系统辨识方法，
        此方法提供数据加载和建议信息。

        参数：
            data_file: PRBS实验数据文件路径
            gcmd: G-code命令对象
        """
        try:
            data = self._load_csv(data_file)
        except Exception as e:
            raise gcmd.error(f"加载数据文件失败: {e}")

        if not data:
            raise gcmd.error("文件中未找到数据")

        gcmd.respond_info(
            f"已加载 {len(data)} 个PRBS数据样本。\n"
            f"建议使用外部工具进行高级系统辨识分析。"
        )

        gcmd.respond_info(
            "推荐分析方法:\n"
            "  1. 从阶跃响应识别时间常数\n"
            "  2. 使用ARX/ARMAX模型估计传递函数\n"
            "  3. 应用子空间辨识方法建立多节点模型"
        )

    def _load_csv(self, filepath):
        """
        加载CSV数据文件

        参数：
            filepath: CSV文件路径

        返回：
            list: 数据行列表，每行为字典格式
        """
        data = []
        try:
            with open(filepath, "r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    data.append(row)
        except Exception as e:
            logging.error("加载CSV失败: %s", e)
        return data


# =============================================================================
# 模块加载入口 (Module Entry Points)
# =============================================================================


def load_config(config):
    """
    模块加载入口函数

    当配置文件中存在 [temp_data_collector] 配置节时，
    Klippy会自动调用此函数加载模块。

    参数：
        config: 配置对象

    返回：
        TemperatureDataCollector 实例
    """
    return TemperatureDataCollector(config)


def load_config_prefix(config):
    """
    带前缀的模块加载入口函数

    当配置文件中存在 [thermal_estimator] 或类似带前缀的配置节时，
    Klippy会调用此函数。

    参数：
        config: 配置对象

    返回：
        ThermalParameterEstimator 实例
    """
    return ThermalParameterEstimator(config)
