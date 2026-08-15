# =============================================================================
# 温度数据采集模块 - 用于热参数辨识与控制器性能分析
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
# 本模块用于3D打印机加热系统的热参数辨识与控制器性能分析：
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
# 3. 控制器闭环性能采集 (Closed-Loop Performance Collection)
#    - 目的：在 PID/MPC 控制器运行期间采集 22 维全量字段
#    - 字段：温度、目标、PWM、EKF 隐状态、3σ 协方差、热损耗分解、
#           送丝速度、风扇、求解耗时与迭代次数、控制器类型与参数摘要
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

PIN_MIN_TIME = 0.100
AMBIENT_TEMP = 25.0
STABILITY_BEFORE_EXTRUDE = 15.0


class TemperatureDataCollector:
    """
    温度数据采集器类

    支持：
    - 手动数据采集模式 (22 维全量字段)
    - 稳态阶梯响应自动实验
    - PRBS动态激励自动实验
    - 自动识别当前控制器类型与参数并写入 CSV
    - 文件名自动带控制器后缀 (mpc_v2 / pid / open_loop)
    """

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")

        self.data_dir = os.path.expanduser(
            config.get("data_dir", "~/printer_data/temp_data")
        )
        self.default_sample_rate = config.getfloat(
            "sample_rate", 10.0, above=0.0
        )
        self.max_heater_power = config.getfloat(
            "max_heater_power", 60.0, above=0.0
        )

        # 额外物理传感器列表配置（实测Th/Tb热电偶、环境温度、水冷温度）
        # 格式: extra_sensors: temperature_sensor th_meas, temperature_sensor tb_meas
        extra_sensors_str = config.get("extra_sensors", "")
        self.extra_sensor_names = [
            s.strip() for s in extra_sensors_str.split(",") if s.strip()
        ]

        self.is_collecting = False
        self.collection_lock = threading.Lock()
        self.current_experiment = None
        self.current_target_temp = None
        self.current_extrude_speed = None
        self.current_phase = "heating"
        self.data_buffer = []
        self.sample_timer = None
        self.start_time = 0.0

        self.heater = None
        self.heater_name = None
        self.pwm_pin = None
        self.sensor = None

        self._open_loop_control = None
        self._original_control = None
        self._control_switched = False

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )

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

        if not os.path.exists(self.data_dir):
            try:
                os.makedirs(self.data_dir)
            except OSError as e:
                logging.warning("无法创建数据目录: %s", e)

    # =========================================================================
    # 事件处理方法 (Event Handlers)
    # =========================================================================

    def _handle_ready(self):
        pass

    def _handle_shutdown(self):
        self._stop_collection()

    # =========================================================================
    # 辅助方法 (Helper Methods)
    # =========================================================================

    def _get_heater(self, heater_name):
        pheaters = self.printer.lookup_object("heaters")
        return pheaters.lookup_heater(heater_name)

    def _get_sensor_temp(self):
        if self.heater is None:
            return 0.0
        return self.heater.get_temp(self.reactor.monotonic())[0]

    def _get_pwm_value(self):
        if self.heater is None:
            return 0.0
        return getattr(self.heater, "last_pwm_value", 0.0)

    def _detect_control_type(self):
        """探测当前加热器所使用的控制器类型字符串"""
        if self.heater is None:
            return "unknown"
        ctrl = getattr(self.heater, "control", None)
        if ctrl is None:
            return "unknown"
        if hasattr(ctrl, "get_type"):
            try:
                return ctrl.get_type()
            except Exception:
                pass
        if hasattr(ctrl, "get_profile"):
            try:
                prof = ctrl.get_profile()
                if isinstance(prof, dict):
                    return prof.get("name", "unknown")
            except Exception:
                pass
        return "unknown"

    def _build_control_params_summary(self, eventtime):
        """根据控制器类型构造参数摘要字符串"""
        ctrl = getattr(self.heater, "control", None)
        if ctrl is None:
            return ""
        try:
            st = ctrl.get_status(eventtime) if hasattr(ctrl, "get_status") else {}
            if not isinstance(st, dict):
                st = {}
            prof = ctrl.get_profile() if hasattr(ctrl, "get_profile") else {}
            if not isinstance(prof, dict):
                prof = {}

            ctype = self._detect_control_type()
            if ctype in ("mpc_v2", "mpc"):
                np_val = st.get("prediction_horizon", prof.get("prediction_horizon", ""))
                nc_val = st.get("control_horizon", prof.get("control_horizon", ""))
                wt_val = st.get("weight_tracking", prof.get("weight_tracking", ""))
                wr_val = st.get("weight_rate", prof.get("weight_rate", ""))
                return (
                    f"Np={np_val},"
                    f"Nc={nc_val},"
                    f"wt={wt_val},"
                    f"wr={wr_val}"
                )
            elif "pid" in ctype or ctype == "unknown":
                kp = st.get("Kp", getattr(ctrl, "Kp", prof.get("pid_kp", 0)))
                ki = st.get("Ki", getattr(ctrl, "Ki", prof.get("pid_ki", 0)))
                kd = st.get("Kd", getattr(ctrl, "Kd", prof.get("pid_kd", 0)))
                return f"Kp={kp}," f"Ki={ki}," f"Kd={kd}"
            else:
                return f"type={ctype}"
        except Exception as e:
            logging.debug(f"构造控制器参数摘要失败: {e}")
        return ""

    def _set_heater_power(self, power, target):
        if self.heater is None or self._open_loop_control is None:
            return
        self._open_loop_control.set_output(power, target)

    def _switch_to_open_loop_control(self):
        if self.heater is None:
            return

        class OpenLoopControl:
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
        采样回调 - 采集 22 维全量字段样本

        字段模块：
        1. 基础控制：time, temperature, target, pwm, power_watts, phase, experiment
        2. 控制器元信息：control_type, control_params
        3. EKF 隐状态与 3σ 协方差：ekf_T_h, ekf_T_b, ekf_T_s, ekf_P_h, ekf_P_b, ekf_P_s
        4. 物理热损耗分解：loss_ambient, loss_cold, loss_radiation, loss_filament
        5. 外部扰动与求解器：v_f, fan_speed, timing_total, pgd_iterations
        """
        if not self.is_collecting:
            return

        temp = self._get_sensor_temp()
        pwm = self._get_pwm_value()
        target = (
            getattr(self.heater, "target_temp", 0.0) if self.heater else 0.0
        )

        control_type = self._detect_control_type()
        control_params = self._build_control_params_summary(eventtime)

        sample = {
            "time": eventtime,
            "temperature": temp,
            "target": target,
            "pwm": pwm,
            "power_watts": pwm * self.max_heater_power,
            "phase": self.current_phase,
            "experiment": self.current_experiment or "",
            "control_type": control_type,
            "control_params": control_params,
            "ekf_T_h": temp,
            "ekf_T_b": temp,
            "ekf_T_s": temp,
            "ekf_P_h": 0.0,
            "ekf_P_b": 0.0,
            "ekf_P_s": 0.0,
            "loss_ambient": 0.0,
            "loss_cold": 0.0,
            "loss_radiation": 0.0,
            "loss_filament": 0.0,
            "v_f": 0.0,
            "fan_speed": 0.0,
            "timing_total": 0.0,
            "pgd_iterations": 0,
        }

        control = getattr(self.heater, "control", None)
        if control is not None and hasattr(control, "get_status"):
            try:
                st = control.get_status(eventtime)
                if isinstance(st, dict):
                    sample["ekf_T_h"] = st.get(
                        "ekf_T_h", st.get("temp_heater", temp)
                    )
                    sample["ekf_T_b"] = st.get(
                        "ekf_T_b", st.get("temp_block", temp)
                    )
                    sample["ekf_T_s"] = st.get(
                        "ekf_T_s", st.get("temp_sensor", temp)
                    )

                    ekf_p = st.get("ekf_P_diag", [0.0, 0.0, 0.0])
                    if isinstance(ekf_p, (list, tuple)) and len(ekf_p) >= 3:
                        sample["ekf_P_h"] = ekf_p[0]
                        sample["ekf_P_b"] = ekf_p[1]
                        sample["ekf_P_s"] = ekf_p[2]

                    sample["loss_ambient"] = st.get("loss_ambient", 0.0)
                    sample["loss_cold"] = st.get("loss_cold", 0.0)
                    sample["loss_radiation"] = st.get("loss_radiation", 0.0)
                    sample["loss_filament"] = st.get("loss_filament", 0.0)

                    sample["v_f"] = st.get("extrude_speed", st.get("v_f", 0.0))

                    sample["timing_total"] = st.get("timing_total", 0.0)
                    sample["pgd_iterations"] = st.get("pgd_iterations", 0)
            except Exception as e:
                logging.debug(f"提取控制器扩展状态失败: {e}")

        try:
            fan_obj = self.printer.lookup_object("fan", None)
            if fan_obj is not None and hasattr(fan_obj, "get_status"):
                fan_st = fan_obj.get_status(eventtime)
                if isinstance(fan_st, dict) and "speed" in fan_st:
                    sample["fan_speed"] = fan_st["speed"] * 100.0
        except Exception:
            pass

        # 采集额外配置的独立物理传感器温度（不干扰控制逻辑，仅记录）
        for sensor_name in self.extra_sensor_names:
            try:
                s_obj = self.printer.lookup_object(sensor_name, None)
                if s_obj is not None and hasattr(s_obj, "get_temp"):
                    sample[sensor_name] = s_obj.get_temp(eventtime)[0]
                else:
                    sample[sensor_name] = 0.0
            except Exception:
                sample[sensor_name] = 0.0

        with self.collection_lock:
            self.data_buffer.append(sample)

    def _start_collection(self, experiment_name=None):
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
        self._sample_callback(eventtime)
        return eventtime + (1.0 / self.default_sample_rate)

    def _stop_collection(self):
        self.is_collecting = False
        if self.sample_timer is not None:
            self.reactor.unregister_timer(self.sample_timer)
            self.sample_timer = None

    def _save_data_to_csv(self, filename):
        """
        将 22 维全量字段保存到 CSV 文件
        """
        if not self.data_buffer:
            return False

        filepath = os.path.join(self.data_dir, filename)
        try:
            with open(filepath, "w", newline="") as csvfile:
                fieldnames = [
                    "time",
                    "temperature",
                    "target",
                    "pwm",
                    "power_watts",
                    "phase",
                    "experiment",
                    "control_type",
                    "control_params",
                    "ekf_T_h",
                    "ekf_T_b",
                    "ekf_T_s",
                    "ekf_P_h",
                    "ekf_P_b",
                    "ekf_P_s",
                    "loss_ambient",
                    "loss_cold",
                    "loss_radiation",
                    "loss_filament",
                    "v_f",
                    "fan_speed",
                    "timing_total",
                    "pgd_iterations",
                ]
                for sensor_name in self.extra_sensor_names:
                    if sensor_name not in fieldnames:
                        fieldnames.append(sensor_name)
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

                start_time = self.data_buffer[0]["time"]
                for sample in self.data_buffer:
                    row = {
                        "time": sample["time"] - start_time,
                        "temperature": sample["temperature"],
                        "target": sample["target"],
                        "pwm": sample["pwm"],
                        "power_watts": sample["power_watts"],
                        "phase": sample.get("phase", "heating"),
                        "experiment": sample.get("experiment", ""),
                        "control_type": sample.get("control_type", "unknown"),
                        "control_params": sample.get("control_params", ""),
                        "ekf_T_h": sample.get("ekf_T_h", sample["temperature"]),
                        "ekf_T_b": sample.get("ekf_T_b", sample["temperature"]),
                        "ekf_T_s": sample.get("ekf_T_s", sample["temperature"]),
                        "ekf_P_h": sample.get("ekf_P_h", 0.0),
                        "ekf_P_b": sample.get("ekf_P_b", 0.0),
                        "ekf_P_s": sample.get("ekf_P_s", 0.0),
                        "loss_ambient": sample.get("loss_ambient", 0.0),
                        "loss_cold": sample.get("loss_cold", 0.0),
                        "loss_radiation": sample.get("loss_radiation", 0.0),
                        "loss_filament": sample.get("loss_filament", 0.0),
                        "v_f": sample.get("v_f", 0.0),
                        "fan_speed": sample.get("fan_speed", 0.0),
                        "timing_total": sample.get("timing_total", 0.0),
                        "pgd_iterations": sample.get("pgd_iterations", 0),
                    }
                    for sensor_name in self.extra_sensor_names:
                        row[sensor_name] = sample.get(sensor_name, 0.0)
                    writer.writerow(row)
            return True
        except Exception as e:
            logging.error("保存CSV数据失败: %s", e)
            return False

    # =========================================================================
    # G-code 命令处理方法 (G-code Command Handlers)
    # =========================================================================

    cmd_TEMP_DATA_COLLECT_help = "启动温度数据采集（22维全量字段）"

    def cmd_TEMP_DATA_COLLECT(self, gcmd):
        """
        TEMP_DATA_COLLECT [HEATER=<加热器>] [SAMPLE_RATE=<Hz>] [EXPERIMENT=<实验名>] [TARGET=<目标温度>] [SPEED=<挤出速度>]
        """
        heater_name = gcmd.get("HEATER", "extruder")
        sample_rate = gcmd.get_float(
            "SAMPLE_RATE", self.default_sample_rate, above=0.0
        )
        experiment_name = gcmd.get("EXPERIMENT", "manual_collection")
        target_temp = gcmd.get_float("TARGET", None)
        extrude_speed = gcmd.get_float("SPEED", None)

        try:
            self.heater = self._get_heater(heater_name)
            self.heater_name = heater_name
        except Exception as e:
            raise gcmd.error(f"未找到加热器 '{heater_name}': {e}")

        self.default_sample_rate = sample_rate
        self.current_target_temp = target_temp
        self.current_extrude_speed = extrude_speed

        ctype = self._detect_control_type()
        if self._start_collection(experiment_name):
            gcmd.respond_info(
                f"已启动 '{heater_name}' 数据采集，"
                f"采样率 {sample_rate} Hz，实验: {experiment_name}，"
                f"控制器: {ctype}"
            )
        else:
            gcmd.respond_info("数据采集已在进行中。")

    cmd_TEMP_DATA_STOP_help = (
        "停止温度数据采集并保存到文件（自动带有控制器、温度、速度后缀）"
    )

    def cmd_TEMP_DATA_STOP(self, gcmd):
        """
        TEMP_DATA_STOP [FILENAME=<文件名>]
        """
        ctype = self._detect_control_type()
        exp = self.current_experiment or "manual"
        ts = time.strftime("%Y%m%d_%H%M%S")

        target_str = ""
        if getattr(self, "current_target_temp", None) is not None:
            t_val = self.current_target_temp
            target_str = f"_{int(t_val)}C" if t_val == int(t_val) else f"_{t_val}C"

        speed_str = ""
        if getattr(self, "current_extrude_speed", None) is not None:
            s_val = self.current_extrude_speed
            speed_str = f"_{int(s_val)}mms" if s_val == int(s_val) else f"_{s_val}mms"

        if exp == "step_response":
            default_name = f"step_response_{ctype}{target_str}_{ts}.csv"
        elif exp == "extrusion_disturbance":
            default_name = f"extrusion_disturbance_{ctype}{target_str}{speed_str}_{ts}.csv"
        else:
            default_name = f"temp_data_{ctype}_{exp}{target_str}{speed_str}_{ts}.csv"

        filename = gcmd.get("FILENAME", default_name)

        self._stop_collection()

        if self._save_data_to_csv(filename):
            filepath = os.path.join(self.data_dir, filename)
            gcmd.respond_info(
                f"数据采集已停止。控制器[{ctype}] "
                f"已保存 {len(self.data_buffer)} 个样本到 {filepath}"
            )
        else:
            gcmd.respond_info("无数据可保存或保存失败。")

        with self.collection_lock:
            self.data_buffer = []
            self.current_experiment = None
            self.current_target_temp = None
            self.current_extrude_speed = None

    cmd_TEMP_DATA_STATUS_help = "获取温度数据采集状态"

    def cmd_TEMP_DATA_STATUS(self, gcmd):
        status = "采集中" if self.is_collecting else "空闲"
        experiment = self.current_experiment or "无"
        samples = len(self.data_buffer)
        ctype = self._detect_control_type()

        gcmd.respond_info(
            f"温度数据采集器状态:\n"
            f"  状态: {status}\n"
            f"  当前实验: {experiment}\n"
            f"  已采集样本: {samples}\n"
            f"  采样率: {self.default_sample_rate} Hz\n"
            f"  当前控制器: {ctype}\n"
            f"  数据目录: {self.data_dir}"
        )

    cmd_STEADY_STATE_CALIBRATE_help = "运行稳态阶梯响应校准实验"

    def cmd_STEADY_STATE_CALIBRATE(self, gcmd):
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
                    f"平均温度={result['avg_temp']:.2f}°C, "
                    f"平均功率={result['avg_power']:.2f}W"
                )

            if cooling_enabled:
                gcmd.respond_info("开始冷却数据采集阶段...")
                cooling_data = self._run_cooling_phase(
                    cooling_mode, cooling_duration, cooling_target_temp, gcmd
                )
        finally:
            self._stop_collection()
            self._save_data_to_csv(filename)

        if results:
            results_filename = filename.replace(".csv", "_results.csv")
            self._save_steady_state_results(results, results_filename)
            gcmd.respond_info(
                f"稳态阶梯响应校准完成。\n"
                f"  数据已保存到 {filename}\n"
                f"  结果摘要已保存到 {results_filename}"
            )

        if cooling_data:
            cooling_filename = filename.replace(".csv", "_cooling.csv")
            self._save_cooling_data(cooling_data, cooling_filename)
            gcmd.respond_info(f"  冷却数据已保存到 {cooling_filename}")

    def _run_steady_state_measurement(
        self,
        target_temp,
        stability_duration,
        gcmd,
        tolerance=1.0,
        extrude_speed=0.0,
        extrude_duration=0.0,
    ):
        self.heater.set_temp(target_temp)
        stable_start = None
        measurement_start = self.reactor.monotonic()
        max_wait = 1800.0
        while True:
            current_temp = self._get_sensor_temp()
            elapsed = self.reactor.monotonic() - measurement_start

            if abs(current_temp - target_temp) < tolerance:
                if stable_start is None:
                    stable_start = self.reactor.monotonic()
                elif (
                    self.reactor.monotonic() - stable_start
                    >= stability_duration
                ):
                    break
            else:
                stable_start = None

            if elapsed > max_wait:
                gcmd.respond_info(
                    f"警告: 温度 {target_temp}°C 未在 {max_wait}s 内稳定，"
                    f"使用当前数据继续。"
                )
                break

            self.reactor.pause(self.reactor.monotonic() + 0.5)

        samples_in_window = []
        window_start = (
            stable_start
            if stable_start is not None
            else self.reactor.monotonic() - 10.0
        )
        for sample in self.data_buffer:
            if sample["time"] >= window_start:
                samples_in_window.append(sample)

        if not samples_in_window:
            return {
                "target_temp": target_temp,
                "avg_temp": current_temp,
                "avg_power": 0.0,
            }

        avg_temp = sum(s["temperature"] for s in samples_in_window) / len(
            samples_in_window
        )
        avg_power = sum(s["power_watts"] for s in samples_in_window) / len(
            samples_in_window
        )

        return {
            "target_temp": target_temp,
            "avg_temp": avg_temp,
            "avg_power": avg_power,
        }

    def _run_cooling_phase(self, mode, duration, target_temp, gcmd):
        self.heater.set_temp(0)
        self.current_phase = "cooling"
        cooling_data = []
        start_time = self.reactor.monotonic()

        if mode == "duration":
            end_condition = (
                lambda: (self.reactor.monotonic() - start_time) >= duration
            )
        else:
            end_condition = lambda: self._get_sensor_temp() <= target_temp

        while not end_condition():
            temp = self._get_sensor_temp()
            pwm = self._get_pwm_value()
            cooling_data.append(
                {
                    "time": self.reactor.monotonic() - start_time,
                    "temperature": temp,
                    "pwm": pwm,
                    "power_watts": pwm * self.max_heater_power,
                }
            )
            self.reactor.pause(self.reactor.monotonic() + 1.0)

        return cooling_data

    def _save_steady_state_results(self, results, filename):
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

    def _save_cooling_data(self, cooling_data, filename):
        filepath = os.path.join(self.data_dir, filename)
        try:
            with open(filepath, "w", newline="") as csvfile:
                fieldnames = ["time", "temperature", "pwm", "power_watts"]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for sample in cooling_data:
                    writer.writerow(sample)
        except Exception as e:
            logging.error("保存冷却结果失败: %s", e)

    cmd_THERMAL_ID_CALIBRATE_help = "运行综合热参数辨识实验"

    def cmd_THERMAL_ID_CALIBRATE(self, gcmd):
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
                            f"STEP3_TEMP 温度值必须大于0: {temp_str}"
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
            f"预计总时长: {total_duration:.0f}秒 "
            f"({total_duration / 60:.1f}分钟)\n"
            f"采样率: {sample_rate} Hz\n"
            f"最高温度限制: {max_temp}°C\n"
            f"\n阶段配置:\n"
            f"  阶段1: 开环阶跃响应 {step1_duration:.0f}秒 "
            f"(满功率升温至{step1_target_temp}°C)\n"
            f"  阶段2: PRBS动态激励 {step2_duration:.0f}秒 "
            f"({prbs_low_power * 100:.0f}%-{prbs_high_power * 100:.0f}%功率)\n"
            f"  阶段3: 稳态实验 {len(step3_temps)}个温度点 "
            f"({', '.join([f'{t:.0f}°C' for t in step3_temps])})\n"
            f"         每个温度点稳定后采集{step3_stable_duration:.0f}秒\n"
            f"  阶段4: 冷却阶段 {step4_duration:.0f}秒 "
            f"(关闭加热，记录温度衰减)"
        )

        self.default_sample_rate = sample_rate
        self._start_collection("thermal_identification")

        try:
            experiment_start = self.reactor.monotonic()

            gcmd.respond_info("\n" + "=" * 50)
            gcmd.respond_info(
                f"阶段1: 开环阶跃响应 - 满功率升温至{step1_target_temp}°C"
            )
            gcmd.respond_info("=" * 50)
            self.current_phase = "open_loop_step"
            self._run_open_loop_step(
                step1_target_temp,
                step1_duration,
                max_temp,
                gcmd,
            )

            gcmd.respond_info("\n" + "=" * 50)
            gcmd.respond_info(f"阶段2: PRBS动态激励 {step2_duration}秒")
            gcmd.respond_info("=" * 50)
            self.current_phase = "prbs"
            self._run_prbs_experiment(
                duration=step2_duration,
                min_pulse=0.2,
                max_pulse=10.0,
                power_levels=[prbs_low_power, prbs_high_power],
                max_temp=max_temp,
                gcmd=gcmd,
                adaptive_power=True,
                resume_pulse=True,
            )

            gcmd.respond_info("\n" + "=" * 50)
            gcmd.respond_info(f"阶段3: 稳态实验 {len(step3_temps)}个温度点")
            gcmd.respond_info("=" * 50)
            self.current_phase = "steady_state"
            for temp_target in step3_temps:
                gcmd.respond_info(f"加热至 {temp_target}°C...")
                self._run_steady_state_measurement(
                    temp_target,
                    step3_stable_duration,
                    gcmd,
                    tolerance=1.0,
                )

            gcmd.respond_info("\n" + "=" * 50)
            gcmd.respond_info(f"阶段4: 冷却阶段 {step4_duration}秒")
            gcmd.respond_info("=" * 50)
            self.current_phase = "cooling"
            self._run_cooling_phase("duration", step4_duration, 30.0, gcmd)

        finally:
            self._stop_collection()
            self._save_data_to_csv(filename)

        gcmd.respond_info(
            f"综合热参数辨识实验完成。\n  数据已保存到 {filename}"
        )

    def _run_open_loop_step(self, target_temp, max_duration, max_temp, gcmd):
        self._switch_to_open_loop_control()
        try:
            self._set_heater_power(1.0, target_temp)
            start_time = self.reactor.monotonic()
            while True:
                current_temp = self._get_sensor_temp()
                elapsed = self.reactor.monotonic() - start_time
                if current_temp >= target_temp or elapsed >= max_duration:
                    break
                if current_temp >= max_temp:
                    gcmd.respond_info(
                        f"警告: 温度 {current_temp:.1f}°C "
                        f"超过最高限制 {max_temp}°C，停止加热。"
                    )
                    break
                self.reactor.pause(self.reactor.monotonic() + 0.5)
        finally:
            self._set_heater_power(0.0, 0.0)
            self._restore_original_control()

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
        sequence = self._generate_prbs_sequence(
            duration, min_pulse, max_pulse, power_levels
        )
        self._switch_to_open_loop_control()
        try:
            start_time = self.reactor.monotonic()
            for power, pulse_duration in sequence:
                current_temp = self._get_sensor_temp()
                if current_temp >= max_temp:
                    gcmd.respond_info(
                        f"警告: 温度 {current_temp:.1f}°C "
                        f"超过最高限制 {max_temp}°C，暂停PRBS。"
                    )
                    self._set_heater_power(0.0, 0.0)
                    while self._get_sensor_temp() >= max_temp - 20.0:
                        self.reactor.pause(self.reactor.monotonic() + 1.0)
                self._set_heater_power(power, power * self.max_heater_power)
                self.reactor.pause(self.reactor.monotonic() + pulse_duration)
        finally:
            self._set_heater_power(0.0, 0.0)
            self._restore_original_control()

        return {
            "duration": duration,
            "protection_time": 0.0,
            "effective_duration": duration,
        }

    cmd_PRBS_CALIBRATE_help = "运行PRBS动态激励校准实验"

    def cmd_PRBS_CALIBRATE(self, gcmd):
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
        sequence = []
        total_time = 0.0

        while total_time < duration:
            pulse_duration = random.uniform(min_pulse, max_pulse)
            pulse_duration = min(pulse_duration, duration - total_time)
            pulse_power = random.choice(powers)
            sequence.append((pulse_power, pulse_duration))
            total_time += pulse_duration

        return sequence

    def get_status(self, eventtime):
        return {
            "is_collecting": self.is_collecting,
            "current_experiment": self.current_experiment or "",
            "current_phase": self.current_phase,
            "samples_collected": len(self.data_buffer),
            "sample_rate": self.default_sample_rate,
            "data_directory": self.data_dir,
            "control_type": self._detect_control_type(),
        }


# =============================================================================
# 热参数估计器类
# Thermal Parameter Estimator Class
# =============================================================================


class ThermalParameterEstimator:
    """
    热参数估计器类

    支持从采集的稳态数据估计线性散热系数 K_lin 和辐射系数 K_rad。
    """

    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")

        self.gcode.register_command(
            "ESTIMATE_THERMAL_PARAMS",
            self.cmd_ESTIMATE_THERMAL_PARAMS,
            desc=self.cmd_ESTIMATE_THERMAL_PARAMS_help,
        )

    cmd_ESTIMATE_THERMAL_PARAMS_help = "从采集数据估计热参数"

    def cmd_ESTIMATE_THERMAL_PARAMS(self, gcmd):
        data_file = gcmd.get("DATA_FILE")
        method = gcmd.get("METHOD", "steady_state")

        if method == "steady_state":
            self._estimate_from_steady_state(data_file, gcmd)
        elif method == "prbs":
            self._estimate_from_prbs(data_file, gcmd)
        else:
            raise gcmd.error(f"未知的估计方法: {method}")

    def _estimate_from_steady_state(self, data_file, gcmd):
        try:
            results_file = data_file.replace(".csv", "_results.csv")
            results = self._load_csv(results_file)
        except Exception as e:
            raise gcmd.error(f"加载数据文件失败: {e}")

        if not results:
            raise gcmd.error("数据文件中未找到稳态结果")

        temps = [float(r["avg_temp"]) for r in results]
        powers = [float(r["avg_power"]) for r in results]

        k_lin, k_rad = self._fit_heat_loss_model(temps, powers)

        gcmd.respond_info(
            f"热参数估计结果:\n"
            f"  线性散热系数 (K_lin): {k_lin:.6f} W/K\n"
            f"  辐射系数 (K_rad): {k_rad:.9f} W/K^4\n"
            f"  热损耗模型: "
            f"P_loss = K_lin × (T - T_amb) + K_rad × (T⁴ - T_amb⁴)"
        )

    def _fit_heat_loss_model(self, temps, powers):
        n = len(temps)
        if n < 2:
            return 0.0, 0.0

        T_amb = AMBIENT_TEMP + 273.15
        T_kelvin = [t + 273.15 for t in temps]

        sum_x1 = sum(T - T_amb for T in T_kelvin)
        sum_x2 = sum(T**4 - T_amb**4 for T in T_kelvin)
        sum_y = sum(powers)
        sum_x1x1 = sum((T - T_amb) ** 2 for T in T_kelvin)
        sum_x2x2 = sum((T**4 - T_amb**4) ** 2 for T in T_kelvin)
        sum_x1x2 = sum((T - T_amb) * (T**4 - T_amb**4) for T in T_kelvin)
        sum_x1y = sum((T_kelvin[i] - T_amb) * powers[i] for i in range(n))
        sum_x2y = sum(
            (T_kelvin[i] ** 4 - T_amb**4) * powers[i] for i in range(n)
        )

        det = sum_x1x1 * sum_x2x2 - sum_x1x2**2
        if abs(det) < 1e-15:
            return sum_y / sum_x1 if sum_x1 != 0 else 0.0, 0.0

        k_lin = (sum_x2x2 * sum_x1y - sum_x1x2 * sum_x2y) / det
        k_rad = (sum_x1x1 * sum_x2y - sum_x1x2 * sum_x1y) / det

        return k_lin, k_rad

    def _estimate_from_prbs(self, data_file, gcmd):
        gcmd.respond_info(
            f"PRBS数据分析需要外部工具（如MATLAB/Python）进行处理。\n"
            f"建议使用系统辨识工具箱对 {data_file} 进行分析。"
        )

    def _load_csv(self, filename):
        filepath = os.path.expanduser(filename)
        if not os.path.exists(filepath):
            filepath = filename
        results = []
        try:
            with open(filepath, "r", newline="") as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    results.append(row)
        except Exception as e:
            logging.error("加载CSV文件失败: %s", e)
        return results


def load_config(config):
    return TemperatureDataCollector(config)


def load_config_prefix(config):
    return TemperatureDataCollector(config)
