# 简化版力传感器探针（load_cell_probe_simple）

## 背景

现有的 [load_cell_probe] 模块功能完整但复杂：SOS 滤波器配置、tap 几何分析、pullback 反算接触点、tap 质量分类器等。
对于只需要"力传感器当限位开关用"的场景（例如简易调平、快速验证传感器工作），这些机制都是负担。

本文档描述一个简化版本：**ADC 读数偏离 tare 的绝对值超过阈值即视为触发**，不做几何分析，不做 pullback。

## 设计目标

1. **触发语义**：`abs(sample - tare_counts) >= trigger_counts` 即触发
2. **单位**：raw ADC counts，不需要 `counts_per_gram` 校准
3. **集成方式**：实现标准 `ProbeEndstopWrapper` 接口，注册为 `probe` 对象，[z_tilt] / [bed_mesh] / [screws_tilt_adjust] 可直接使用
4. **不动 MCU 代码**：复用现有 MCU 端 `load_cell_probe` 命令
5. **依赖 [load_cell] 模块**：复用传感器硬件初始化、collector、tare 逻辑

## 关键约束

### MCU 端触发机制不可绕过

[src/load_cell_probe.c](file:///workspace/src/load_cell_probe.c) 的 `load_cell_probe_report_sample` 强制执行：
1. 安全范围检查（`safety_counts_min/max`）
2. `counts_to_grams` 转换（用 `grams_per_count` 定点系数）
3. `sosfilt` 滤波（必经路径）
4. 与 `trigger_grams_fixed` 比较

**绕过策略**：把 SOS 滤波器配成 0 段直通（[sos_filter.c:64-66](file:///workspace/src/sos_filter.c#L64-L66) 在 `n_sections == 0` 时直接返回原值），
并用 `counts_per_gram = 2.0` 的虚拟校准让 MCU 端的 grams 比较等价于 counts 比较。

### 数学等价性证明

设用户配置阈值为 `T`（counts 单位），主机下发：
- `grams_per_count = 0.5`（Q2 定点）
- `trigger_grams = T / 2`（取整后下发，要求 T 为偶数或允许 1 counts 误差）

MCU 端计算：
```
delta = sample - tare_counts               # counts
grams_q16 = delta * grams_per_count        # Q2.29 * 缩放 = Q16.15
            = delta * 0.5 << (29-15)
            = delta << 14
trigger_grams_fixed = trigger_grams << 15
                    = (T/2) << 15
                    = T << 14
比较：abs(grams_q16) >= trigger_grams_fixed
等价：abs(delta << 14) >= T << 14
等价：abs(delta) >= T                       # 用户期望的语义
```

简化版采用 `counts_per_gram = 2.0` 作为虚拟值，主机端在 `load_cell.tare()` 后用固定系数下发，不依赖真实校准。

### 不支持的传感器

`alps_serial` 传感器通过 USB 串口直接喂样本到主机，[attach_load_cell_probe 是空操作](file:///workspace/klippy/extras/load_cell/alps_serial.py#L399-L401)，
MCU 端 `load_cell_probe` 收不到样本。简化版同样**只支持 MCU 直连的传感器**（hx71x / ads1220 / ads131m0x），
与现有 [load_cell_probe] 模块的限制一致。

## 配置示例

```ini
[load_cell]
sensor_type: hx711
... (硬件引脚、采样率等)

[load_cell_probe_simple]
# 必填：触发阈值，raw ADC counts 偏离 tare 的绝对值
trigger_counts: 500
# 可选：安全限值，超过此值报错（counts 单位，0=禁用，默认 50000）
force_safety_limit: 50000
# 可选：drift 安全限值（counts 单位，0=禁用，默认 10000）
drift_safety_limit: 10000
# 可选：tare 采样时长（秒，默认 0.1）
tare_time: 0.1
# 可选：Z 偏移（与 [probe] z_offset 语义一致，默认 0.0）
z_offset: 0.0
# 可选：probe 激活/去活 gcode（与 [probe] 一致）
#activate_gcode: ...
#deactivate_gcode: ...
```

**注意**：不需要 `counts_per_gram`、不需要 `reference_tare_counts`、不需要 `CALIBRATE_LOAD_CELL`。
但 `LOAD_CELL_TARE` 命令仍可用（来自 [load_cell] 模块）。

## 文件结构

新增单文件 `klippy/extras/load_cell_probe_simple.py`，包含以下类：

### 1. `SimpleConfigHelper`

读取配置参数，提供：
- `get_tare_samples(gcmd)` → `max(2, ceil(tare_time * sps))`
- `get_trigger_counts(gcmd)` → 用户配置的 `trigger_counts`
- `get_trigger_grams()` → `trigger_counts // 2`（下发到 MCU 的虚拟 grams 值）
- `get_grams_per_count()` → 固定返回 Q2 定点表示的 0.5
- `get_safety_limit_counts(gcmd)` / `get_drift_safety_limit(gcmd)`
- `get_reference_safety_range(gcmd)` → 基于 sensor range 的安全带
- `assert_force_safety_limit(tare_counts, gcmd)` → tare 前检查
- `get_probe_drift_range(tare_counts, gcmd)` → 探测过程的 drift 范围

### 2. `SimplePassthroughFilter`

构造一个 0 段的 `FixedPointSosFilter` + `SosFilter`：
- `max_sections = 1`（MCU 端 `oid_alloc` 不允许 size=0）
- `n_sections = 0`（运行时直通）
- 提供 `get_oid()` 和 `reset_filter()` 接口（reset 是空操作）

### 3. `SimpleMcuProbe`

精简版 `McuLoadCellProbe`，封装 MCU 端 `load_cell_probe` 命令：
- `__init__`：创建 oid、发 `config_load_cell_probe`、注册 `_build_config` 和 `_on_connect`
- `_build_config`：lookup 三个命令（`query_state` / `set_range` / `home`）
- `_on_connect`：调用 `sensor.attach_load_cell_probe(oid)`
- `set_endstop_range(tare_counts, gcmd)`：调用 `load_cell.tare()` + 下发 `set_range` 命令
- `home_start(print_time)` / `clear_home()`：与现有逻辑一致
- WATCHDOG_MAX = 3，错误码与现有模块一致

### 4. `SimplePrimitives`

精简版 `LoadCellPrimitives`，提供 tare / home_start / home_wait / probing_move：
- `tare(gcmd)`：启动 collector，采 `tare_samples` 个样本求均值，调 `assert_force_safety_limit`，调 `set_endstop_range`
- `home_start(print_time)`：**不检查 `is_calibrated()`**（简化版不需要校准），启动 trsync + MCU home
- `home_wait(home_end_time)`：等待触发，处理错误码
- `probing_move(mcu_probe, pos, speed, gcmd)`：tare → 启动 collector → 调 `PrinterHoming.probing_move`
- 不实现 tap 分析相关代码

ERROR_MAP 与现有模块一致（COMMS_TIMEOUT / SAFETY_RANGE / OVERFLOW / WATCHDOG）。

### 5. `SimpleEndstopWrapper`

实现 `ProbeEndstopWrapper` 接口，被 `PrinterProbe` 使用：
- `get_position_endstop()` → 返回 `z_offset`
- `_handle_mcu_identify` → 注册 Z 轴 stepper
- 委托 `SimplePrimitives` 实现 `home_start` / `home_wait` / `add_stepper` / `get_steppers` / `get_mcu` / `query_endstop`
- `probing_move` 直接委托 `SimplePrimitives.probing_move`（无 pullback、无 tap 分析）
- `probe_prepare` / `probe_finish` / `multi_probe_begin` / `multi_probe_end` 委托 `ProbeActivationHelper`

### 6. `load_config(config)`

组装以上组件：
1. 从 `[load_cell]` section 查找已加载的 `LoadCell` 对象
2. 构造 `SimpleConfigHelper`、`SimplePassthroughFilter`、`SimpleMcuProbe`、`SimplePrimitives`、`SimpleEndstopWrapper`
3. 用 `PrinterProbe(config, wrapper)` 创建并注册为 `probe` 对象

## 数据流

```
用户调 PROBE / BED_MESH_CALIBRATE / SCREWS_TILT_CALCULATE
       │
       ▼
PrinterProbe.run_probe() → mcu_probe.probing_move(pos, speed)
       │
       ▼
SimpleEndstopWrapper.probing_move(pos, speed)
       │
       ▼
SimplePrimitives.probing_move(mcu_probe, pos, speed, gcmd)
       │
       ├─ tare(gcmd)
       │    ├─ collector.start_collecting()
       │    ├─ collector.collect_min(tare_samples) → 求均值 → tare_counts
       │    ├─ assert_force_safety_limit(tare_counts)
       │    └─ SimpleMcuProbe.set_endstop_range(tare_counts)
       │         ├─ load_cell.tare(tare_counts)
       │         ├─ 下发 load_cell_probe_set_range(safety_min, safety_max,
       │         │                              tare, trigger_grams=T/2,
       │         │                              grams_per_count=0.5_Q2)
       │         └─ sos_filter.reset_filter()  (空操作，0 段滤波器)
       │
       ├─ collector = start_collector()
       │
       └─ PrinterHoming.probing_move(mcu_probe, pos, speed)
            └─ MCU 端：每个样本 → counts_to_grams → sosfilt(直通) → 比较
                 若 abs(delta) >= T → trsync_do_trigger(REASON_ENDSTOP_HIT)
            返回触发时的 Z 位置 epos
```

## 错误处理

| 错误码 | 触发条件 | 用户表现 |
|--------|----------|----------|
| `REASON_ENDSTOP_HIT` | 正常触发 | 返回 Z 位置 |
| `ERROR_SAFETY_RANGE` | 样本超出 `safety_counts_min/max` | 抛 `command_error`，"force exceeded drift_safety_limit" |
| `ERROR_OVERFLOW` | 定点运算溢出（理论上不会发生，因为 grams_per_count=0.5） | 抛 `command_error` |
| `ERROR_WATCHDOG` | 超过 WATCHDOG_MAX=3 个 rest_ticks 无样本 | 抛 `command_error`，"timed out waiting for sensor data" |
| tare 前安全检查失败 | tare 时静止力已超 `force_safety_limit` | 抛 `command_error`，"force of Xg exceeds force_safety_limit" |

## 不实现的功能

- tap 几何分析、tap 校验、tap 质量分类
- pullback move
- SOS 滤波器配置（drift/buzz/notch）
- `LOAD_CELL_TEST_TAP` 命令
- 主动 drift 补偿（`drift_safety_limit` 仍作为安全限值保留）
- grams 单位的触发阈值（仅支持 raw counts）
- 对 `alps_serial` 传感器的支持

## 集成点

- 文件：`klippy/extras/load_cell_probe_simple.py`
- 入口：`load_config(config)`，Klipper 自动加载 `[load_cell_probe_simple]` section
- 注册对象：`printer.add_object("probe", printer_probe)`
- 不修改 MCU 代码、不修改现有 `load_cell_probe.py`、不修改 `probe.py`
- 不修改任何文档（这是个人实验模块，参考 `docs/mydocs/` 约定）

## 验证方式

1. **静态检查**：`uv run ruff check klippy/extras/load_cell_probe_simple.py`
2. **格式化**：`uv run ruff format klippy/extras/load_cell_probe_simple.py`
3. **单元测试**：现有 pytest 套件不直接覆盖，但模块导入应无错误
4. **集成测试**：用户配置 `[load_cell]` + `[load_cell_probe_simple]` 后，运行 `PROBE` / `BED_MESH_CALIBRATE` 验证触发行为
