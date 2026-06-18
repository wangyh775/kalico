# MPC V2 模型预测控制算法使用文档

## 目录

1. [算法基本原理概述](#1-算法基本原理概述)
2. [环境依赖与安装步骤](#2-环境依赖与安装步骤)
3. [初始化配置参数说明](#3-初始化配置参数说明)
4. [核心API接口详解](#4-核心api接口详解)
5. [典型使用场景示例代码](#5-典型使用场景示例代码)
6. [常见问题解决方案及调试指南](#6-常见问题解决方案及调试指南)

---

## 1. 算法基本原理概述

### 1.1 什么是模型预测控制（MPC）

模型预测控制（Model Predictive Control，MPC）是一种先进的温度控制方法，与传统PID控制相比具有显著优势。MPC利用系统模型来预测加热头的温度变化，并主动调整加热功率以实现精确的温度控制。

### 1.2 MPC V2 的核心特点

MPC V2 是基于**三节点热力学模型**的改进版本，相比原版MPC具有以下特点：

| 特性 | 描述 |
|------|------|
| **三节点模型** | 加热器(Heater)、加热块(Block)、传感器(Sensor)三个热节点 |
| **等效参数** | 使用系统辨识得到的等效参数(theta_1 ~ theta_9) |
| **预测优化** | 基于预测时域和控制时域的滚动优化 |
| **Numba加速** | 可选的Numba JIT编译加速，显著提升计算性能 |
| **实时监控** | 完整的时间统计和性能监控 |

### 1.3 三节点热力学模型

```
┌─────────────────────────────────────────────────────────────────┐
│                        三节点热力学模型                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    ┌──────────┐    θ₂·(T_h-T_b)    ┌──────────┐    θ₃·(T_b-T_s)    ┌──────────┐
│    │  加热器   │ ─────────────────▶ │  加热块   │ ─────────────────▶ │  传感器   │
│    │   T_h    │                     │   T_b    │                     │   T_s    │
│    └──────────┘                     └──────────┘                     └──────────┘
│         │                                │                                │
│         │ θ₁·(T_h-T_b)                   │                                │
│         │                                │                                │
│         ▼                                ▼                                │
│    ┌──────────┐                    ┌──────────┐                         │
│    │  热回流   │                    │  热损耗   │                         │
│    └──────────┘                    └──────────┘                         │
│                                          │                             │
│                    ┌─────────────────────┼─────────────────────┐       │
│                    │                     │                     │       │
│                    ▼                     ▼                     ▼       │
│              ┌──────────┐         ┌──────────┐         ┌──────────┐   │
│              │ 对流损耗  │         │ 传导损耗  │         │ 辐射损耗  │   │
│              │ θ₅·(T_b-T_env)│     │ θ₆·(T_b-T_cold)│   │ θ₇·(T_b⁴-T_env⁴)│ │
│              └──────────┘         └──────────┘         └──────────┘   │
│                                                                 │
│    功率输入: P × θ₈                                              │
│    挤出损耗: v_f × c_p × θ₉ × (T_b - T_filament)                 │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 1.4 数学模型

#### 温度动态方程

**加热器温度变化率：**
```
dT_h/dt = θ₈ × P - θ₁ × (T_h - T_b)
```

**加热块温度变化率：**
```
dT_b/dt = q_hb - q_bs - P_conv - P_cond - P_rad - P_extrusion

其中：
  q_hb = θ₂ × (T_h - T_b)           # 加热器到加热块的热传递
  q_bs = θ₃ × (T_b - T_s)           # 加热块到传感器的热传递
  P_conv = θ₅ × (T_b - T_env)       # 对流损耗
  P_cond = θ₆ × (T_b - T_cold)      # 传导损耗（到冷端）
  P_rad = θ₇ × (T_b⁴ - T_env⁴)      # 辐射损耗（斯特藩-玻尔兹曼定律）
  P_extrusion = v_f × c_p × θ₉ × (T_b - T_filament)  # 挤出损耗
```

**传感器温度变化率：**
```
dT_s/dt = θ₄ × (T_b - T_s)
```

#### MPC优化目标函数

```
J = w_t × Σ(T_s - T_target)² + w_c × Σu² + w_r × Σ(Δu)²

其中：
  w_t : 跟踪误差权重
  w_c : 控制量权重
  w_r : 控制变化率权重
  u   : 控制量（功率）
```

### 1.5 MPC控制流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                        MPC控制周期流程                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────┐                                                    │
│  │ 1. 获取状态  │ ← 读取传感器温度、挤出速率、环境温度等               │
│  └──────┬──────┘                                                    │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐                                                    │
│  │ 2. 模型预测  │ ← 使用上一步功率预测当前温度                        │
│  └──────┬──────┘                                                    │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐                                                    │
│  │ 3. 状态校正  │ ← 使用实际测量值校正模型状态                        │
│  └──────┬──────┘                                                    │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐           │
│  │ 4. 轨迹预测  │ ──▶ │ 5. 目标计算  │ ──▶ │ 6. 优化求解  │           │
│  └─────────────┘     └─────────────┘     └──────┬──────┘           │
│                                                   │                  │
│                                                   ▼                  │
│                                            ┌─────────────┐           │
│                                            │ 7. 输出控制  │           │
│                                            └─────────────┘           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. 环境依赖与安装步骤

### 2.1 系统要求

| 项目 | 要求 |
|------|------|
| Python版本 | Python 3.7+ |
| NumPy | 必需 |
| SciPy | 必需（优化求解） |
| Numba | 可选（性能加速） |

### 2.2 依赖安装

```bash
# 基本依赖
pip install numpy scipy

# 可选：Numba加速（推荐）
pip install numba
```

### 2.3 验证安装

```python
# 在Kalico环境中验证
python -c "import numpy; import scipy; print('基本依赖OK')"
python -c "import numba; print('Numba加速可用')" 2>/dev/null || echo "Numba未安装，将使用纯Python版本"
```

---

## 3. 初始化配置参数说明

### 3.1 基本配置示例

```ini
[extruder]
# 控制算法选择
control: mpc_v2

# 加热器功率（必需）
heater_power: 40

# 系统辨识参数（使用默认值或自行标定）
theta_1: 0.05029312
theta_2: 0.2806417
theta_3: 0.01065468
theta_4: 0.1370236
theta_5: 0.003195262
theta_6: 0.02327857
theta_7: 0.00000000002571527
theta_8: 0.08000314

# 耗材参数
filament_diameter: 1.75
filament_density: 1.20
filament_heat_capacity: 1.8

# 可选：环境温度传感器
# ambient_temp_sensor: temperature_sensor my_sensor

# 可选：冷端温度传感器
# cold_temp_sensor: temperature_sensor cold_sensor

# 可选：冷却风扇
# cooling_fan: fan
```

### 3.2 参数详细说明

#### 3.2.1 必需参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `control` | string | 控制算法类型，设为 `mpc_v2` |
| `heater_power` | float | 加热器标称功率（瓦特） |

#### 3.2.2 系统辨识参数（theta参数）

| 参数 | 默认值 | 物理意义 |
|------|--------|----------|
| `theta_1` | 5.029312e-02 | 加热器到加热块的热传递系数 |
| `theta_2` | 2.806417e-01 | 加热器到加热块的热流系数 |
| `theta_3` | 1.065468e-02 | 加热块到传感器的热传递系数 |
| `theta_4` | 1.370236e-01 | 传感器响应系数 |
| `theta_5` | 3.195262e-03 | 对流损耗系数 |
| `theta_6` | 2.327857e-02 | 传导损耗系数 |
| `theta_7` | 2.571527e-11 | 辐射损耗系数（斯特藩-玻尔兹曼相关） |
| `theta_8` | 8.000314e-02 | 功率转换系数 |

> **注意**: `theta_9` 由系统自动计算：`theta_9 = theta_8 × theta_2 / theta_1`

#### 3.2.3 耗材参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `filament_diameter` | 1.75 | 耗材直径（mm） |
| `filament_density` | 1.20 | 耗材密度（g/cm³） |
| `filament_heat_capacity` | 1.8 | 耗材比热容（J/g/K） |
| `filament_temperature_source` | "ambient" | 耗材温度来源 |

**耗材温度来源选项：**
- `ambient`: 使用环境温度（默认）
- `sensor`: 使用指定传感器温度
- 数值: 使用固定温度值

#### 3.2.4 模型参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `smoothing` | 0.83 | 状态校正平滑系数（秒） |
| `min_ambient_change` | 1.0 | 最小环境温度变化率（°C/s） |
| `steady_state_rate` | 0.5 | 稳态判定变化率（°C/s） |
| `maximum_retract` | 2.0 | 最大回抽量（mm） |

#### 3.2.5 MPC优化参数（代码内部默认值）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `prediction_horizon` | 30 | 预测时域（步数） |
| `control_horizon` | 10 | 控制时域（步数） |
| `weight_tracking` | 10.0 | 跟踪误差权重 |
| `weight_control` | 0.001 | 控制量权重 |
| `weight_rate` | 0.1 | 控制变化率权重 |
| `max_iterations` | 200 | 最大优化迭代次数 |
| `tolerance` | 1e-5 | 优化收敛容差 |

### 3.3 常见耗材材料参数参考

| 材料 | 密度 (g/cm³) | 比热容 (J/g/K) |
|------|-------------|----------------|
| PLA | 1.25 | 1.8 - 2.2 |
| PETG | 1.27 | 1.7 - 2.2 |
| ABS | 1.06 | 1.25 - 2.4 |
| ASA | 1.07 | 1.3 - 2.1 |
| PA (尼龙) | 1.15 | 2.0 - 2.5 |
| PC | 1.20 | 1.1 - 1.9 |
| TPU | 1.21 | 1.5 - 2.0 |

---

## 4. 核心API接口详解

### 4.1 GCode命令

#### 4.1.1 MPC_SET_V2 - 设置MPC参数

```
MPC_SET_V2 [参数名=值] ...
```

**可用参数：**

| 参数 | 说明 |
|------|------|
| `FILAMENT_DIAMETER` | 耗材直径（mm） |
| `FILAMENT_DENSITY` | 耗材密度（g/cm³） |
| `FILAMENT_HEAT_CAPACITY` | 耗材比热容（J/g/K） |
| `THETA_1` ~ `THETA_8` | 系统辨识参数 |
| `FILAMENT_TEMP` | 耗材温度来源（"sensor"/"ambient"/数值） |

**示例：**
```gcode
; 设置ASA耗材参数
MPC_SET_V2 FILAMENT_DENSITY=1.07 FILAMENT_HEAT_CAPACITY=1.7

; 设置耗材温度来源为传感器
MPC_SET_V2 FILAMENT_TEMP=sensor

; 设置耗材温度为固定值
MPC_SET_V2 FILAMENT_TEMP=25
```

### 4.2 状态查询接口

#### 4.2.1 通过HTTP API查询

```
http://<打印机IP>:7125/printer/objects/query?extruder
```

**返回的状态信息：**

```json
{
  "temp_heater": 220.5,
  "temp_block": 219.8,
  "temp_sensor": 220.0,
  "temp_ambient": 25.3,
  "temp_cold": 28.1,
  "power": 15.2,
  "loss_ambient": 0.62,
  "loss_filament": 0.15,
  "loss_cold": 0.45,
  "loss_radiation": 0.08,
  "theta_1": 0.05029312,
  "theta_2": 0.2806417,
  "prediction_horizon": 30,
  "control_horizon": 10,
  "timing_model_step": 0.05,
  "timing_predict": 0.8,
  "timing_optimize": 2.5,
  "timing_total": 3.2,
  "timing_max": 5.1,
  "timing_avg": 3.0,
  "numba_enabled": true
}
```

#### 4.2.2 状态字段说明

| 字段 | 说明 |
|------|------|
| `temp_heater` | 模型估计的加热器温度（°C） |
| `temp_block` | 模型估计的加热块温度（°C） |
| `temp_sensor` | 模型估计的传感器温度（°C） |
| `temp_ambient` | 估计的环境温度（°C） |
| `temp_cold` | 冷端温度（°C） |
| `power` | 当前输出功率（W） |
| `loss_ambient` | 对流损耗（W） |
| `loss_filament` | 挤出损耗（W） |
| `loss_cold` | 传导损耗（W） |
| `loss_radiation` | 辐射损耗（W） |
| `timing_model_step` | 单步模型预测时间（ms） |
| `timing_predict` | 轨迹预测时间（ms） |
| `timing_optimize` | 优化求解时间（ms） |
| `timing_total` | 总控制周期时间（ms） |
| `timing_max` | 最大周期时间（ms） |
| `timing_avg` | 平均周期时间（ms） |
| `numba_enabled` | Numba加速是否启用 |

### 4.3 Python API

#### 4.3.1 ControlMPCV2 类

```python
class ControlMPCV2:
    """MPC V2控制器类"""
    
    def temperature_update(self, read_time, temp, target_temp):
        """
        温度更新主循环
        
        参数:
            read_time: 当前时间 (s)
            temp: 当前测量温度 (°C)
            target_temp: 目标温度 (°C)
        """
        pass
    
    def get_status(self, eventtime):
        """
        获取控制器状态
        
        返回:
            dict: 状态信息字典
        """
        pass
    
    def _model_step_v2(self, T_h, T_b, T_s, power, dt, T_env, T_cold, v_f, T_filament):
        """
        单步模型仿真
        
        返回:
            (T_h_new, T_b_new, T_s_new): 更新后的温度元组
        """
        pass
    
    def _predict_trajectory_v2(self, initial_state, power_sequence, dt, ...):
        """
        多步轨迹预测
        
        返回:
            (T_h_arr, T_b_arr, T_s_arr): 温度轨迹数组
        """
        pass
    
    def _solve_mpc_v2(self, initial_state, setpoint, dt, ...):
        """
        MPC优化求解
        
        返回:
            float: 最优控制量（功率 W）
        """
        pass
```

---

## 5. 典型使用场景示例代码

### 5.1 基本配置示例

```ini
# printer.cfg

[extruder]
step_pin: PF0
dir_pin: PF1
...
# MPC V2 控制配置
control: mpc_v2
heater_power: 40

# 使用默认系统辨识参数
# 或根据实际测量填写

# PLA耗材默认参数
filament_diameter: 1.75
filament_density: 1.25
filament_heat_capacity: 2.0
```

### 5.2 多材料切换宏

```ini
[gcode_macro SET_FILAMENT_MPC]
description: 根据材料类型设置MPC参数
gcode:
    {% set material = params.MATERIAL|upper %}
    
    # PLA
    {% if material == "PLA" %}
        MPC_SET_V2 FILAMENT_DENSITY=1.25 FILAMENT_HEAT_CAPACITY=2.0
    
    # PETG
    {% elif material == "PETG" %}
        MPC_SET_V2 FILAMENT_DENSITY=1.27 FILAMENT_HEAT_CAPACITY=2.0
    
    # ABS
    {% elif material == "ABS" %}
        MPC_SET_V2 FILAMENT_DENSITY=1.06 FILAMENT_HEAT_CAPACITY=2.0
    
    # ASA
    {% elif material == "ASA" %}
        MPC_SET_V2 FILAMENT_DENSITY=1.07 FILAMENT_HEAT_CAPACITY=1.7
    
    # TPU
    {% elif material == "TPU" %}
        MPC_SET_V2 FILAMENT_DENSITY=1.21 FILAMENT_HEAT_CAPACITY=1.8
    
    {% else %}
        {action_respond_info("未知材料类型，使用默认参数")}
    {% endif %}
```

### 5.3 打印开始宏集成

```ini
[gcode_macro PRINT_START]
gcode:
    {% set bed_temp = params.BED_TEMP|default(60)|float %}
    {% set extruder_temp = params.EXTRUDER_TEMP|default(200)|float %}
    {% set material = params.MATERIAL|default("PLA") %}
    
    # 设置MPC耗材参数
    SET_FILAMENT_MPC MATERIAL={material}
    
    # 预热
    M140 S{bed_temp}
    M104 S{extruder_temp}
    
    # 等待温度
    M190 S{bed_temp}
    M109 S{extruder_temp}
    
    # 开始打印...
```

### 5.4 性能监控宏

```ini
[gcode_macro MPC_STATUS]
description: 显示MPC性能统计
gcode:
    {% set stats = printer.extruder %}
    
    {action_respond_info("=== MPC V2 状态 ===")}
    {action_respond_info("加热器温度: %.2f°C" % stats.temp_heater)}
    {action_respond_info("加热块温度: %.2f°C" % stats.temp_block)}
    {action_respond_info("传感器温度: %.2f°C" % stats.temp_sensor)}
    {action_respond_info("环境温度: %.2f°C" % stats.temp_ambient)}
    {action_respond_info("输出功率: %.2f W" % stats.power)}
    {action_respond_info("--- 热损耗 ---")}
    {action_respond_info("对流损耗: %.3f W" % stats.loss_ambient)}
    {action_respond_info("传导损耗: %.3f W" % stats.loss_cold)}
    {action_respond_info("辐射损耗: %.3f W" % stats.loss_radiation)}
    {action_respond_info("挤出损耗: %.3f W" % stats.loss_filament)}
    {action_respond_info("--- 性能统计 ---")}
    {action_respond_info("模型步进: %.3f ms" % stats.timing_model_step)}
    {action_respond_info("轨迹预测: %.3f ms" % stats.timing_predict)}
    {action_respond_info("优化求解: %.3f ms" % stats.timing_optimize)}
    {action_respond_info("总周期: %.3f ms" % stats.timing_total)}
    {action_respond_info("最大周期: %.3f ms" % stats.timing_max)}
    {action_respond_info("平均周期: %.3f ms" % stats.timing_avg)}
    {action_respond_info("Numba加速: %s" % ("启用" if stats.numba_enabled else "未启用"))}
```

### 5.5 高级配置示例（带环境传感器）

```ini
# 配置环境温度传感器
[temperature_sensor chamber]
sensor_type: NTC 100K beta 3950
sensor_pin: ...

[temperature_sensor cold_end]
sensor_type: NTC 100K beta 3950
sensor_pin: ...

[extruder]
control: mpc_v2
heater_power: 50

# 系统辨识参数（需要实际标定）
theta_1: 0.048
theta_2: 0.265
theta_3: 0.012
theta_4: 0.145
theta_5: 0.0035
theta_6: 0.025
theta_7: 0.000000000028
theta_8: 0.085

# 传感器配置
ambient_temp_sensor: temperature_sensor chamber
cold_temp_sensor: temperature_sensor cold_end

# 冷端默认温度
T_cold: 30

# 冷却风扇
cooling_fan: fan
```

---

## 6. 常见问题解决方案及调试指南

### 6.1 常见问题

#### 问题1：温度震荡

**症状：** 温度在目标值附近来回波动

**可能原因：**
- theta参数不准确
- 权重参数设置不当

**解决方案：**
```ini
# 增加控制变化率权重，减少控制量权重
# 需要修改代码中的默认值或通过GCode设置
```

#### 问题2：温度响应慢

**症状：** 温度变化缓慢，跟不上目标温度

**可能原因：**
- heater_power设置过低
- theta参数偏小

**解决方案：**
```ini
# 确认加热器功率设置正确
heater_power: 50  # 实际测量或查看规格

# 或调整theta参数
```

#### 问题3：温度超调

**症状：** 加热时温度超过目标值

**可能原因：**
- 预测时域太短
- 跟踪权重过高

**解决方案：**
- 增加预测时域（需要修改代码）
- 降低跟踪权重

#### 问题4：Numba未启用

**症状：** `numba_enabled: False`

**解决方案：**
```bash
# 安装Numba
pip install numba

# 重启Kalico
```

### 6.2 调试指南

#### 6.2.1 启用详细日志

在 `printer.cfg` 中添加：

```ini
[logger]
level: DEBUG
```

#### 6.2.2 实时监控

使用HTTP API实时查看状态：

```bash
# 查询MPC状态
curl http://localhost:7125/printer/objects/query?extruder

# 持续监控（需要jq工具）
watch -n 1 'curl -s http://localhost:7125/printer/objects/query?extruder | jq .'
```

#### 6.2.3 性能分析

```gcode
; 在打印过程中执行
MPC_STATUS

; 查看输出
; timing_total 应该 < 10ms
; timing_optimize 应该 < 5ms
```

### 6.3 参数调优建议

#### 6.3.1 Theta参数标定

Theta参数需要通过系统辨识获得，建议：

1. 使用原版MPC校准获取初始参数
2. 通过实验数据拟合优化
3. 记录不同工况下的参数

#### 6.3.2 权重参数调优

| 现象 | 调整建议 |
|------|----------|
| 温度跟踪慢 | 增加 `weight_tracking` |
| 控制量过大 | 增加 `weight_control` |
| 控制震荡 | 增加 `weight_rate` |
| 响应迟钝 | 减少 `weight_rate` |

### 6.4 故障排除流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                        故障排除流程                                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  问题发生                                                           │
│      │                                                              │
│      ▼                                                              │
│  ┌─────────────┐                                                    │
│  │ 检查配置    │ ← heater_power是否正确？                            │
│  └──────┬──────┘                                                    │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐                                                    │
│  │ 检查参数    │ ← theta参数是否已设置？耗材参数是否正确？            │
│  └──────┬──────┘                                                    │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐                                                    │
│  │ 检查传感器  │ ← 温度传感器是否正常？环境传感器是否可用？           │
│  └──────┬──────┘                                                    │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐                                                    │
│  │ 检查性能    │ ← timing_total是否过高？Numba是否启用？             │
│  └──────┬──────┘                                                    │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐                                                    │
│  │ 查看日志    │ ← 是否有错误或警告信息？                            │
│  └─────────────┘                                                    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 6.5 性能优化建议

1. **启用Numba加速**
   ```bash
   pip install numba
   ```

2. **调整预测时域**
   - 较短的预测时域计算更快，但可能影响控制质量
   - 默认30步通常足够

3. **调整控制时域**
   - 较短的控制时域计算更快
   - 默认10步是合理的平衡

4. **监控计算时间**
   - `timing_total` 应小于控制周期（通常100ms）
   - 如果超时，考虑减少预测时域

---

## 附录

### A. 参数快速参考表

| 参数类别 | 参数名 | 默认值 | 单位 |
|----------|--------|--------|------|
| 系统 | control | mpc_v2 | - |
| 系统 | heater_power | 必需 | W |
| 模型 | theta_1 | 0.0503 | - |
| 模型 | theta_2 | 0.2806 | - |
| 模型 | theta_3 | 0.0107 | - |
| 模型 | theta_4 | 0.1370 | - |
| 模型 | theta_5 | 0.0032 | - |
| 模型 | theta_6 | 0.0233 | - |
| 模型 | theta_7 | 2.57e-11 | - |
| 模型 | theta_8 | 0.0800 | - |
| 耗材 | filament_diameter | 1.75 | mm |
| 耗材 | filament_density | 1.20 | g/cm³ |
| 耗材 | filament_heat_capacity | 1.8 | J/g/K |
| MPC | prediction_horizon | 30 | 步 |
| MPC | control_horizon | 10 | 步 |
| MPC | weight_tracking | 10.0 | - |
| MPC | weight_control | 0.001 | - |
| MPC | weight_rate | 0.1 | - |

### B. 版本历史

| 版本 | 日期 | 变更说明 |
|------|------|----------|
| 1.0 | 2024 | 初始版本，基于三节点热力学模型 |

### C. 相关文档

- [MPC.md](../MPC.md) - 原版MPC文档
- [PID.md](../PID.md) - PID控制文档
- [Config_Reference.md](../Config_Reference.md) - 配置参考
