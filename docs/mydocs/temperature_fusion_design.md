# 温度传感器数据融合模块设计文档

## 概述

`temperature_fusion` 是 Kalico 的一个虚拟温度传感器模块,用于将多条 Modbus 总线上的多个通道温度读数融合为一个合成温度,输出给加热器控制回路使用。

**典型场景**:加热仓部署 12 个温度传感器分布在不同热区,通过加权融合得到一个稳定、可信的仓温代表值,供 `[heater_generic chamber]` 的 PID 控制器使用。

**模块形态**:`sensor_type: temperature_fusion` 工厂注册(与 [temperature_combined](../../klippy/extras/temperature_combined.py) 同构),不是独立命名节。

**输入来源**:仅支持直接读取 [modbus_temperature](../../klippy/extras/modbus_temperature.py) 总线(通过 `modbus_temperature_manager`)。不通过 `sensor_list` 间接引用其他命名温度对象。

## 1. 架构总览

### 数据流

```
[modbus_temperature] 节                [temperature_fusion] 节 (sensor_type 工厂)
  ┌──────────────────┐                ┌──────────────────────────────┐
  │ ModbusBus        │  ←─lookup──    │ PrinterSensorFusion          │
  │  serial_path     │                │   └─ modbus_channels: [...]  │
  │  slave_id        │                │   └─ weights / zones / ...    │
  │  scale / signed  │                │                              │
  │  channel_count   │   ─read_all──▶ │   _sample_timer()            │
  └──────────────────┘                │     ├─ batch read_registers  │
                                      │     ├─ decode + outlier      │
                                      │     ├─ fusion_strategy.fuse  │
                                      │     └─ temperature_callback  │
                                      └──────────────────────────────┘
                                                    │
                                                    ▼
                                         [heater_generic chamber]
                                         sensor_type: temperature_fusion
                                         (后续仓温加热器模块消费)
```

### 文件与注册点

| 作用 | 路径 |
|---|---|
| 模块主文件 | `klippy/extras/temperature_fusion.py` |
| 工厂自动加载 stub | `klippy/extras/temperature_sensors.cfg`(新增 `[temperature_fusion]` 一行) |
| 策略注册表 | 模块内部维护 `_fusion_strategies` dict,暴露 `register_fusion_strategy()` 函数 |

### 核心类结构

```
PrinterSensorFusion                # 主类,实现 4 个传感器协议方法
├── __init__(config)               # 读配置、注册 connect/ready 事件
├── _handle_connect()              # klippy:connect 事件:lookup modbus_temperature_manager + 取 bus
├── _handle_ready()                # klippy:ready 事件:启动 reactor 定时器(错开初值)
├── _sample_timer(eventtime)       # 批量读 modbus → 融合 → 推送 → 越界检查
├── setup_minmax / setup_callback / get_report_time_delta / get_temp
├── get_status(eventtime)          # API 状态
└── _strategy: FusionStrategy      # 当前策略实例

FusionStrategy (抽象基类)
├── update(samples: list[SensorSample], eventtime)   # 每周期调用,可维护内部状态
├── fuse() -> FusionResult                          # 返回融合温度 + 置信度
└── get_diagnostics() -> dict                       # 策略内部状态用于 status

# 内置 4 个策略
WeightedMeanStrategy                # 默认:加权平均 + MAD 离群剔除
LayeredWeightedMeanStrategy         # 分层加权平均(MAD 剔除继承自 weighted_mean)
RobustMedianStrategy                # 鲁棒中位数 + IQR 离群剔除
KalmanFusionStrategy               # 状态空间融合 + 方差估计

# 策略注册 API(供用户自定义)
register_fusion_strategy(name, factory_class)   # 模块级函数
```

### 依赖关系

- **强依赖**:`modbus_temperature_manager`(必须先配置 `[modbus_temperature]` 节)
- **强依赖**:`heaters` 工厂注册机制
- **弱依赖**:`danger_options`(`temp_ignore_limits` 开关)

### SensorSample 数据结构

```python
@dataclass
class SensorSample:
    channel: int              # modbus 通道号
    raw_value: int            # 原始寄存器值
    temperature: float        # 解码后温度(°C)
    weight: float             # 该通道权重
    zone: Optional[str]       # 区域标签(可选)
    position: Optional[tuple] # 空间坐标 (x,y,z)(可选,预留)
    is_valid: bool            # 是否通过有效性检查
    timestamp: float          # 采样时间戳
```

### FusionResult 数据结构

```python
@dataclass
class FusionResult:
    temperature: float        # 融合温度
    confidence: float         # 0.0-1.0 置信度
    valid_samples: int        # 本次有效样本数
    excluded_samples: list    # 被剔除的通道及原因
```

## 2. 配置接口

### 使用方式

fusion 模块作为 `sensor_type` 工厂注册。用法分两层:

**第 1 层:必须先有一条 modbus bus**

```ini
[modbus_temperature]
serial_path: /dev/ttyUSB0
baudrate: 9600
slave_id: 1
register_start: 0
channel_count: 16
scale: 0.1
signed: True
func_code: 3
report_time: 1.0
```

**第 2 层:heater/sensor 声明使用 fusion**

```ini
[heater_generic chamber]
heater_pin: ...
sensor_type: temperature_fusion
modbus_bus: default               # 可选,单 bus 时省略
modbus_channels: 0,1,2,3,4,5,6,7,8,9,10,11
weights: 1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0
zones: inlet,outlet,bed,bed,side,side,top,top,bottom,bottom,front,rear
fusion_strategy: weighted_mean     # 默认值
min_temp: 0
max_temp: 100
```

或仅做监控(无 heater):

```ini
[temperature_sensor chamber_fused]
sensor_type: temperature_fusion
modbus_channels: 0,1,2,3,4,5,6,7,8,9,10,11
fusion_strategy: kalman
```

### 配置参数表

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `modbus_bus` | str | `None` | modbus 总线名。省略时按 `modbus_temperature_manager.get_bus()` 规则取默认 bus(仅 1 条时自动选)。 |
| `modbus_channels` | list[int] | **必填** | modbus 通道号列表,0-based,每个 `< channel_count`。长度 N 决定融合集大小(典型 12)。 |
| `weights` | list[float] | 全 1.0 | 每个通道的静态权重,长度必须等于 `modbus_channels`。用于加权平均/中位数等策略。 |
| `zones` | list[str] | 空 | 每个通道的区域标签(可选)。仅诊断/状态上报用,不影响融合算法本身(算法只看 weights)。 |
| `positions` | list[tuple] | 空 | 每个通道的空间坐标 `[x0,y0,z0, x1,y1,z1, ...]`。预留给未来空间加权策略,当前不强制使用。 |
| `noise_variance` | list[float] | 空 | 每个通道的测量噪声方差。Kalman 策略必填;其他策略忽略。 |
| `fusion_strategy` | str | `weighted_mean` | 融合策略名。内置:`weighted_mean` / `layered_weighted_mean` / `robust_median` / `kalman`。自定义策略通过 `register_fusion_strategy()` 注册。 |
| `layer_assignment` | list[int] | 空 | 仅 `layered_weighted_mean` 策略使用。长度必须等于 `modbus_channels`,值是层索引。把每个通道映射到所属物理层。 |
| `layer_weights` | list[float] | 空 | 仅 `layered_weighted_mean` 策略使用。每层一个权重,长度 = 层数 = `max(layer_assignment)+1`。设了 `layer_assignment` 时必填。 |
| `fusion_*` | any | — | 任意 `fusion_` 前缀字段透传给策略类,由策略类自己解析。如 `fusion_outlier_zscore: 3.0`、`fusion_q: 0.01`、`fusion_r: 0.1`。 |
| `report_time` | float | 1.0 | 采样周期(秒)。每个周期读一次 modbus 并更新融合值。最小 0.3。 |
| `min_temp` | float | 0 K | 融合温度下限。超出触发 `invoke_shutdown`(受 `temp_ignore_limits` 影响)。 |
| `max_temp` | float | 99999999.9 | 融合温度上限。 |
| `maximum_deviation` | float | 999.9 | 任意两个有效样本间最大允许差值。超出触发 `invoke_shutdown`。设大值可禁用。 |
| `gcode_id` | str | None | M105 上报时显示的 ID 字符。 |

### 校验规则(在 `__init__` 中强制)

1. `modbus_channels` 长度 ≥ 1
2. `weights` 若提供,长度必须等于 `modbus_channels` 长度,否则 `config.error`
3. `zones` / `positions` / `noise_variance` 若提供,长度同上
4. `fusion_strategy` 必须已注册,否则 `config.error("temperature_fusion: unknown strategy 'xxx'; available: ...")`
5. `min_temp < max_temp`
6. 每个通道号 `0 ≤ ch < bus.channel_count`(延迟到 `_handle_connect` 才能校验,因为 bus 此时才能 lookup)
7. `layered_weighted_mean` 策略专属:
   - `layer_assignment` 若提供,长度必须等于 `modbus_channels`,且无负索引
   - `layer_assignment` 非空时 `layer_weights` 必填
   - `max(layer_assignment) < len(layer_weights)`(所有层索引都能在 `layer_weights` 中找到)

### `fusion_` 前缀透传规则

策略类可声明 `STRATEGY_CONFIG_KEYS = ["outlier_zscore", "q", "r"]`。模块主类 `__init__` 扫描所有 `fusion_<key>` 字段,构造 dict 传给策略 `__init__(strategy_config)`。这样策略类不必直接访问 `config` 对象,保持解耦。

### 配置最小化示例

```ini
[modbus_temperature]
serial_path: /dev/ttyUSB0
baudrate: 9600

[temperature_sensor chamber_fused]
sensor_type: temperature_fusion
modbus_channels: 0,1,2,3,4,5,6,7,8,9,10,11
fusion_strategy: weighted_mean
maximum_deviation: 8
min_temp: 0
max_temp: 80
```

## 3. 融合策略详细设计

### 策略基类接口

```python
class FusionStrategy:
    """所有融合策略的抽象基类。"""

    # 策略可声明的配置字段名(不含 fusion_ 前缀)
    STRATEGY_CONFIG_KEYS: list[str] = []

    def __init__(self, strategy_config: dict, num_channels: int):
        """主类在 __init__ 时构造策略实例,传入解析后的配置 dict
        和通道数。子类在此读取自己的参数。"""
        self.num_channels = num_channels

    def update(self, samples: list[SensorSample], eventtime: float):
        """每个采样周期调用一次。samples 已经过基础有效性过滤
        (disconnected_raw / NaN 已剔除)。策略可在此维护内部
        状态(如滑动窗、协方差)。"""
        raise NotImplementedError

    def fuse(self) -> FusionResult:
        """返回当前融合结果。update() 之后立即调用。"""
        raise NotImplementedError

    def get_diagnostics(self) -> dict:
        """返回策略内部状态,供 get_status() 暴露。可空。"""
        return {}
```

### 内置策略 1:`weighted_mean`(默认)

**算法**:加权平均 + MAD 离群剔除

**步骤**:

1. 收到 N 个 `SensorSample`(已剔除无效通道)
2. 计算加权平均 $\bar{T} = \frac{\sum w_i T_i}{\sum w_i}$
3. 计算偏差 $d_i = |T_i - \bar{T}|$
4. 计算 MAD(中位绝对偏差)$M = \text{median}(d_i)$
5. 计算修正 Z 分数 $Z_i = \frac{0.6745 \cdot d_i}{M}$(当 $M=0$ 时所有 $Z_i=0$)
6. 剔除 $Z_i > Z_{\text{threshold}}$ 的样本(默认 $Z_{\text{threshold}} = 3.5$)
7. 用剩余样本重新计算加权平均作为输出
8. 置信度按第 3 节"置信度统一公式"计算

**配置字段**:

| 字段 | 默认 | 说明 |
|---|---|---|
| `fusion_outlier_zscore` | 3.5 | 修正 Z 分数阈值,超过则剔除。设大值可禁用剔除。 |

**状态**:无状态策略(每次 `update` 都独立计算)

**诊断输出**:`{"excluded": [...], "mad": ..., "weighted_mean": ..., "z_scores": [...]}`

### 内置策略 2:`layered_weighted_mean`

**算法**:分层加权平均 + MAD 离群剔除(继承自 `weighted_mean`)

**适用场景**:腔体水平面四角对称、同一高度层 4 个传感器物理等价的多层布局(典型 3 层 × 4 角 = 12 传感器)。把 12 个 per-channel 权重简化为 3 个 per-layer 权重,配置更短、语义更清晰,且支持层内一致性诊断。

**步骤**:

1. `__init__` 阶段:`_expand_weights()` 把 `layer_weights` 按 `layer_assignment` 展开成 per-channel 权重列表
2. 每周期 `update()`:
   - 把展开后的权重覆盖到每个 `SensorSample.weight`
   - 调用 `super().update()`(即 `WeightedMeanStrategy.update`),执行加权平均 + MAD + 修正 Z-score 剔除
   - 计算每层均值(用于诊断):按 `layer_assignment` 分组,对未被剔除的样本求平均
3. `fuse()` 完全继承父类,置信度公式不变

**权重展开规则**:

$$ w_i = W_{\text{layer\_assignment}[i]} $$

例:`layer_assignment = [0,0,0,0, 1,1,1,1, 2,2,2,2]`、`layer_weights = [1.0, 1.5, 0.8]` 展开为 `[1.0,1.0,1.0,1.0, 1.5,1.5,1.5,1.5, 0.8,0.8,0.8,0.8]`,等价于显式写 `weights`。

**配置字段**:

| 字段 | 默认 | 说明 |
|---|---|---|
| `fusion_outlier_zscore` | 3.5 | 修正 Z 分数阈值,继承自 `weighted_mean`。 |
| `layer_assignment` | 空 | 通道到层索引的映射,长度 = `modbus_channels`。 |
| `layer_weights` | 空 | 每层一个权重,长度 = `max(layer_assignment)+1`。 |

**与 `weights` 的关系**:两者可同时配置,但 `layer_weights` 会在 `update()` 里覆盖 `weights`。语义上 `weights` 被忽略——和 `kalman` 策略忽略 `weights` 一样,是策略自身的职责。建议只用 `layer_weights`,不配 `weights`。

**状态**:无状态策略(权重展开在 `__init__` 一次完成,运行时只读)

**诊断输出**:`{"excluded": [...], "mad": ..., "weighted_mean": ..., "z_scores": [...], "layer_weights": [...], "layer_means": [...], "layer_counts": [...]}`

- `layer_means`:每层未被剔除样本的平均温度。用于追踪竖直温度梯度和检测层内故障
- `layer_counts`:每层未被剔除的样本数。某层 count < 4 表明该层有传感器被剔除

**配置示例**:

```ini
[heater_generic chamber]
sensor_type: temperature_fusion
modbus_channels: 0,1,2,3,4,5,6,7,8,9,10,11
fusion_strategy: layered_weighted_mean
# 通道0-3=底层,4-7=中层,8-11=顶层
layer_assignment: 0,0,0,0,1,1,1,1,2,2,2,2
layer_weights: 1.0,1.5,0.8
fusion_outlier_zscore: 5.0
maximum_deviation: 200.0
min_temp: 0
max_temp: 300
```

### 内置策略 3:`robust_median`

**算法**:加权中位数 + IQR 离群剔除

**步骤**:

1. 收到 N 个样本
2. 按 IQR(四分位距)剔除离群点:
   - $Q_1, Q_3$ = 25 / 75 百分位
   - $\text{IQR} = Q_3 - Q_1$
   - 剔除 $T_i < Q_1 - 1.5 \cdot \text{IQR}$ 或 $T_i > Q_3 + 1.5 \cdot \text{IQR}$ 的样本
3. 对剩余样本按温度排序,找累计权重首次超过 $\frac{\sum w_i}{2}$ 的样本作为加权中位数
4. 置信度同 `weighted_mean`

**配置字段**:

| 字段 | 默认 | 说明 |
|---|---|---|
| `fusion_iqr_multiplier` | 1.5 | IQR 剔除倍数。设大值可禁用剔除。 |

**状态**:无状态策略

**诊断输出**:`{"excluded": [...], "q1": ..., "q3": ..., "iqr": ..., "weighted_median": ...}`

### 内置策略 4:`kalman`

**算法**:常值状态卡尔曼滤波 + 多传感器顺序融合

**模型**:

- 状态向量 $\mathbf{x} = [T]$(单变量,假设腔室真实温度短时恒定)
- 状态转移 $F = 1$,过程噪声 $Q$
- 每个传感器 $i$ 的观测 $z_i = T_i$,观测噪声 $R_i$ 来自 `noise_variance`
- 每周期把 N 个观测顺序融合到同一个状态估计

**预测**:

$$\hat{x}^-_k = F \cdot \hat{x}_{k-1}$$
$$P^-_k = F \cdot P_{k-1} \cdot F^T + Q$$

**对每个有效观测 $z_i$ 顺序更新**:

$$K_i = \frac{P^-_k}{P^-_k + R_i}$$
$$\hat{x}^+_k = \hat{x}^-_k + K_i \cdot (z_i - \hat{x}^-_k)$$
$$P^+_k = (1 - K_i) \cdot P^-_k$$

**最终输出**:$\hat{x}^+_k$ 作为融合温度,$P^+_k$ 反映估计不确定度

**配置字段**:

| 字段 | 默认 | 说明 |
|---|---|---|
| `fusion_q` | 0.01 | 过程噪声方差。值越大跟踪越快但越噪声。 |
| `fusion_r_default` | 0.1 | 当 `noise_variance` 未指定时所有通道的默认观测噪声。 |
| `fusion_init_p` | 10.0 | 初始协方差。设大值让首周期快速收敛。 |

**状态**:维护 $\hat{x}_{k-1}$ 和 $P_{k-1}$,跨周期持续更新

**诊断输出**:`{"state_estimate": ..., "covariance": ..., "innovation": [...], "kalman_gains": [...]}`

**初始化**:$\hat{x}_0$ = 首个有效观测,$P_0 = \text{fusion\_init\_p}$

### 置信度统一公式

所有策略返回的 `confidence` ∈ [0, 1]:

$$\text{confidence} = \alpha \cdot \frac{n_{\text{valid}}}{n_{\text{total}}} + (1 - \alpha) \cdot \text{consistency}$$

其中:

- $\alpha = 0.7$(有效性权重更高)
- $\text{consistency} = \max(0, 1 - \frac{\text{MAD or IQR}}{T_{\text{range}}})$,反映样本间一致性
- Kalman 策略特殊:$\text{consistency} = \max(0, 1 - \frac{P^+_k}{P_0})$,用协方差反映不确定度

### 自定义策略注册

用户在自己的 extras 模块中:

```python
# klippy/extras/my_fusion_strategy.py
from .temperature_fusion import FusionStrategy, register_fusion_strategy

class MyStrategy(FusionStrategy):
    STRATEGY_CONFIG_KEYS = ["threshold", "window_size"]

    def __init__(self, strategy_config, num_channels):
        super().__init__(strategy_config, num_channels)
        self.threshold = strategy_config.get("threshold", 5.0)
        self.window_size = strategy_config.get("window_size", 10)
        self.window = []

    def update(self, samples, eventtime):
        self.window.append([s.temperature for s in samples])
        if len(self.window) > self.window_size:
            self.window.pop(0)

    def fuse(self):
        # ... 自定义融合逻辑
        return FusionResult(temperature=..., confidence=...,
                            valid_samples=..., excluded_samples=[...])

    def get_diagnostics(self):
        return {"window_size": len(self.window)}

def load_config(config):
    register_fusion_strategy(config, "my_strategy", MyStrategy)
    return config.get_printer()
```

用户配置:

```ini
[my_fusion_strategy]   # 触发 load_config 注册策略
# 无参数,仅注册

[temperature_sensor chamber_fused]
sensor_type: temperature_fusion
modbus_channels: 0,1,2,3,4,5,6,7,8,9,10,11
fusion_strategy: my_strategy
fusion_threshold: 5.0
fusion_window_size: 10
```

### 策略选择指引

| 场景 | 推荐 | 理由 |
|---|---|---|
| 12 个传感器均匀布置、读数稳定 | `weighted_mean` | 简单、可解释、计算开销最小 |
| 多层对称布局(3 层 × 4 角)、要按层加权 | `layered_weighted_mean` | 权重从 12 个降到 3 个,语义清晰,带层内诊断 |
| 环境存在扰动、个别传感器偶发跳变 | `robust_median` | IQR 比 MAD 更鲁棒于多离群点 |
| 高 PID 稳定性要求、噪声大 | `kalman` | 输出平滑、有不确定度反馈 |
| 多区域异构 | `weighted_mean` + 不同 weights | 不需要换策略,调权重即可 |

## 4. 数据流与生命周期

### 启动时序

参考 `temperature_combined.py` 的两阶段延迟绑定模式:

```
1. heaters.load_config()
   └─ 读 temperature_sensors.cfg
       └─ [temperature_fusion] stub 节
           └─ load_config() 注册 sensor_factory
               pheaters.add_sensor_factory("temperature_fusion", PrinterSensorFusion)

2. 用户配置解析
   └─ [heater_generic chamber]
       sensor_type: temperature_fusion
       └─ heaters.setup_sensor(config)
           └─ PrinterSensorFusion.__init__(config)
               ├─ 读 modbus_channels/weights/zones/...
               ├─ 创建策略实例(仅传配置,不访问硬件)
               ├─ register_event_handler("klippy:connect", _handle_connect)
               └─ register_event_handler("klippy:ready", _handle_ready)
               【此时不 lookup modbus_manager,因为可能尚未加载】

3. klippy:connect 事件(所有对象已 __init__ 完成)
   └─ _handle_connect()
       ├─ manager = printer.lookup_object("modbus_temperature_manager")
       │   若 None → config.error("temperature_fusion: requires [modbus_temperature] section")
       ├─ bus = manager.get_bus(modbus_bus_name, config)
       │   失败 → config.error 列出可用 bus
       ├─ 校验每个 modbus_channel < bus.channel_count
       ├─ 复用 bus.scale / bus.signed / bus.disconnected_raw 作为解码参数
       └─ 缓存 bus 引用,但不打开串口

4. klippy:ready 事件(所有 connect 钩子已完成)
   └─ _handle_ready()
       ├─ reactor.update_timer(self._sample_timer, reactor.NOW + 1.0)
       │   延迟 1 秒避免与其他传感器定时器冲突
       └─ 标记 self._ready = True

5. 首次 _sample_timer(eventtime) 触发
   └─ 读取 → 融合 → 推送 → 设定下次时间
```

### 单周期数据流

`_sample_timer(eventtime)` 是核心循环,每个 `report_time` 周期触发一次:

```
┌─────────────────────────────────────────────────────────────────────┐
│ 1. 批量读取 modbus                                                   │
│    serial = bus.open_serial()                                        │
│    regs = serial.read_registers(                                    │
│        bus.slave_id,                                                │
│        bus.register_start,                                          │
│        bus.channel_count,    # 一次读所有通道(与单通道同开销)       │
│        bus.func_code                                                 │
│    )                                                                 │
│    失败 → invoke_shutdown                                           │
├─────────────────────────────────────────────────────────────────────┤
│ 2. 解码每个目标通道                                                  │
│    for ch in self.modbus_channels:                                  │
│        raw = regs[ch]                                                │
│        if raw == bus.disconnected_raw:                              │
│            invoke_shutdown(...)  # 严格"一坏即停机"                  │
│            return eventtime + report_time                           │
│        if bus.signed and raw & 0x8000:                              │
│            raw -= 0x10000                                           │
│        temp = raw * bus.scale                                       │
│        构造 SensorSample(...)                                        │
├─────────────────────────────────────────────────────────────────────┤
│ 3. 基础有效性过滤                                                    │
│    - NaN / Inf 检测 → invoke_shutdown                               │
│    - 这里不进行离群剔除(留给策略)                                    │
├─────────────────────────────────────────────────────────────────────┤
│ 4. 全局一致性检查                                                    │
│    if maximum_deviation < 999.0:                                    │
│        range = max(temps) - min(temps)                              │
│        if range > maximum_deviation:                                │
│            invoke_shutdown("deviation ...")                         │
│            return eventtime + report_time                           │
│    (一坏即停机:任何两传感器偏差过大就停机)                           │
├─────────────────────────────────────────────────────────────────────┤
│ 5. 策略更新与融合                                                    │
│    try:                                                             │
│        self._strategy.update(valid_samples, eventtime)              │
│        result = self._strategy.fuse()                               │
│    except Exception as e:                                           │
│        invoke_shutdown("strategy raised: ...")                      │
├─────────────────────────────────────────────────────────────────────┤
│ 6. 越界检查                                                          │
│    if (result.temperature < min_temp or                            │
│        result.temperature > max_temp) and                          │
│       not get_danger_options().temp_ignore_limits:                 │
│        invoke_shutdown("Fusion temp ... outside ...")               │
├─────────────────────────────────────────────────────────────────────┤
│ 7. 缓存与推送                                                        │
│    self.last_temp = result.temperature                              │
│    self.last_confidence = result.confidence                         │
│    self.last_result = result                                        │
│    mcu = printer.lookup_object("mcu")                                │
│    self._callback(mcu.estimated_print_time(eventtime),              │
│                   result.temperature)                               │
├─────────────────────────────────────────────────────────────────────┤
│ 8. 调度下次                                                          │
│    return eventtime + self.report_time                              │
└─────────────────────────────────────────────────────────────────────┘
```

### `--debug-output` 模式

参考 `modbus_temperature.py` 的 debug 路径:

```python
if self._is_debug:
    self.last_temp = 0.0
    if self._callback is not None:
        mcu = self.printer.lookup_object("mcu")
        now = self.reactor.monotonic()
        self._callback(mcu.estimated_print_time(now), 0.0)
    return eventtime + self.report_time
```

在 debug 模式下不访问硬件,输出 0°C,不触发停机。这是单元测试和 `--debug-output` smoke test 的关键路径。

### 协议方法实现

```python
def setup_minmax(self, min_temp, max_temp):
    self.min_temp = min_temp
    self.max_temp = max_temp

def setup_callback(self, cb):
    self._callback = cb

def get_report_time_delta(self):
    return self.report_time

def get_temp(self, eventtime):
    return (self.last_temp, 0.0)   # (current, target);fusion 无目标概念
```

### 关闭时序

fusion 模块**不**直接管理串口生命周期——串口由 `modbus_temperature_manager` 持有,fusion 仅持有 bus 引用。fusion 模块不需要实现 `__del__` 或关闭钩子,避免与 modbus 模块抢资源。

### reactor 定时器特性

- 周期 `report_time`(默认 1.0s,最小 0.3s)
- 单次执行预算应 < 100ms(一次 modbus 读 16 通道通常 ~10ms)
- modbus 读取是同步阻塞的(用 `threading.Lock` 串行化),与单通道传感器行为一致。fusion 不会比 12 个独立 `modbus_temperature` 传感器更慢,反而省了 11 次 modbus 往返。

## 5. 状态接口与诊断

### `get_status(eventtime)` 返回结构

```python
def get_status(self, eventtime):
    return {
        # === 必需字段(与 heaters 协议一致)===
        "temperature": round(self.last_temp, 2),

        # === 融合质量指标 ===
        "confidence": round(self.last_confidence, 3),
        "valid_samples": self.last_valid_count,
        "total_samples": len(self.modbus_channels),

        # === 各通道原始温度(按 modbus_channels 顺序)===
        "samples": [
            {"channel": ch, "temperature": round(t, 2),
             "weight": w, "zone": z, "valid": v}
            for ch, t, w, z, v in zip(
                self.modbus_channels,
                self.last_raw_temps,
                self.weights,
                self.zones,
                self.last_valid_flags,
            )
        ],

        # === 剔除信息 ===
        "excluded": self.last_excluded,

        # === 策略诊断(透传 strategy.get_diagnostics())===
        "strategy": self._strategy.get_diagnostics(),

        # === 运行统计 ===
        "measured_min_temp": round(self.measured_min, 2),
        "measured_max_temp": round(self.measured_max, 2),
    }
```

### 字段说明

| 字段 | 类型 | 说明 |
|---|---|---|
| `temperature` | float | 融合后温度。必需,与 temperature_sensor / heaters 协议一致。 |
| `confidence` | float | 0.0-1.0 置信度。公式见第 3 节。 |
| `valid_samples` | int | 本周期有效样本数。 |
| `total_samples` | int | 总通道数(= `len(modbus_channels)`)。 |
| `samples` | list[dict] | 每个通道的实时数据,含 channel/temperature/weight/zone/valid。便于 UI 显示热图或仪表盘。 |
| `excluded` | list[dict] | 本周期被剔除的通道列表。每项含 `channel` + `reason`(`"disconnected"` / `"outlier"` / `"nan"` 等)。 |
| `strategy` | dict | 策略内部状态。`weighted_mean` 暴露 MAD/Z 分数;`kalman` 暴露协方差/状态估计。 |
| `measured_min_temp` | float | 自启动以来融合温度最小值。 |
| `measured_max_temp` | float | 自启动以来融合温度最大值。 |

### `stats(eventtime)` 上报

```python
def stats(self, eventtime):
    return False, "temperature_fusion: temp=%.2f conf=%.2f valid=%d/%d" % (
        self.last_temp, self.last_confidence,
        self.last_valid_count, len(self.modbus_channels),
    )
```

返回 `False` 表示非活跃定时器,仅打印状态字符串到日志。

### `is_adc_faulty()` 不实现

`is_adc_faulty()` 是 `heaters.py` 的 ADC 故障检测接口,仅用于底层 ADC 传感器(thermistor / PT100)。fusion 是虚拟传感器,越界检查已在 `_sample_timer` 中通过 `invoke_shutdown` 处理,不需要此方法。

### G-code 命令

新增三个 G-code 命令用于运行时诊断:

**`TEMP_FUSION_STATUS`**

显示当前融合状态摘要:

```
TEMP_FUSION_STATUS [SENSOR=<name>]
```

输出示例:

```
temperature_fusion chamber_fused:
  fused_temp=42.5°C  confidence=0.92  valid=12/12
  strategy=weighted_mean  mad=0.3  excluded=[]
  channel temps: ch0=42.1 ch1=42.3 ch2=42.8 ...
```

**`TEMP_FUSION_LIST_STRATEGIES`**

列出已注册的策略:

```
Available fusion strategies:
  kalman                 (built-in)
  layered_weighted_mean  (built-in)
  robust_median          (built-in)
  weighted_mean          (built-in)
  my_strategy            (custom)
```

**`TEMP_FUSION_RESET`**

重置策略内部状态(如 Kalman 协方差、滑动窗):

```
TEMP_FUSION_RESET SENSOR=chamber_fused
```

对有状态策略(`kalman`)特别有用——传感器更换或维护后重新初始化。

### 实例注册表

模块级维护一个 dict:

```python
_fusion_instances = {}   # name -> PrinterSensorFusion 实例

def register_fusion_instance(name, instance):
    _fusion_instances[name] = instance

def get_fusion_instance(name):
    return _fusion_instances.get(name)
```

每个 `PrinterSensorFusion.__init__` 自动注册自己;G-code 命令通过此表查找实例。这个表是模块级的(不是 printer 级),考虑到 Klipper 单 printer 进程,这是可接受的简化。

## 6. 错误处理与测试策略

### 错误分类与响应矩阵

| 错误类型 | 检测点 | 响应 | 是否停机 | 用户可恢复 |
|---|---|---|---|---|
| **配置错误** | `__init__` / `_handle_connect` | `config.error(...)` 抛出,启动失败 | — | 改配置后重启 |
| modbus_manager 未找到 | `_handle_connect` | `config.error("temperature_fusion: requires [modbus_temperature] section")` | — | 加 `[modbus_temperature]` 节 |
| bus 名不存在 | `_handle_connect` | `config.error` 列出可用 bus | — | 改 `modbus_bus` |
| 通道号 ≥ channel_count | `_handle_connect` | `config.error` 列出有效范围 | — | 改 `modbus_channels` |
| weights/zones 长度不匹配 | `__init__` | `config.error` | — | 改配置 |
| 未知策略名 | `__init__` | `config.error` 列出已注册策略 | — | 改 `fusion_strategy` 或注册新策略 |
| **运行时错误** | `_sample_timer` | 见下文 | | |
| modbus 通信失败 | `read_registers` 抛异常 | `invoke_shutdown("...modbus read failed...")` | 是 | 检查接线和电源 |
| 单通道 disconnected_raw | 解码阶段 | `invoke_shutdown("...channel N...disconnected")` | 是 | 重新连接该通道传感器 |
| 融合温度越 min/max | 融合后 | `invoke_shutdown("...fused temp...outside...")` | 是(受 `temp_ignore_limits` 影响) | 检查传感器或调宽 min/max |
| 全局偏差超 maximum_deviation | 解码后、融合前 | `invoke_shutdown("...deviation...")` | 是 | 检查个别传感器故障 |
| NaN/Inf 解码结果 | 解码阶段 | `invoke_shutdown("...NaN/Inf...")` | 是 | modbus 寄存器异常 |
| **策略错误** | `_sample_timer` 第 5 步 | | | |
| 策略 update/fuse 抛异常 | try/except 包裹 | `invoke_shutdown("...strategy...raised...")` | 是 | 检查策略代码 |

### 关键设计点

**1. "一坏即停机"严格执行**

所有运行时错误都走 `invoke_shutdown`,无软隔离、无 backoff、无降级模式。这与 `temperature_combined.py` 的"超 deviation 即停机"语义一致,但比 `modbus_temperature.py` 的"通信失败仅 backoff"更严格。

理由:fusion 输出直接喂给 heater 控制回路,任何不确定数据都可能导致热失控。停机比错误加热安全。

**2. `temp_ignore_limits` 全局开关**

参考 `modbus_temperature.py` / `temperature_combined.py`,所有越界检查都尊重 `danger_options` 的 `temp_ignore_limits` 开关。但**仅适用于温度越界**,通信故障和 disconnected_raw 仍然停机(这些是硬件故障,不是数值越界)。

**3. 错误信息可读性**

所有 `invoke_shutdown` 消息都包含:

- 模块名 `temperature_fusion`
- 实例名(如 `chamber_fused`)
- bus 名和通道号(如适用)
- 具体数值
- 期望范围

格式示例:

```
temperature_fusion[chamber_fused]: modbus read failed on bus 'default': Modbus timeout: no response header
temperature_fusion[chamber_fused]: channel 5 on bus 'default' reports disconnected (raw=3000)
temperature_fusion[chamber_fused]: fused temp 95.3 outside 0.0:80.0
temperature_fusion[chamber_fused]: deviation 12.5 > max 8.0 between channels 2 (38.1) and 7 (50.6)
```

### 测试策略

按 `AGENTS.md` 测试工作流,分三层:

**第 1 层:单元测试(pytest)**

文件:`test/test_temperature_fusion.py`

覆盖:

- 每个策略的 `update` / `fuse` 纯函数测试(不依赖 printer/reactor)
- `WeightedMeanStrategy`:均匀分布、单离群、多离群、全部相同、空 samples
- `RobustMedianStrategy`:同上 + IQR 边界情况
- `KalmanFusionStrategy`:收敛性、协方差递减、噪声跟踪
- `SensorSample` / `FusionResult` 构造与字段
- 置信度公式边界值(0/0、1/0、12/12 等)
- 配置解析:合法/非法配置,长度匹配校验

参考 `test/test_autosave.py` 的 pytest 风格,不依赖完整 Klipper runtime。

**第 2 层:Klippy 集成测试(.test)**

目录:`test/klippy/temperature_fusion/`

覆盖端到端流程:

- `test_basic_fusion.test`:12 通道 + weighted_mean → 检查输出温度
- `test_deviation_shutdown.test`:注入离群通道 → 验证 invoke_shutdown
- `test_disconnected_shutdown.test`:注入 disconnected_raw → 验证 invoke_shutdown
- `test_kalman_strategy.test`:kalman 策略 + 多周期更新 → 检查状态收敛
- `test_custom_strategy.test`:注册自定义策略 → 验证可被加载

参考 `test/klippy/conftest.py` 的 `.test` 收集器,需要 MCU 字典文件。

**第 3 层:固件编译验证**

fusion 是纯主机模块,**不需要**固件编译验证(不涉及 `src/`)。但需要在 `test/configs/` 中加一个使用 fusion 的示例配置,确保 CI 流程能跑通。

由于 fusion 强依赖 modbus(也是纯主机模块,模拟串口),CI 跑完整集成测试有困难——`--debug-output` 模式下 modbus 不读硬件。**最可行的 CI 路径**是单元测试 + 一个 `--debug-output` 模式的 smoke test 验证启动流程不断。

### 测试辅助:MockModbusBus

为单元测试和集成测试提供 mock:

```python
class MockModbusBus:
    """模拟 ModbusBus,按通道号返回预设温度。"""
    def __init__(self, channel_count=16, scale=0.1, signed=True,
                 disconnected_raw=3000):
        self.channel_count = channel_count
        self.scale = scale
        self.signed = signed
        self.disconnected_raw = disconnected_raw
        self.slave_id = 1
        self.register_start = 0
        self.func_code = 3
        self.bus_name = "mock"
        self._temps = {}   # channel -> raw_value

    def set_channel_temp(self, channel, temp):
        self._temps[channel] = int(temp / self.scale)

    def open_serial(self):
        return self   # 自己当 serial

    def read_registers(self, slave_id, reg_addr, count, func_code):
        return [self._temps.get(ch, 0) for ch in range(count)]
```

这个 mock 可以被 `PrinterSensorFusion` 在测试中注入(通过 monkey-patch `lookup_object` 返回 mock manager)。

## 7. 文档更新清单

按 `AGENTS.md` 强制要求:

| 文档 | 更新内容 |
|---|---|
| `docs/Config_Reference.md` | 新增 `### Fused temperature sensor` 节,参数表见第 2 节。位置在 temperature_combined 节后。 |
| `docs/Status_Reference.md` | 新增 `## temperature_fusion` 节,字段表见第 5 节。 |
| `docs/G-Codes.md` | 新增 `TEMP_FUSION_STATUS` / `TEMP_FUSION_LIST_STRATEGIES` / `TEMP_FUSION_RESET` 三个命令。 |
| `docs/Config_Changes.md` | 若有破坏性变更(无——这是新模块)。可加一条"新增 temperature_fusion 模块"的 changelog 条目。 |
| `docs/mydocs/temperature_fusion_design.md` | 本设计文档。 |
| `docs/_kalico/mkdocs.yml` | 在"我的文档"导航加入本设计文档。 |
| `klippy/extras/temperature_sensors.cfg` | 新增 `[temperature_fusion]` stub 节触发工厂注册。 |

## 8. 验证步骤

按 `AGENTS.md` 的提交前检查清单:

```bash
# 1. Ruff 检查
uv run ruff check klippy/extras/temperature_fusion.py
uv run ruff format klippy/extras/temperature_fusion.py

# 2. 空白检查
./scripts/check_whitespace.sh

# 3. 单元测试
uv run pytest test/test_temperature_fusion.py -v

# 4. 集成测试(需要字典文件)
uv run pytest test/klippy -k temperature_fusion

# 5. 文档构建
cd docs/_kalico && uv run mkdocs build --strict

# 6. Pre-commit
uv run pre-commit run --files klippy/extras/temperature_fusion.py
```

## 9. 与后续仓温加热器模块的关系

`temperature_fusion` 只管"把 N 路温度熔成 1 路",不关心谁在用。仓温加热器模块(`heater_chamber` 之类)后续独立实现,它的 `sensor_type` 可以指向 `temperature_fusion`,也可以直接用 `temperature_fusion` 的输出。两者通过传感器注册协议解耦。

后续的仓温加热器模块设计要点(本设计不含):

- 保护"温度传感器"和"加热器"两个对象(用户已说明)
- 可能涉及传感器健康检查、加热器功率限制、安全 interlock 等
- 建议作为独立 extras 模块(`klippy/extras/heater_chamber.py`),通过 `sensor_type: temperature_fusion` 引用本模块的输出

## 10. 已确认的设计决策汇总

| 维度 | 决策 |
|---|---|
| 模块形态 | `sensor_type: temperature_fusion` 工厂注册(与 `temperature_combined` 同构) |
| 输入来源 | 仅 modbus 直读(通过 `modbus_temperature_manager`) |
| 物理布局 | 多热区加权(每个传感器有权重/区域/位置) |
| 融合算法 | 可插拔策略(4 内置 + 自定义) |
| 故障模型 | 一坏即停机(保持 `temperature_combined` 安全语义) |
| 配置接口 | 扁平列表式(逗号分隔) |
| 实现路线 | Kalico 惯例的注册式策略插件(`register_fusion_strategy()`) |
| numpy 依赖 | 无(Kalman 用顺序更新,纯标量运算) |
| 修正 Z 分数阈值 | 3.5(Iglewicz & Hoaglin 经典值) |
| 置信度公式 | $0.7 \cdot \text{有效比} + 0.3 \cdot \text{一致性}$ |
| debug 模式 | 输出 0°C,不触发停机 |
| 关闭钩子 | 不实现(串口由 `modbus_temperature_manager` 管理) |
| `positions` 字段 | 当前预留不用,留给未来空间加权策略 |
| G-code 命令 | `TEMP_FUSION_STATUS` / `TEMP_FUSION_LIST_STRATEGIES` / `TEMP_FUSION_RESET` |
| 模块级实例表 | 简化设计,Klipper 单进程可接受 |
