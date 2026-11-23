# 二维舵机云台 · 视觉追踪（OpenMV + STM32）

用 OpenMV 识别色块并算出目标偏离画面中心的偏差，经串口发给 STM32F103C8T6，
由 STM32 输出两路 50 Hz PWM 驱动双轴舵机云台，让镜头始终对准目标。

```
┌─────────┐   UART 115200         ┌──────────────┐   PWM 50Hz   ┌────────┐
│ OpenMV  │  0xFF X Y 0xFE        │ STM32F103C8T6│  PA0 / PA1   │ 云台舵机│
│ H7 Plus │ ────────────────────► │  TIM2 CH1/CH2│ ───────────► │ 双轴   │
└─────────┘                       └──────────────┘              └────────┘
```

## 一、目录结构

```
gimbal/
├── OpenMV/
│   └── gimbal_main.py      OpenMV 端：色块追踪 + 双轴独立 PID + 串口发送
└── STM32/
    ├── Hardware/
    │   ├── Servo.c/.h      TIM2 CH1(PA0)/CH2(PA1) 输出 50Hz PWM
    │   └── Serial.c/.h     USART1 115200 + 帧解析状态机
    ├── User/main.c         主程序
    ├── Library/            标准外设库
    ├── Start/              启动文件与 CMSIS
    └── Project.uvprojx     Keil 工程（已含 STM32F10X_MD 宏与 .\Hardware 路径）
```

## 二、接线

| OpenMV | STM32 | 说明 |
|---|---|---|
| P4 (TX) | PA10 (USART1_RX) | OpenMV 发 → STM32 收 |
| P5 (RX) | PA9 (USART1_TX) | 预留反向通信 |
| GND | GND | **必须共地**，否则数据乱码 |
| — | PA0 (TIM2_CH1) | 1 号舵机信号 |
| — | PA1 (TIM2_CH2) | 2 号舵机信号 |
| 5V 电源 | 舵机 VCC | 建议独立 5V ≥2A，地线与 STM32 共地 |

> 舵机不要从 STM32 的 3.3V 取电；两路同时动作时电流可达 1A 以上。

## 三、通信协议

每帧 4 字节：

```
0xFF | X | Y | 0xFE
```

- `X`、`Y` 取值 0~255，128 为中间位置
- STM32 侧状态机逐字节匹配：帧头 → 两个数据字节 → 帧尾；错位会自动重新同步
- 未检测到目标时 OpenMV 发送 `0xFF 0x80 0x80 0xFE`（两轴回中）

STM32 端把 0~255 线性映射成舵机脉宽：

```
脉宽(us) = 500 + value * 2000 / 255      // 500~2500us，对应 0~180°
```

## 四、OpenMV 端关键参数

```python
green_threshold = (20, 120, -120, -10, -50, 50)   # LAB 阈值，光照变了要重取
pixels_threshold = 80                              # 噪点过滤
area_threshold   = 80

# X 轴（左右）
KP_X, KI_X, KD_X = 0.6, 0.02, 0.20
DEAD_ZONE_X = 6          # 中心死区，小于它不动，防抖
ANGLE_K_X   = 1.8 / 15   # 像素误差 → 角度 的比例，越小越灵敏

# Y 轴（上下）
KP_Y, KI_Y, KD_Y = 1.0, 0.02, 0.25
DEAD_ZONE_Y = 8
ANGLE_K_Y   = 1.8 / 66
```

阈值取法：OpenMV IDE → 工具 → Machine Vision → **Threshold Editor**，框住目标物体自动生成六个值。

## 五、编译与烧录

1. Keil MDK 打开 `STM32/Project.uvprojx`
2. 确认工程设置：Device `STM32F103C8`、Define `USE_STDPERIPH_DRIVER,STM32F10X_MD`、Include Paths 含 `.\Hardware`
3. `F7` 编译 → `F8` 烧录（ST-Link，SWD）
4. OpenMV IDE 打开 `OpenMV/gimbal_main.py` → 运行

上电后两个舵机先回中，OpenMV 检测到目标后云台跟随；STM32 收到数据时 PC13 灯亮。

## 六、方向与轴向修正

STM32 `User/main.c` 顶部：

```c
#define AXIS_SWAP   0   /* 1 = 交换两轴（PA0 用 Y，PA1 用 X） */
#define INVERT_X    0   /* 1 = 水平轴反向 */
#define INVERT_Y    0   /* 1 = 俯仰轴反向 */
```

OpenMV 端也可单独反向：`INVERT_X` / `INVERT_Y = True`。

## 七、调试经验（踩过的坑）

| 现象 | 原因 | 解决 |
|---|---|---|
| `TypeError: 'int' object isn't callable` | 固件 5.x 里 `blob.pixels/cx/cy/rect` 是**属性**不是方法 | 用 `_get(obj, name)` 兼容，判断 `callable` |
| `TypeError: object 'int' isn't a tuple or list` | 固件 5.x 画图函数要求传**元组** | `draw_cross((cx, cy))`；并用 try/except 保证画图失败不影响控制 |
| `AttributeError: time.clock` | 新固件不推荐 `time.clock()` | 改用帧内计算或直接去掉 FPS |
| 舵机不转 | 一半是**供电不足**，一半是信号线接错 | 独立 5V ≥2A、GND 共地、信号线对 PA0/PA1 |
| 云台某轴迟钝 | 两轴机械惯量不同，底层舵机带整个上层质量 | 两轴 PID 参数独立；该轴 `ANGLE_K` 调小（更灵敏） |
| 静止时轻微抖动 | 死区太小 / Kd 过大 | 加大 `DEAD_ZONE`，或减小 `KD` |
| 颜色一直 `not found` | 阈值与光照不匹配 | Threshold Editor 重取；先放宽 L 范围 |
