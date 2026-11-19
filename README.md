# Jax Balance Car —— STM32 两轮自平衡小车

基于 **STM32F103C8T6 + MPU6050 + 双霍尔编码器电机** 的两轮自平衡小车，
使用 **STM32CubeMX + HAL 库 + Keil MDK** 开发。

控制算法为串级 PID：**直立环 PD（5ms）+ 速度环 PI（6ms）+ 转向阻尼 + 倒地保护**。

---

## 一、硬件与引脚

| 功能 | 引脚 | 说明 |
|---|---|---|
| 左电机 PWM | PA2 (TIM2_CH3) | PWM 驱动 |
| 右电机 PWM | PA3 (TIM2_CH4) | PWM 驱动 |
| 左电机方向 | PA15 / PB3 | IN1 / IN2 |
| 右电机方向 | PA4 / PA5 | IN3 / IN4 |
| 左编码器 A/B | PA0 / PA1 | EXTI 双边沿软件正交解码（4 倍频） |
| 右编码器 A/B | PA6 / PA7 | TIM3 硬件编码器模式（4 倍频） |
| MPU6050 | PB10 / PB11 | 软件 I2C |
| OLED (SSD1306) | PB8 / PB9 | 硬件 I2C1，地址 0x78 |
| 控制时基 | TIM4 | 1ms 更新中断 |
| LED | PC13 | 运行指示 |
| 调试下载 | PA13 / PA14 | SWD（JTAG 已关闭，释放 PA15/PB3） |

> PWM 频率：TIM2 ARR = 7199，PSC = 0 → 72MHz / 7200 = **10 kHz**（避开人耳可闻的啸叫频段）。

---

## 二、软件结构

```
Core/Src/main.c          初始化、上电流程、OLED 显示
Core/Src/pid.c           直立环 PD / 速度环 PI / 转向阻尼 / 倒地保护
Core/Src/motor.c         双电机方向 + PWM 输出
Core/Src/encode.c        左轮 EXTI 软件解码 + 右轮 TIM3 硬件编码器
Core/Src/tim.c           TIM2=PWM, TIM3=编码器, TIM4=1ms 控制时基
Core/Src/oled.c          OLED 驱动（硬件 I2C1）
Core/Src/i2c.c           I2C1 初始化
MyCode/mpu6050.c         MPU6050 寄存器读写
MyCode/IIC.c             软件 I2C（MPU6050 用）
```

### 控制节拍

```
TIM4 更新中断（1ms）
├── 每  5ms → PID_Balance_Loop()   直立环 PD + 转向阻尼 + 倒地保护 + 电机输出
└── 每  6ms → PID_Speed_Loop()     速度环 PI（软启动、误差低通、积分限幅）
```

---

## 三、控制算法

### 1. 互补滤波求角度（5ms）

```c
Gyro_Rate = (Gyro_Offset - GY) / 16.4;                    // 角速度，度/秒
Acc_Angle = atan2(AX, AZ) * 180 / π;                      // 加速度计角度
Angle     = 0.9*(Angle + Gyro_Rate*0.005) + 0.1*Acc_Angle; // 互补滤波
```

### 2. 直立环 PD

```c
Balance_Error = 0 - (Angle - Angle_Zero);
Balance_Out   = Balance_Kp * Balance_Error - Balance_Kd * Gyro_Rate;   // 限幅 ±100
```

### 3. 速度环 PI（6ms）

```c
Speed_Actual = -(Encoder_Get_Left() + Encoder_Get_Right()) / 2.0;
// 软启动爬坡 0.1/步 → 误差低通(0.2/0.8) → 积分(±300) → 倾角>30°清积分
Speed_Out = Speed_Kp * Speed_Error + Speed_Ki * Speed_ErrorInt;        // 限幅 ±100
```

### 4. 转向阻尼

```c
Turn_Out = Turn_Kp * ((GZ - GZ_Offset) / 16.4);                        // 限幅 ±50
```

### 5. 混控与倒地保护

```c
Base = Balance_Out + Speed_Out;
v_L  = Base - Turn_Out;   v_R = Base + Turn_Out;
Motor_SetSpeed_Left(v_L);  Motor_SetSpeed_Right(-v_R);   // 右轮取负（镜像安装）

if (|Angle - Angle_Zero| > 45°) { 两轮置 0 }              // 倒地保护
```

---

## 四、可调参数

全部集中在 `Core/Inc/pid.h` 顶部，改完直接编译烧录：

| 参数 | 默认值 | 作用 | 调整方向 |
|---|---|---|---|
| `BALANCE_KP_VALUE` | 2.31 | 直立环 P | 软倒→加大；猛冲→减小 |
| `BALANCE_KD_VALUE` | 0.41 | 直立环 D | 摆动→加大；高频抖→减小 |
| `SPEED_KP_VALUE` | 5.4 | 速度环 P | 前后窜→减小 |
| `SPEED_KI_VALUE` | 0.027 | 速度环 I | 慢慢漂→加大 |
| `TURN_KP_VALUE` | 0.4 | 转向阻尼 | 原地打转→加大 |
| `ANGLE_ZERO_VALUE` | -2.81 | 竖直零点 | 按实车标定 |
| `ANGLE_LIMIT_VALUE` | 45.0 | 倒地保护角 | — |

---

## 五、编译与烧录

1. **首次使用需要重新生成 HAL 库**
   本仓库未包含 `Drivers/`（约 65 MB 的 ST HAL 库）。用 STM32CubeMX 打开根目录的
   `Jaxbalancecar.ioc` → `GENERATE CODE` 即可重新生成。**注意不要覆盖 `Core/Src/pid.c`、
   `motor.c`、`encode.c`、`main.c` 里 USER CODE 区间之外的代码**。
2. Keil MDK 打开 `MDK-ARM/Jaxbalancecar.uvprojx`
3. 工程设置确认：
   - Device：`STM32F103C8`
   - Define：`USE_HAL_DRIVER, STM32F103xB`
   - 编译器：AC5（V5.06）或 AC6，C99
4. `F7` 编译 → `F8` 烧录（ST-Link，SWD）

---

## 六、上电流程

```
OLED 初始化 → Motor_Init（PWM 启动）→ Encoder_Init → MPU_Init
→ 显示 MPU6050 OK
→ PID_Calibrate()：陀螺仪零偏 + 角度零点（约 1 秒，车必须静止竖直）
→ 等待 3 秒（把车扶正）
→ 启动 TIM4 控制时基，进入平衡
```

OLED 显示：

```
Jax Balance Car
Kp:2.31 Kd:0.41      ← 直立环 PD
Kp:5.40 Ki:0.027     ← 速度环 PI
Ang:-3.5 S0          ← 当前角度 + 启停状态
```

---

## 七、调试经验

- **车轮方向反了 → 平衡环变正反馈 → 必然爆冲**。判断方法：车拿在手上、轮子离地，
  车头前倾时两轮应**向前**追。
- **倾角零点偏移**会让车"总往一个方向软倒"，看起来像"力气不够"，其实是 `Angle_Zero` 不对。
- **PWM 频率过低（1.1 kHz）**会让电机持续啸叫，改到 10 kHz 后消失。
- 编码器为集电极开路输出，引脚必须**上拉**（`GPIO_PULLUP`），否则计数恒为 0。

---

## 八、License

个人学习/课程项目代码，可自由参考。
