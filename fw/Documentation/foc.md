---
- version: v1.0  
- date: 2026-01-07  
- author: Wibot Studio
---

# 磁场定向控制（FOC）理论与架构

## 概述

**磁场定向控制**（Field-Oriented Control, FOC），又称矢量控制，是现代永磁同步电机（PMSM）和异步电机高性能控制的核心技术。FOC 通过坐标变换将三相交流电机等效为直流电机模型，实现磁链与转矩的解耦控制，从而获得快速的动态响应、高效率和精确的力矩控制能力。

本文档从理论基础出发，系统阐述 FOC 的数学原理、控制架构、坐标变换、环路设计，并与本项目的 STM32G431 固件实现建立对应关系。

---

## 一、理论基础

### 1.1 永磁同步电机的数学模型

永磁同步电机（PMSM）在定子坐标系下的电压方程为三相耦合的非线性系统。为便于控制，需通过坐标变换将其转换到旋转坐标系（dq 坐标系）。

#### 1.1.1 定子三相坐标系 (ABC)

电机定子三相绕组（U, V, W 相或 A, B, C 相）在空间上彼此相差 120° 电角度，通电后产生旋转磁场。三相电压方程为：

$$
\begin{bmatrix}
u_A \\
u_B \\
u_C
\end{bmatrix}
= R_s
\begin{bmatrix}
i_A \\
i_B \\
i_C
\end{bmatrix}
+ L
\frac{d}{dt}
\begin{bmatrix}
i_A \\
i_B \\
i_C
\end{bmatrix}
+ 
\begin{bmatrix}
e_A \\
e_B \\
e_C
\end{bmatrix}
$$

其中：
- $u_{A,B,C}$：三相定子电压
- $i_{A,B,C}$：三相定子电流
- $R_s$：定子电阻
- $L$：定子电感（简化模型，忽略互感）
- $e_{A,B,C}$：反电动势

### 1.2 FOC 的核心思想

FOC 的核心是通过坐标变换，将三相交流系统映射到与转子同步旋转的 **dq 坐标系**，在该坐标系下：

- **d 轴**：与转子磁链方向重合（励磁分量）
- **q 轴**：与 d 轴正交，超前 90° 电角度（转矩分量）

在 dq 坐标系中，电机模型简化为类似直流电机的形式：
- $i_d$ 控制磁链（类似直流电机的励磁电流）
- $i_q$ 控制转矩（类似直流电机的电枢电流）

对于表贴式永磁同步电机（SPMSM），通常设定 **$i_d = 0$**，即全部电流用于产生转矩，实现最大转矩电流比（MTPA）。

---

## 二、FOC 整体架构

FOC 控制系统采用**级联控制结构**，由外环到内环依次为：

```
位置环 → 速度环 → 电流环 → SVPWM → 逆变器 → 电机
         ↑                    ↑
      编码器反馈           电流采样反馈
```

### 2.1 控制框图

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐     ┌─────────────┐
│  位置指令   │───>│  位置环 PI  │───>│  速度指令   │     │             │
│  θ_ref      │    │  (可选)     │    │  ω_ref      │     │             │
└─────────────┘    └─────────────┘    └─────────────┘     │             │
                          ↑                               │             │
                     ┌────┴────┐                          │             │
                     │ 位置反馈│                          │  速度环 PI  │
                     │   θ     │                          │             │
                     └─────────┘                          │             │
                                                          └──────┬──────┘
┌─────────────┐                                                  │
│  速度指令   │──────────────────────────────────────────────────┤
│  ω_ref      │                                                  │
└─────────────┘                                                  ↓
                                                          ┌─────────────┐
                                                          │  转矩指令   │
                                                          │  T_ref/i_q  │
                                                          └──────┬──────┘
                                                                 │
                          ┌──────────────────────────────────────┘
                          ↓
                   ┌─────────────┐       ┌─────────────┐
                   │  电流环 PI  │       │  Park 逆变换│
                   │  (dq 坐标)  │──────>│   (dq→αβ)   │
                   │  id=0, iq   │       └──────┬──────┘
                   └──────┬──────┘              │
                          ↑                     ↓
                   ┌──────┴──────┐       ┌─────────────┐
                   │ Park 变换   │       │   SVPWM     │
                   │  (αβ→dq)    │       │  调制算法   │
                   └──────┬──────┘       └──────┬──────┘
                          │                     │
                   ┌──────┴──────┐              │
                   │ Clarke 变换 │              ↓
                   │  (ABC→αβ)   │       ┌─────────────┐
                   └──────┬──────┘       │  三相 PWM   │
                          │              │  (U, V, W)  │
                          │              └──────┬──────┘
                   ┌──────┴──────┐              │
                   │  三相电流   │              ↓
                   │  采样 ADC   │       ┌─────────────┐
                   └─────────────┘       │  三相逆变器 │
                                         │  (6-MOSFET) │
                                         └──────┬──────┘
                                                │
                                         ┌──────┴──────┐
                                         │  永磁同步   │
                                         │  电机 PMSM  │
                                         └─────────────┘
```

### 2.2 控制模式

根据 [apps/model.hpp](../apps/model.hpp) 中的定义，系统支持以下控制模式：

- **开环控制**（Open Loop）：直接给定电压角度和幅值，不依赖反馈，用于初始定位或低速启动
- **电流闭环**（Current Closed Loop）：力矩控制模式，直接控制 $i_q$，响应速度最快
- **速度闭环**（Speed Closed Loop）：速度控制模式，速度环输出作为电流环的 $i_q$ 参考
- **位置闭环**（Position Closed Loop）：位置控制模式，最外层位置环输出作为速度环参考

---

## 三、坐标变换

FOC 依赖两个核心坐标变换：**Clarke 变换**和 **Park 变换**。

### 3.1 Clarke 变换（ABC → αβ）

**Clarke 变换**将三相静止坐标系（ABC）变换到两相静止坐标系（αβ），α 轴与 A 相重合。

#### 3.1.1 变换公式（幅值不变型）- 推荐用于 FOC

**矩阵形式**：

$$
\begin{bmatrix}
i_\alpha \\
i_\beta
\end{bmatrix}
=
\sqrt{\frac{2}{3}}
\begin{bmatrix}
1 & -\frac{1}{2} & -\frac{1}{2} \\
0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2}
\end{bmatrix}
\begin{bmatrix}
i_A \\
i_B \\
i_C
\end{bmatrix}
$$

**简化形式**（利用 $i_A + i_B + i_C = 0$）：

$$
\begin{aligned}
i_\alpha &= i_A \cdot \sqrt{\frac{2}{3}} \\[0.5em]
i_\beta &= (i_A + 2i_B) \cdot \sqrt{\frac{2}{9}}
\end{aligned}
$$

其中 $\sqrt{\frac{2}{3}} \approx 0.8165$，$\sqrt{\frac{2}{9}} \approx 0.4714$

**幅值不变型的优势**（为什么 FOC 推荐使用）：

- ✅ **幅值保持不变**：$|\mathbf{I}_{abc}| = |\mathbf{I}_{\alpha\beta}|$，电流反馈值与 ADC 采样值量纲一致
- ✅ **电流环控制更直观**：过流保护阈值与实际电流值直接对应，无需额外转换
- ✅ **参数整定更容易**：PI 控制器参数与电流实际值一一对应，有利于稳定性
- ✅ **硬件保护对齐**：运放比较器的过流设定值与软件阈值保持一致

相比之下，功率不变型虽然保持功率守恒（$P_{abc} = P_{\alpha\beta}$），但会缩放电流幅值，不适合精确的电流闭环控制。

#### 3.1.2 项目实现对应

- **DSP 层实现**：[`Clarke` 类](../libs/wibotlib2/src/dsp/transform/clarke-park.hpp)，提供静态方法的数学计算
- **管道层实现**：[`ClarkeNode` 类](../libs/wibotlib2/src/pp/transform/clarke-park-node.hpp)，集成到 FOC 处理管道
- **执行时机**：在电流环控制任务中执行，采样三相电流后立即进行 Clarke 变换

### 3.2 Park 变换（αβ → dq）

**Park 变换**将两相静止坐标系（αβ）变换到两相旋转坐标系（dq），旋转角度为转子电角度 $\theta_e$。

#### 3.2.1 变换公式

$$
\begin{bmatrix}
i_d \\
i_q
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta_e & \sin\theta_e \\
-\sin\theta_e & \cos\theta_e
\end{bmatrix}
\begin{bmatrix}
i_\alpha \\
i_\beta
\end{bmatrix}
$$

其中 $\theta_e$ 是电角度，与机械角度的关系为：

$$
\theta_e = p \cdot \theta_m
$$

$p$ 为电机极对数（pole pairs）。

#### 3.2.2 逆变换（dq → αβ）

电流环输出的 dq 轴电压需要通过 Park 逆变换转回 αβ 坐标系：

$$
\begin{bmatrix}
u_\alpha \\
u_\beta
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta_e & -\sin\theta_e \\
\sin\theta_e & \cos\theta_e
\end{bmatrix}
\begin{bmatrix}
u_d \\
u_q
\end{bmatrix}
$$

#### 3.2.3 项目实现对应

- 数据结构：[`ParkTransform`](../apps/model.hpp)
- 角度获取：通过编码器/霍尔传感器获取机械角度，乘以极对数得到电角度
- 变换实现：在电流环控制任务中执行

---

## 四、控制环路设计

### 4.1 电流环（最内层，最快）

电流环是 FOC 的核心，运行在最高频率（通常与 PWM 同步，10~40 kHz）。

#### 4.1.1 dq 轴电压方程

在 dq 坐标系下，PMSM 的电压方程为：

$$
\begin{aligned}
u_d &= R_s i_d + L_d \frac{di_d}{dt} - \omega_e L_q i_q \\
u_q &= R_s i_q + L_q \frac{di_q}{dt} + \omega_e (L_d i_d + \psi_f)
\end{aligned}
$$

其中：
- $R_s$：定子电阻
- $L_d, L_q$：dq 轴电感（表贴式电机中 $L_d = L_q$）
- $\omega_e = p \cdot \omega_m$：电角速度
- $\psi_f$：永磁体磁链

#### 4.1.2 PI 控制器设计

采用两个独立的 PI 控制器分别控制 d 轴和 q 轴电流：

$$
\begin{aligned}
u_d^* &= K_{p,d}(i_d^* - i_d) + K_{i,d}\int (i_d^* - i_d)dt - \omega_e L_q i_q \\
u_q^* &= K_{p,q}(i_q^* - i_q) + K_{i,q}\int (i_q^* - i_q)dt + \omega_e (\psi_f + L_d i_d)
\end{aligned}
$$

最后两项为**前馈解耦项**，用于补偿 dq 轴交叉耦合，提高动态性能。

#### 4.1.3 MTPA 策略

对于表贴式 PMSM（$L_d = L_q$），最大转矩电流比（MTPA）策略为：

$$
i_d^* = 0, \quad T_e = \frac{3}{2} p \psi_f i_q
$$

即 d 轴电流参考设为 0，全部定子电流用于产生转矩。

#### 4.1.4 项目实现对应

- 电流采样：通过 ADC2（见 [Core/Inc/adc.h](../Core/Inc/adc.h)）采样三相电流
- 采样时机：PWM 中心对齐模式的顶点（见 [SVPWM 文档](components/svpwm.md) 的 `dutyAdcTrig`）
- PI 控制器：使用 wibotlib2 的 PID 控制器模块
- 电流限幅：参考 [`MotorParameters::maxCurrent`](../apps/model.hpp)

### 4.2 速度环（中层，中速）

速度环控制电机转速，其输出作为电流环的 $i_q^*$ 参考。速度环带宽通常为电流环的 1/5 ~ 1/10。

#### 4.2.1 速度环 PI 控制器

$$
i_q^* = K_{p,\omega}(\omega^* - \omega) + K_{i,\omega}\int (\omega^* - \omega)dt
$$

其中：
- $\omega^*$：速度参考（rad/s 或 RPM）
- $\omega$：实际转速（来自编码器或观测器）

#### 4.2.2 参数整定

- 速度环周期：1~10 ms
- 参数关系：$\frac{K_{i,\omega}}{K_{p,\omega}} = \frac{1}{\tau_\omega}$，$\tau_\omega$ 为速度环时间常数

#### 4.2.3 项目实现对应

- 速度测量：通过编码器（SPI3，见 [Core/Inc/spi.h](../Core/Inc/spi.h)）或反电动势观测器
- 控制周期：由 ThreadX 任务调度，典型值 1~5 ms
- 限幅：速度环输出应限制在 [`MotorParameters::maxCurrent`](../apps/model.hpp) 内

### 4.3 位置环（最外层，最慢）

位置环控制电机转子位置，其输出作为速度环的 $\omega^*$ 参考。

#### 4.3.1 位置环 PID 控制器

$$
\omega^* = K_{p,\theta}(\theta^* - \theta) + K_{i,\theta}\int (\theta^* - \theta)dt + K_{d,\theta}\frac{d(\theta^* - \theta)}{dt}
$$

#### 4.3.2 参数整定

- 位置环周期：10~100 ms
- 带宽关系：位置环带宽 < 速度环带宽 < 电流环带宽

#### 4.3.3 项目实现对应

- 位置测量：通过高分辨率编码器（如 SPI 磁编码器）
- 位置误差检测：见 [`ErrorFlags::positionError`](../apps/model.hpp)

---

## 五、PWM 调制与逆变器

### 5.1 SVPWM 调制

电流环输出的 dq 轴电压通过 Park 逆变换得到 αβ 轴电压，再通过 **空间矢量 PWM（SVPWM）** 调制生成三相 PWM 占空比。

SVPWM 相比传统正弦 PWM 的优势：
- 直流母线利用率提高 15%（$1/\sqrt{3}$ vs $1/2$）
- 谐波含量更低
- 便于电流采样同步

详细原理参见 [SVPWM 文档](components/svpwm.md)。

#### 5.1.1 项目实现对应

- SVPWM 算法：[`apps/components/svpwm.hpp`](../apps/components/svpwm.hpp)、[`apps/components/svpwm.cpp`](../apps/components/svpwm.cpp)
- PWM 驱动：[`apps/drivers/PwmDriver.hpp`](../apps/drivers/PwmDriver.hpp)、[`apps/drivers/PwmDriver.cpp`](../apps/drivers/PwmDriver.cpp)
- 定时器：TIM16（见 [Core/Inc/tim.h](../Core/Inc/tim.h)）
- 死区时间：在 `PwmDriver::setDeadTime()` 中配置，防止上下桥臂直通

### 5.2 三相逆变器

三相桥式逆变器由 6 个 MOSFET/IGBT 组成（每相 2 个），通过互补 PWM 驱动实现三相电压输出。

#### 5.2.1 关键参数

- 母线电压：$V_{dc}$，通过 ADC 采样（见 [`AdcSamples::busVoltage`](../apps/model.hpp)）
- 死区时间：防止上下桥臂直通的保护时间，典型值 500ns ~ 2μs
- 最大输出电压：线性区为 $V_{dc}/\sqrt{3}$（见 [SVPWM 文档](components/svpwm.md)）

---

## 六、传感器与反馈

### 6.1 电流采样

采用**双电阻或三电阻采样**方案，通过运放放大后送 ADC2 采样。

#### 6.1.1 采样时机

采用 PWM 同步采样：
- 中心对齐模式：在 PWM 计数器顶点（或底点）触发 ADC
- 优势：此时三相下桥臂导通时间最长，电流稳定
- 实现：SVPWM 输出的 `dutyAdcTrig` 用于配置定时器 ADC 触发时机

#### 6.1.2 项目实现对应

- ADC 配置：[Core/Inc/adc.h](../Core/Inc/adc.h)、[Core/Src/adc.c](../Core/Src/adc.c)
- DMA 传输：通过 DMA 自动搬运 ADC 数据到 [`AdcSamples`](../apps/model.hpp) 结构
- 电流标定：将 ADC 原始值转换为实际电流（A），需标定放大倍数和偏置

### 6.2 位置/速度传感器

#### 6.2.1 编码器方案

- **磁编码器**（如 AS5047P、MA730）：通过 SPI3 接口读取（见 [Core/Inc/spi.h](../Core/Inc/spi.h)）
- **光电编码器**：通过 TIM 编码器接口读取
- **霍尔传感器**：通过 GPIO 或 TIM 输入捕获读取（见 [Core/Inc/gpio.h](../Core/Inc/gpio.h)）

#### 6.2.2 速度计算

- **M 法**：固定时间测量位置增量，$\omega = \frac{\Delta\theta}{\Delta t}$
- **T 法**：固定角度增量测量时间，$\omega = \frac{\Delta\theta}{\Delta t}$
- **M/T 法**：结合两者，适用于宽速度范围

#### 6.2.3 项目实现对应

- 编码器接口：SPI3（磁编码器）或 TIM（正交编码器）
- 极对数配置：见 [`MotorParameters::polePairs`](../apps/model.hpp)
- 电角度计算：$\theta_e = p \cdot \theta_m \mod 2\pi$

### 6.3 母线电压与温度监测

- **母线电压**：通过电阻分压后 ADC 采样，用于 SVPWM 线性区限制
- **温度监测**：通过 NTC 热敏电阻或芯片内置温度传感器

---

## 七、实时控制与任务调度

### 7.1 ThreadX 任务架构

本项目基于 Azure RTOS ThreadX 实时操作系统，控制任务优先级分配：

| 任务       | 优先级 | 周期        | 功能                             |
| ---------- | ------ | ----------- | -------------------------------- |
| 电流环任务 | 0~2    | PWM 同步    | Clarke/Park 变换、PI 控制、SVPWM |
| 速度环任务 | 3~5    | 1~10 ms     | 速度 PI 控制                     |
| 位置环任务 | 6~8    | 10~100 ms   | 位置 PID 控制                    |
| 通信任务   | 9~12   | 按需        | USART/CAN 通信                   |
| 监控任务   | 13~15  | 100~1000 ms | 状态监控、故障检测               |

#### 7.1.1 项目实现对应

- 任务创建：见 [Core/Src/app_threadx.c](../Core/Src/app_threadx.c)
- 应用启动：[apps/app.cpp](../apps/app.cpp) 的 `bootApp()` 函数
- 控制循环框架：使用 wibotlib2 的 `control-loop.hpp` 模块

### 7.2 中断与同步

#### 7.2.1 关键中断

- **ADC 采样完成中断**：触发电流环任务，优先级最高
- **PWM 周期中断**：用于时序同步
- **编码器索引中断**：用于绝对位置标定
- **故障中断**：过流、过压等硬件保护

#### 7.2.2 实时性保证

- 电流环抖动：< 1% PWM 周期
- 中断响应时间：< 5 μs
- 上下文切换时间：< 2 μs（Cortex-M4 @ 170MHz）

---

## 八、故障保护与安全

### 8.1 硬件保护

- **过流保护**：运放比较器 + 硬件刹车（TIM Break 功能）
- **过压/欠压保护**：ADC 实时监测母线电压
- **过温保护**：温度传感器监测

### 8.2 软件保护

根据 [`ErrorFlags`](../apps/model.hpp) 定义：

- 过流检测：$i_{max} > I_{limit}$
- 堵转检测：电流超限且速度接近 0
- 位置误差检测：$|\theta^* - \theta| > \theta_{threshold}$
- 编码器故障检测：通信超时或校验失败

### 8.3 故障处理流程

```
检测到故障 → 设置 ErrorFlags → 进入 kError 状态 → 
紧急刹车（PwmDriver::emergencyBrake()） → 停止所有 PWM → 
等待故障清除 → 用户重启
```

---

## 九、参数整定指南

### 9.1 电机参数测量

根据 [`MotorParameters`](../apps/model.hpp) 结构，需要测量：

1. **极对数 $p$**：通过手动旋转转子一圈，计数编码器电周期数
2. **定子电阻 $R_s$**：锁定转子，施加直流电压测量电流
3. **定子电感 $L_s$**：施加高频小信号测量阻抗
4. **磁链 $\psi_f$**：通过反电动势常数 $K_e = \psi_f \cdot p$ 计算

### 9.2 PI 参数整定步骤

#### 9.2.1 电流环整定

1. 设定 $K_{i,d} = K_{i,q} = 0$，逐步增大 $K_{p,d}, K_{p,q}$ 直到出现震荡
2. 取震荡临界值的 60%
3. 加入积分项：$K_i = \frac{K_p}{\tau_i}$，$\tau_i \approx 5T_s$（$T_s$ 为采样周期）

#### 9.2.2 速度环整定

1. 先整定电流环
2. 设定 $K_{i,\omega} = 0$，逐步增大 $K_{p,\omega}$
3. 加入积分项：$K_{i,\omega} = \frac{K_{p,\omega}}{\tau_\omega}$，$\tau_\omega \approx 10T_\omega$

#### 9.2.3 位置环整定

1. 先整定速度环
2. 仅使用比例增益：$K_{p,\theta}$，逐步增大直到稳态误差可接受
3. 可选：加入微分项抑制超调

---

## 十、调试与验证

### 10.1 开环测试

1. **定位测试**：给定固定 $\theta_e$ 和 $i_q$，检查转子是否定位到指定角度
2. **开环旋转**：以固定频率递增 $\theta_e$，检查转子是否跟随旋转

### 10.2 电流环测试

1. 设定 $i_d^* = 0$，$i_q^* = $ 小电流（如 0.5A）
2. 观察实际 $i_d, i_q$ 是否跟踪参考
3. 检查 dq 轴解耦效果：改变 $i_q^*$ 时 $i_d$ 是否稳定

### 10.3 速度环测试

1. 给定速度参考（如 100 RPM）
2. 观察速度响应曲线：上升时间、超调量、稳态误差
3. 阶跃响应测试：快速改变速度参考，检查动态性能

### 10.4 日志与监控

使用 wibotlib2 的日志系统（见 `logger.hpp`）：

```cpp
LOG_INFO("电流环: id=%.3f, iq=%.3f", id, iq);
LOG_WARN("母线电压异常: Vdc=%.1f V", vdc);
LOG_ERROR("过流保护: I=%.2f A", current);
```

通过 USART2（见 [Core/Inc/usart.h](../Core/Inc/usart.h)）输出调试信息。

---

## 十一、与项目实现的对应关系

### 11.1 核心模块映射

| FOC 功能模块 | 项目实现                                                                        |
| ------------ | ------------------------------------------------------------------------------- |
| 电机参数定义 | [`MotorParameters`](../apps/model.hpp)                                          |
| 控制模式     | [`ControlMode`](../apps/model.hpp)                                              |
| 状态机       | [`MotorState`](../apps/model.hpp)                                               |
| 故障管理     | [`ErrorFlags`](../apps/model.hpp)                                               |
| Clarke 变换  | [`Clarke` 类](../libs/wibotlib2/src/dsp/transform/clarke-park.hpp) (幅值不变型) |
| Park 变换    | [`Park` 类](../libs/wibotlib2/src/dsp/transform/clarke-park.hpp)                |
| InvPark 变换 | [`InvPark` 类](../libs/wibotlib2/src/dsp/transform/clarke-park.hpp)             |
| SVPWM 算法   | [`Svpwm` 类](../apps/components/svpwm.hpp)                                      |
| PWM 驱动     | [`PwmDriver` 类](../apps/drivers/PwmDriver.hpp)                                 |
| ADC 采样     | [`AdcSamples`](../apps/model.hpp) + [adc.c](../Core/Src/adc.c)                  |
| PI 控制器    | wibotlib2 提供（待集成）                                                        |
| 控制循环框架 | wibotlib2 `control-loop.hpp`                                                    |

### 11.2 硬件外设映射

| 外设   | 用途                   | 配置文件                                  |
| ------ | ---------------------- | ----------------------------------------- |
| ADC2   | 三相电流、母线电压采样 | [Core/Inc/adc.h](../Core/Inc/adc.h)       |
| TIM16  | 三相 PWM 生成          | [Core/Inc/tim.h](../Core/Inc/tim.h)       |
| SPI3   | 磁编码器通信           | [Core/Inc/spi.h](../Core/Inc/spi.h)       |
| USART2 | 调试日志输出           | [Core/Inc/usart.h](../Core/Inc/usart.h)   |
| DMA    | ADC/SPI 数据搬运       | [Core/Inc/dma.h](../Core/Inc/dma.h)       |
| CORDIC | 三角函数硬件加速       | [Core/Inc/cordic.h](../Core/Inc/cordic.h) |

### 11.3 文件组织结构

```
apps/
├── model.hpp              # 数据结构与电机参数定义
├── app.cpp                # 应用程序主入口
├── config.hpp             # 配置参数
├── components/            # 硬件无关算法
│   ├── svpwm.hpp/cpp     # SVPWM 调制算法
│   ├── foc.hpp/cpp       # FOC 控制核心算法
│   └── (待添加: PI 控制器等)
└── drivers/               # 硬件相关驱动
    └── PwmDriver.hpp/cpp # PWM 定时器驱动

libs/
└── wibotlib2/
    └── src/
        ├── dsp/transform/
        │   └── clarke-park.hpp     # Clarke/Park/InvPark 变换（DSP 层）
        └── pp/transform/
            └── clarke-park-node.hpp # Clarke/Park/InvPark 管道节点（管道层）
```

---

## 十二、参考资料

### 12.1 经典教材

- 《永磁同步电机矢量控制技术》- 袁登科
- 《现代永磁同步电机控制原理及MATLAB仿真》- 袁雷
- "Vector Control of AC Drives" - Peter Vas

### 12.2 应用笔记

- STM32 FOC SDK 文档（ST Application Notes）
- Texas Instruments "Field Oriented Control (FOC) for Motor Drives"
- Infineon "PMSM FOC Motor Control"

### 12.3 项目文档

- [SVPWM 原理推导与实现](components/svpwm.md)
- [项目开发指南 AGENTS.md](../AGENTS.md)

---

## 附录：常用公式速查

### A.1 坐标变换

**Clarke 变换（ABC → αβ，幅值不变型）**：
$$
i_\alpha = i_A \cdot \sqrt{\frac{2}{3}}, \quad i_\beta = (i_A + 2i_B) \cdot \sqrt{\frac{2}{9}}
$$

**Park 变换（αβ → dq）**：
$$
i_d = i_\alpha \cos\theta_e + i_\beta \sin\theta_e, \quad
i_q = -i_\alpha \sin\theta_e + i_\beta \cos\theta_e
$$

**Park 逆变换（dq → αβ）**：
$$
u_\alpha = u_d \cos\theta_e - u_q \sin\theta_e, \quad
u_\beta = u_d \sin\theta_e + u_q \cos\theta_e
$$

### A.2 电机方程

**转矩方程**（表贴式 PMSM）：
$$
T_e = \frac{3}{2} p \psi_f i_q
$$

**机械方程**：
$$
J \frac{d\omega_m}{dt} = T_e - T_L - B\omega_m
$$

其中 $J$ 为转动惯量，$B$ 为摩擦系数，$T_L$ 为负载转矩。

### A.3 SVPWM

**线性区最大电压**：
$$
V_{\max,lin} = \frac{V_{dc}}{\sqrt{3}} \approx 0.577 \times V_{dc}
$$

---


