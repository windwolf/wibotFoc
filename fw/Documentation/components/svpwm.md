**SVPWM 原理推导与实现对照（结合实现）**

本节面向本项目的三相两电平逆变器，系统性推导空间矢量 PWM（Space Vector PWM, SVPWM）的几何原理与时间分配公式，并给出与工程实现的一一对应关系。实现代码见 [apps/components/svpwm.hpp](../../apps/components/svpwm.hpp)、[apps/components/svpwm.cpp](../../apps/components/svpwm.cpp)。

**概述**

- 目标：在一个载波周期内，使用两条相邻有功矢量与零矢量的加权时间合成目标电压矢量，使定子电压矢量在 $\alpha$-$\beta$ 平面尽可能逼近参考矢量。
- 优势：相比正弦 PWM，SVPWM 在线性区可将最大相电压幅值提升至直流母线的 $1/\sqrt{3}$，并通过对称七段式序列抑制低次谐波、便于电流同步采样。

**坐标系与线性区**

- 通过 Clarke 变换，三相参考电压映射到 $\alpha$-$\beta$ 平面，参考矢量表示为 $(V_\alpha, V_\beta)$。
- 两电平三相逆变器在 $\alpha$-$\beta$ 平面的可达空间是正六边形，其内切圆半径对应线性调制区最大幅值：
	$$
	V_{\max,\,lin} = \frac{V_{dc}}{\sqrt{3}}
	$$
- 实现对应：最大线性幅值由 `Svpwm::getMaxModulationVoltage()` 提供；`Svpwm::limitVoltage()` 在计算前对 $(V_\alpha,V_\beta)$ 做等比例缩放，确保在线性区边界内，见 [apps/components/svpwm.cpp](../../apps/components/svpwm.cpp)。

**扇区判定（Sector）**

六边形划分为 6 个 60° 扇区。参考矢量所在扇区由三个判别量的符号确定：

$$
\begin{aligned}
A &= \operatorname{sign}(V_\beta),\\
B &= \operatorname{sign}\!\left(\tfrac{\sqrt{3}}{2}V_\alpha - \tfrac{1}{2}V_\beta\right),\\
C &= \operatorname{sign}\!\left(-\tfrac{\sqrt{3}}{2}V_\alpha - \tfrac{1}{2}V_\beta\right),\\
N &= 4C + 2B + A\;\in\;\{1,2,3,4,5,6\}.
\end{aligned}
$$

在实现中，`Svpwm::determineSector()` 使用上述规则计算 $N$，并查表得到 `sector ∈ {1…6}`。常量 $\sqrt{3}/2$ 以 `kSQRT3_2` 表示，详见 [apps/components/svpwm.cpp](../../apps/components/svpwm.cpp)。

扇区划分与线性区示意：见 [Documentation/components/img/svpwm_sectors.svg](img/svpwm_sectors.svg)。

**两有功矢量作用时间的推导**

将参考矢量归一化为相对于母线电压的无量纲量：$V_{\alpha,n}=V_\alpha/V_{dc}$，$V_{\beta,n}=V_\beta/V_{dc}$。定义三个投影量：

$$
\begin{aligned}
x &= V_{\beta,n},\\
y &= \tfrac{\sqrt{3}}{2}V_{\alpha,n} - \tfrac{1}{2}V_{\beta,n},\\
z &= -\tfrac{\sqrt{3}}{2}V_{\alpha,n} - \tfrac{1}{2}V_{\beta,n}.
\end{aligned}
$$

在任一扇区内，参考矢量可分解为两条相邻有功矢量 $\mathbf{V}_k,\mathbf{V}_{k+1}$ 的线性组合作用时间与零矢量时间之和：

$$
t_1\,\mathbf{V}_k + t_2\,\mathbf{V}_{k+1} + t_0\,\mathbf{0} = \mathbf{V}^*,\quad t_0+t_1+t_2=1,\; t_i\ge 0.
$$

对照实现，六个扇区的 $(t_1,t_2)$ 由 $(x,y,z)$ 按下表给出（见 [apps/components/svpwm.cpp](../../apps/components/svpwm.cpp) 的 `switch` 分支）：

- Sector 1: $t_1=-z,\; t_2=-y$
- Sector 2: $t_1= z,\; t_2= x$
- Sector 3: $t_1=-x,\; t_2=-z$
- Sector 4: $t_1= y,\; t_2= x$
- Sector 5: $t_1=-x,\; t_2=-y$
- Sector 6: $t_1= z,\; t_2= y$

实现随后将 $t_1,t_2$ 裁剪到 $[0,1]$ 并计算 $t_0=1-t_1-t_2$。

**七段式对称调制与占空比合成**

采用对称七段式序列：

0 → $\mathbf{V}_k$ → $\mathbf{V}_{k+1}$ → 0 → $\mathbf{V}_{k+1}$ → $\mathbf{V}_k$ → 0，

将零矢量时间平均分配于两端，记 $t_0' = t_0/2$。三相占空比在一个归一化周期内按扇区为：

- Sector 1: U: $t_0'+t_1+t_2$，V: $t_0'+t_2$，W: $t_0'$
- Sector 2: U: $t_0'+t_1$，V: $t_0'+t_1+t_2$，W: $t_0'$
- Sector 3: U: $t_0'$，V: $t_0'+t_1+t_2$，W: $t_0'+t_2$
- Sector 4: U: $t_0'$，V: $t_0'+t_1$，W: $t_0'+t_1+t_2$
- Sector 5: U: $t_0'+t_2$，V: $t_0'$，W: $t_0'+t_1+t_2$
- Sector 6: U: $t_0'+t_1+t_2$，V: $t_0'$，W: $t_0'+t_1$

上述公式在 `Svpwm::calculate()` 的扇区分支中直接实现，输出字段为 `dutyU/dutyV/dutyW`。ADC 采样触发使用中心对齐“顶点”，在实现中 `dutyAdcTrig = 1.0f`。

- 七段式对称调制时序（归一化）：见 [Documentation/components/img/svpwm_timing.svg](img/svpwm_timing.svg)。

**线性区限制与过调处理**

- 线性区限制：若 $\|\mathbf{V}^*\|>V_{dc}/\sqrt{3}$，则按比例缩放至边界：
	$$
	(V_\alpha,V_\beta) \leftarrow \frac{V_{dc}/\sqrt{3}}{\sqrt{V_\alpha^2+V_\beta^2}}\,(V_\alpha,V_\beta).
	$$
	对应实现：`Svpwm::limitVoltage()`。
- 过调 I（平顶化）：若计算得 $t_0<0$，实现将两有功时间按和归一化并置 $t_0=0$，即
	$$
	t_1\leftarrow\frac{t_1}{t_1+t_2},\quad t_2\leftarrow\frac{t_2}{t_1+t_2},\quad t_0\leftarrow 0,
	$$
	对应 `Svpwm::calculate()` 中对负 $t_0$ 的处理。该策略提升等效基波幅值但增加谐波与器件应力。

**极坐标接口与工程细节**

- 极坐标输入：`Svpwm::calculatePolar(angle, magnitude)` 通过 `Math::sincos(angle, magnitude)` 生成 $(m\sin\theta, m\cos\theta)$ 并映射为 $(V_\alpha,V_\beta)=(m\cos\theta, m\sin\theta)` 后复用 `calculate()`，见 [apps/components/svpwm.cpp](../../apps/components/svpwm.cpp)。
- 常量与数值稳定：实现使用 `kSQRT3_2=\sqrt{3}/2`，对 $t_1,t_2$ 进行边界裁剪，`dutyAdcTrig` 便于与中心对齐 PWM 的电流采样同步。
- 母线电压接口：`setBusVoltage()/getBusVoltage()` 允许运行期更新 $V_{dc}$，以适配电池/母线波动。

**与代码对照**

- `Svpwm` 类与输出结构：见 [apps/components/svpwm.hpp](../../apps/components/svpwm.hpp)
- 扇区判定：`Svpwm::determineSector()`，见 [apps/components/svpwm.cpp](../../apps/components/svpwm.cpp)
- 主计算：`Svpwm::calculate()`（含 $(x,y,z)$、$t_1,t_2,t_0$、七段式占空比），见 [apps/components/svpwm.cpp](../../apps/components/svpwm.cpp)
- 极坐标入口：`Svpwm::calculatePolar()`，见 [apps/components/svpwm.cpp](../../apps/components/svpwm.cpp)
- 线性区边界与缩放：`getMaxModulationVoltage()` 与 `limitVoltage()`，见 [apps/components/svpwm.cpp](../../apps/components/svpwm.cpp)

**使用建议**

- 幅值规划：若需始终在线性区，规划 $\|\mathbf{V}^*\|\le V_{dc}/\sqrt{3}$；可保留 `limitVoltage()` 自动限幅。
- 采样同步：采用中心对齐 PWM，在计数器顶点采样电流信噪比更佳；可使用输出中的 `dutyAdcTrig` 与定时器触发逻辑配合。
- 电压自适应：直流母线随负载/电池波动时，周期性更新 `setBusVoltage()`，以维持调制线性与磁链利用率。

—— 完 ——

