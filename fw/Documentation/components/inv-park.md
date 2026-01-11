# InvPark（反Park变换）使用指南

## 概述

InvPark（反Park变换）是 FOC 电机控制中的核心坐标变换模块，用于将 **dq 旋转坐标系** 的电压/电流转换回 **αβ 静止坐标系**。

在 FOC 控制流程中的位置：

```
电流环PID输出(dq) → InvPark变换(dq→αβ) → SVPWM调制(αβ→PWM) → 三相逆变器
```

## 数学原理

反Park变换的数学公式：

```
┌───────┐   ┌                      ┐ ┌───┐
│ u_α   │   │ cos(θ)   -sin(θ)    │ │ ud│
│       │ = │                      │ │   │
│ u_β   │   │ sin(θ)    cos(θ)    │ │ uq│
└───────┘   └                      ┘ └───┘
```

其中：
- `ud`, `uq`: dq 旋转坐标系下的分量（电流环输出）
- `θ`: 转子电角度（从编码器获取）
- `u_α`, `u_β`: αβ 静止坐标系下的分量（输入到 SVPWM）

### 物理意义

- **d 轴**：磁场方向分量（励磁），通常设为 0（表贴式电机）
- **q 轴**：转矩方向分量，控制电机输出转矩
- **α 轴**：静止坐标系水平方向（对齐 A 相）
- **β 轴**：静止坐标系垂直方向（超前 α 轴 90°）

## 使用方式

### 方式 1：直接调用静态方法（最快）

适用于简单场景，不需要 Node 封装：

```cpp
#include "dsp/transform/inv-park.hpp"

using namespace wibot;

// 输入
f32 ud = 0.0f;      // d轴电压（通常为0）
f32 uq = 15.0f;     // q轴电压（控制转矩）
f32 theta = 1.047f; // 电角度（60度）

// 输出
f32 uAlpha, uBeta;

// 执行变换
InvPark::transform(ud, uq, theta, uAlpha, uBeta);

// 继续传给 SVPWM
svpwm.calculate(uAlpha, uBeta);
```

### 方式 2：使用 Node 在 Pipeline 中（推荐）

适用于复杂控制系统，便于模块化和调试：

```cpp
#include "pp/transform/inv-park-node.hpp"
#include "pp/pipeline.hpp"

using namespace wibot;

// 1. 创建数据存储
struct FocData {
    f32 ud;        // d轴电压输出
    f32 uq;        // q轴电压输出
    f32 theta;     // 电角度
    f32 uAlpha;    // α轴电压
    f32 uBeta;     // β轴电压
} focData;

// 2. 创建 Node
InvParkNode invParkNode;

// 3. 构建 Pipeline
PipelineChainBuilder<16> builder;
builder.addNode(invParkNode);

// 4. 绑定数据
builder.bind(invParkNode.inputs.d, focData.ud);
builder.bind(invParkNode.inputs.q, focData.uq);
builder.bind(invParkNode.inputs.theta, focData.theta);
builder.bind(invParkNode.outputs.alpha, focData.uAlpha);
builder.bind(invParkNode.outputs.beta, focData.uBeta);

// 5. 构建执行链
auto chain = builder.build();

// 6. 在控制循环中执行
void currentLoopCallback() {
    // ... PID 控制器更新 focData.ud, focData.uq
    // ... 编码器更新 focData.theta
    
    // 执行 Pipeline（包含 InvPark）
    chain.tick();
    
    // focData.uAlpha, focData.uBeta 已更新
    // ... 传给 SVPWM
}
```

### 方式 3：在完整 FOC Pipeline 中使用

```cpp
#include "pp/transform/inv-park-node.hpp"
#include "pp/controller/pid-node.hpp"
#include "pp/source/constant-source.hpp"

// 创建完整 FOC 控制链
PipelineChainBuilder<32> builder;

// 电流环 PID 节点
PidNode idPid(idPidConfig);  // d轴电流环
PidNode iqPid(iqPidConfig);  // q轴电流环

// 反Park变换节点
InvParkNode invPark;

// 添加到 Pipeline
builder.addNode(idPid)
       .addNode(iqPid)
       .addNode(invPark);

// 连接数据流
builder.bind(idPid.inputs.setPoint, currentData.idTarget);
builder.bind(idPid.inputs.measurement, currentData.idMeasured);
builder.bind(idPid.outputs.output, voltageData.ud);

builder.bind(iqPid.inputs.setPoint, currentData.iqTarget);
builder.bind(iqPid.inputs.measurement, currentData.iqMeasured);
builder.bind(iqPid.outputs.output, voltageData.uq);

builder.bind(invPark.inputs.d, voltageData.ud);
builder.bind(invPark.inputs.q, voltageData.uq);
builder.bind(invPark.inputs.theta, positionData.electricalAngle);
builder.bind(invPark.outputs.alpha, voltageData.uAlpha);
builder.bind(invPark.outputs.beta, voltageData.uBeta);

auto chain = builder.build();
```

## 性能优化

### 1. 预计算 sin/cos

如果相同角度需要多次变换，可以预先计算三角函数：

```cpp
f32 cosTheta = std::cos(theta);
f32 sinTheta = std::sin(theta);

// 多次变换使用相同角度
InvPark::transformWithSinCos(ud1, uq1, cosTheta, sinTheta, uAlpha1, uBeta1);
InvPark::transformWithSinCos(ud2, uq2, cosTheta, sinTheta, uAlpha2, uBeta2);
```

### 2. 使用硬件 CORDIC

STM32G431 内置 CORDIC 协处理器，可加速三角函数计算。wibotlib 的 Math 类会自动使用硬件加速。

### 3. 定点运算

对于资源受限的系统，可以使用 Q15/Q31 定点数代替浮点数（需要修改 InvPark 实现）。

## 验证和调试

### 单元测试

项目提供了完整的测试用例：

```bash
# 运行测试
InvParkTest::runTests();
```

测试覆盖：
- 零输入测试
- 特殊角度测试（0°, 90°, 180°）
- 任意角度测试
- 能量守恒验证（向量模长不变）
- Node 接口一致性
- Pipeline 集成测试

### 常见问题

**Q1: 输出电压幅值不正确**

检查：
- 电角度 `theta` 是否正确（应在 0 到 2π 范围内）
- dq 输入是否已经过 PID 限幅
- 是否使用了正确的坐标系（电角度 vs 机械角度）

**Q2: 电机抖动或运行不稳定**

检查：
- 电角度更新频率是否足够高（应与 PWM 同步）
- 电角度是否连续（避免突变）
- InvPark 输出是否立即传给 SVPWM（避免延迟）

**Q3: 电机方向错误**

可能需要调整：
- 电角度的极对数（`θ_e = polePairs × θ_m`）
- 电角度的方向（正转 vs 反转）
- Park/InvPark 变换的符号

## 相关模块

- **Park 变换**：InvPark 的逆变换（αβ → dq），用于电流反馈
- **Clarke 变换**：三相 → αβ 变换（ABC → αβ）
- **SVPWM**：αβ → PWM 调制
- **PID 控制器**：生成 dq 轴电压指令

## 参考资料

- [FOC 理论文档](../Documentation/foc.md)
- [SVPWM 文档](../Documentation/components/svpwm.md)
- [STM32 电机控制应用手册](https://www.st.com/resource/en/application_note/an1078-foc-pmsm-motor-drive.pdf)

## 示例代码

完整示例见：
- 测试代码：[`tests/inv_park_test.cpp`](../../tests/inv_park_test.cpp)
- FOC 控制器：[`apps/components/foc.cpp`](../../apps/components/foc.cpp)
