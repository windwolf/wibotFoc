#pragma once

#include "type.hpp"

namespace wibot::motor {

struct MotorParameters {
  u8 polePairs;           // 电机极对数
  float statorResistance; // 定子电阻 (Ohms)
  float statorInductance; // 定子电感 (Henrys)
  float fluxLinkage;      // 磁链 (Weber)
  float ratedCurrent;     // 额定电流 (Amperes)
  float ratedSpeed;       // 额定转速 (RPM)
  float ratedVoltage;     // 额定电压 (Volts)
  float maxCurrent;       // 最大电流限制 (Amperes)
  float inertia;          // 转动惯量 (kg*m^2)
  float friction;         // 摩擦系数 (Nm/(rad/s))
};

// ==================== 控制模式 ====================
enum class ControlMode : u8 {
  kIdle,               // 空闲状态
  kOpenLoop,           // 开环控制
  kCurrentClosedLoop,  // 电流闭环 (力矩控制)
  kSpeedClosedLoop,    // 速度闭环
  kPositionClosedLoop, // 位置闭环
  kCalibration,        // 校准模式
  kError,              // 错误状态
};

// ==================== 运行状态 ====================
enum class MotorState : u8 {
  kStopped,     // 停止
  kStarting,    // 启动中
  kRunning,     // 运行中
  kBraking,     // 制动中
  kFault,       // 故障
  kCalibrating, // 校准中
};

// ==================== 错误标志位 ====================
struct ErrorFlags {
  bool overCurrent : 1;     // 过流
  bool overVoltage : 1;     // 过压
  bool underVoltage : 1;    // 欠压
  bool overTemperature : 1; // 过温
  bool encoderFault : 1;    // 编码器故障
  bool hallFault : 1;       // 霍尔传感器故障
  bool motorStall : 1;      // 电机堵转
  bool positionError : 1;   // 位置误差过大
  u8 reserved : 8;          // 保留位

  // 清除所有错误
  void clear() { *reinterpret_cast<u16 *>(this) = 0; }

  // 检查是否有错误
  bool hasError() const {
    return (*reinterpret_cast<const u16 *>(this) & 0xFF) != 0;
  }
};

// ==================== Clarke 变换结构 (三相 -> 两相静止坐标系)
// ====================
struct ClarkeTransform {
  float alpha; // α轴分量
  float beta;  // β轴分量
};

// ==================== Park 变换结构 (两相静止 -> 两相旋转坐标系)
// ====================
struct ParkTransform {
  float d; // d轴分量 (磁场方向)
  float q; // q轴分量 (转矩方向)
};

// ==================== 三相电流/电压 ====================
struct ThreePhase {
  float u; // U相 (A相)
  float v; // V相 (B相)
  float w; // W相 (C相)

  void clear() {
    u = 0.0f;
    v = 0.0f;
    w = 0.0f;
  }
};

// ==================== ADC 采样数据 ====================
struct AdcSamples {
  u16 currentU;    // U相电流 ADC 原始值
  u16 currentV;    // V相电流 ADC 原始值
  u16 currentW;    // W相电流 ADC 原始值 (可选)
  u16 busVoltage;  // 母线电压 ADC 原始值
  u16 temperature; // 温度传感器 ADC 原始值
  u32 timestamp;   // 采样时间戳 (us)
};

// ==================== 电流测量值 ====================
struct CurrentMeasurement {
  ThreePhase phase;          // 三相电流 (Amperes)
  ClarkeTransform alphabeta; // Clarke 变换后的电流
  ParkTransform dq;          // Park 变换后的电流
  float busVoltage;          // 母线电压 (Volts)
  u32 timestamp;             // 时间戳 (us)
};

// ==================== 位置和速度信息 ====================
struct PositionInfo {
  float mechanicalAngle; // 机械角度 (弧度: 0 ~ 2π)
  float electricalAngle; // 电气角度 (弧度: 0 ~ 2π)
  float speed;           // 转速 (RPM)
  float angularVelocity; // 角速度 (rad/s)
  i32 encoderCount;      // 编码器计数值
  bool valid;            // 位置信息有效标志
  u32 timestamp;         // 时间戳 (us)
};

// ==================== PID 控制器参数 ====================
struct PidConfig {
  float kp;            // 比例系数
  float ki;            // 积分系数
  float kd;            // 微分系数
  float outputLimit;   // 输出限制
  float integralLimit; // 积分限制 (抗饱和)

  void setDefaults() {
    kp = 0.0f;
    ki = 0.0f;
    kd = 0.0f;
    outputLimit = 0.0f;
    integralLimit = 0.0f;
  }
};

// ==================== PID 控制器状态 ====================
struct PidState {
  float integral;  // 积分累加值
  float lastError; // 上次误差值
  float output;    // 控制器输出

  void reset() {
    integral = 0.0f;
    lastError = 0.0f;
    output = 0.0f;
  }
};

// ==================== FOC 完整配置 ====================
struct FocConfig {
  // 电机参数
  MotorParameters motorParams;

  // 控制模式
  ControlMode controlMode;

  // PID 控制器参数
  PidConfig currentDAxisPid; // d轴电流环 PID
  PidConfig currentQAxisPid; // q轴电流环 PID
  PidConfig speedPid;        // 速度环 PID
  PidConfig positionPid;     // 位置环 PID

  // 控制周期
  float currentLoopPeriod;  // 电流环周期 (秒)
  float speedLoopPeriod;    // 速度环周期 (秒)
  float positionLoopPeriod; // 位置环周期 (秒)

  // PWM 配置
  u16 pwmFrequency;  // PWM 频率 (Hz)
  float pwmDeadTime; // 死区时间 (秒)

  // 限制参数
  float maxSpeed;        // 最大速度 (RPM)
  float maxAcceleration; // 最大加速度 (RPM/s)
  float maxDeceleration; // 最大减速度 (RPM/s)

  // 其他参数
  bool enableFieldWeakening;   // 使能弱磁控制
  float fieldWeakeningCurrent; // 弱磁电流 (Amperes)
};

// ==================== 控制目标/指令 ====================
struct ControlCommand {
  ControlMode mode; // 控制模式

  // 目标值 (根据控制模式使用不同字段)
  float targetCurrent;  // 目标电流 (Amperes) - 电流模式
  float targetSpeed;    // 目标速度 (RPM) - 速度模式
  float targetPosition; // 目标位置 (弧度) - 位置模式

  // 开环控制参数
  float openLoopVoltage;   // 开环电压 (Volts)
  float openLoopFrequency; // 开环频率 (Hz)

  // 前馈控制
  float feedforwardCurrent; // 前馈电流 (Amperes)
  float feedforwardVoltage; // 前馈电压 (Volts)

  // 指令时间戳
  u32 timestamp; // 时间戳 (us)

  void clear() {
    targetCurrent = 0.0f;
    targetSpeed = 0.0f;
    targetPosition = 0.0f;
    openLoopVoltage = 0.0f;
    openLoopFrequency = 0.0f;
    feedforwardCurrent = 0.0f;
    feedforwardVoltage = 0.0f;
  }
};

// ==================== 电压指令 ====================
struct VoltageCommand {
  ParkTransform dq;          // dq轴电压指令
  ClarkeTransform alphabeta; // αβ轴电压指令
  ThreePhase phase;          // 三相电压指令
};

// ==================== FOC 运行状态 ====================
struct FocState {
  // 系统状态
  MotorState motorState; // 电机状态
  ErrorFlags errors;     // 错误标志

  // 测量值
  CurrentMeasurement current; // 电流测量
  PositionInfo position;      // 位置和速度
  float temperature;          // 温度 (摄氏度)

  // 控制器状态
  PidState currentDAxisState; // d轴电流环状态
  PidState currentQAxisState; // q轴电流环状态
  PidState speedState;        // 速度环状态
  PidState positionState;     // 位置环状态

  // 电压指令
  VoltageCommand voltage; // 电压指令

  // 性能统计
  u32 loopCounter;   // 循环计数器
  float avgLoopTime; // 平均循环时间 (us)
  float maxLoopTime; // 最大循环时间 (us)

  // 系统时间
  u32 systemTime; // 系统时间 (us)

  void reset() {
    motorState = MotorState::kStopped;
    errors.clear();
    currentDAxisState.reset();
    currentQAxisState.reset();
    speedState.reset();
    positionState.reset();
    loopCounter = 0;
    avgLoopTime = 0.0f;
    maxLoopTime = 0.0f;
  }
};

// ==================== 校准数据 ====================
struct CalibrationData {
  // ADC 偏移量
  float currentUOffset; // U相电流 ADC 偏移
  float currentVOffset; // V相电流 ADC 偏移
  float currentWOffset; // W相电流 ADC 偏移

  // ADC 增益
  float currentGain; // 电流采样增益 (A/ADC)
  float voltageGain; // 电压采样增益 (V/ADC)

  // 编码器校准
  i32 encoderZeroOffset;  // 编码器零点偏移
  float encoderDirection; // 编码器方向 (+1.0 或 -1.0)

  // 电角度偏移
  float electricalAngleOffset; // 电角度偏移 (弧度)

  // 校准标志
  bool isCalibrated;   // 是否已校准
  u32 calibrationTime; // 校准时间戳
};

// ==================== 系统监控数据 ====================
struct SystemMonitor {
  // 功率统计
  float inputPower;  // 输入功率 (Watts)
  float outputPower; // 输出功率 (Watts)
  float efficiency;  // 效率 (%)

  // 能量统计
  float energyConsumed;    // 消耗能量 (Joules)
  float energyRegenerated; // 回馈能量 (Joules)

  // 温度监控
  float motorTemperature;  // 电机温度 (摄氏度)
  float driverTemperature; // 驱动器温度 (摄氏度)

  // 运行时间
  u32 totalRunTime;   // 总运行时间 (秒)
  u32 sessionRunTime; // 本次运行时间 (秒)

  void reset() {
    inputPower = 0.0f;
    outputPower = 0.0f;
    efficiency = 0.0f;
    energyConsumed = 0.0f;
    energyRegenerated = 0.0f;
    sessionRunTime = 0;
  }
};

// ==================== FOC 完整数据模型 ====================
struct FocModel {
  FocConfig config;            // 配置参数
  FocState state;              // 运行状态
  ControlCommand command;      // 控制指令
  CalibrationData calibration; // 校准数据
  SystemMonitor monitor;       // 系统监控

  // 初始化默认值
  void initDefaults() {
    // 默认电机参数 (示例值)
    config.motorParams.polePairs = 7;
    config.motorParams.statorResistance = 0.5f;
    config.motorParams.statorInductance = 0.0005f;
    config.motorParams.fluxLinkage = 0.01f;
    config.motorParams.ratedCurrent = 10.0f;
    config.motorParams.ratedSpeed = 3000.0f;
    config.motorParams.ratedVoltage = 24.0f;
    config.motorParams.maxCurrent = 15.0f;

    // 默认控制模式
    config.controlMode = ControlMode::kIdle;

    // 默认 PID 参数
    config.currentDAxisPid.setDefaults();
    config.currentQAxisPid.setDefaults();
    config.speedPid.setDefaults();
    config.positionPid.setDefaults();

    // 默认控制周期
    config.currentLoopPeriod = 0.0001f; // 100us (10kHz)
    config.speedLoopPeriod = 0.001f;    // 1ms (1kHz)
    config.positionLoopPeriod = 0.01f;  // 10ms (100Hz)

    // PWM 配置
    config.pwmFrequency = 20000;    // 20kHz
    config.pwmDeadTime = 0.000001f; // 1us

    // 限制参数
    config.maxSpeed = 5000.0f;         // RPM
    config.maxAcceleration = 10000.0f; // RPM/s
    config.maxDeceleration = 20000.0f; // RPM/s

    // 弱磁控制
    config.enableFieldWeakening = false;
    config.fieldWeakeningCurrent = 0.0f;

    // 校准数据
    calibration.isCalibrated = false;
    calibration.currentGain = 0.001f; // 需要根据硬件调整
    calibration.voltageGain = 0.01f;  // 需要根据硬件调整
    calibration.encoderDirection = 1.0f;

    // 重置状态
    state.reset();
    command.clear();
    monitor.reset();
  }
};

}; // namespace wibot::motor