#pragma once

#include "stm32g4xx_hal.h"
#include "tim.h"
#include <cstdint>

namespace wibot::motor {

/**
 * @brief PWM 驱动器类 - 硬件相关的 PWM 生成
 *
 * 负责将 SVPWM 计算出的占空比实际应用到 STM32 定时器硬件上
 */
class PwmDriver {
public:
  /**
   * @brief 构造函数
   * @param htim 定时器句柄（通常是 TIM1 或 TIM8 的高级定时器）
   * @param period PWM 周期计数值
   */
  explicit PwmDriver(TIM_HandleTypeDef *htim, uint32_t period = 4200);

  ~PwmDriver();

  /**
   * @brief 初始化 PWM 定时器
   * @return true 成功, false 失败
   */
  bool init();

  /**
   * @brief 启动 PWM 输出
   * @return true 成功, false 失败
   */
  bool start();

  /**
   * @brief 停止 PWM 输出
   */
  void stop();

  /**
   * @brief 设置三相 PWM 占空比和 ADC 触发
   * @param dutyU U相占空比 (0.0 ~ 1.0)
   * @param dutyV V相占空比 (0.0 ~ 1.0)
   * @param dutyW W相占空比 (0.0 ~ 1.0)
   * @param dutyAdcTrig ADC触发占空比 (0.0 ~ 1.0)
   */
  void setDuty(float dutyU, float dutyV, float dutyW, float dutyAdcTrig);

  /**
   * @brief 设置 PWM 周期
   * @param period 周期计数值
   */
  void setPeriod(uint32_t period);

  /**
   * @brief 获取 PWM 周期
   * @return 周期计数值
   */
  uint32_t getPeriod() const { return _period; }

  /**
   * @brief 设置死区时间
   * @param deadTime 死区时间 (定时器计数值)
   */
  void setDeadTime(uint32_t deadTime);

  /**
   * @brief 紧急刹车 - 立即关闭所有 PWM 输出
   */
  void emergencyBrake();

  /**
   * @brief 设置 PWM 输出到空闲状态（50% 占空比）
   */
  void setIdle();

  /**
   * @brief 检查 PWM 是否运行
   * @return true 运行中, false 已停止
   */
  bool isRunning() const { return _isRunning; }

  /**
   * @brief 使能或禁用 PWM 互补输出
   * @param enable true 使能, false 禁用
   */
  void enableComplementary(bool enable);

private:
  /**
   * @brief 限制占空比到有效范围
   * @param duty 占空比 (0.0 ~ 1.0)
   * @return 限制后的占空比
   */
  float clampDuty(float duty) const;

  /**
   * @brief 将占空比转换为定时器比较值
   * @param duty 占空比 (0.0 ~ 1.0)
   * @return 定时器比较值
   */
  uint32_t dutyToCompare(float duty) const;

  /**
   * @brief 设置定时器比较值（辅助函数，避免 volatile 警告）
   * @param channel 定时器通道
   * @param compare 比较值
   */
  inline void setCompare(uint32_t channel, uint32_t compare) {
    if (channel == TIM_CHANNEL_1)
      _htim->Instance->CCR1 = compare;
    else if (channel == TIM_CHANNEL_2)
      _htim->Instance->CCR2 = compare;
    else if (channel == TIM_CHANNEL_3)
      _htim->Instance->CCR3 = compare;
    else if (channel == TIM_CHANNEL_4)
      _htim->Instance->CCR4 = compare;
    else if (channel == TIM_CHANNEL_5)
      _htim->Instance->CCR5 = compare;
    else if (channel == TIM_CHANNEL_6)
      _htim->Instance->CCR6 = compare;
  }

  TIM_HandleTypeDef *_htim;   // 定时器句柄
  uint32_t _period;           // PWM 周期
  bool _isRunning;            // 运行状态标志
  bool _complementaryEnabled; // 互补输出使能标志

  // 三相 PWM 通道定义
  static constexpr uint32_t kChannelU = TIM_CHANNEL_1;
  static constexpr uint32_t kChannelV = TIM_CHANNEL_2;
  static constexpr uint32_t kChannelW = TIM_CHANNEL_3;
  static constexpr uint32_t kChannelAdcTrig = TIM_CHANNEL_4; // ADC触发通道

  // 占空比限制
  static constexpr float kMinDuty = 0.0f;
  static constexpr float kMaxDuty = 1.0f;
};

} // namespace wibot::motor
