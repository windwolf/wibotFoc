#include "PwmDriver.hpp"
#include "stm32g4xx_hal_tim_ex.h"
#include <algorithm>

namespace wibot::motor {

PwmDriver::PwmDriver(TIM_HandleTypeDef *htim, uint32_t period)
    : _htim(htim), _period(period), _isRunning(false),
      _complementaryEnabled(true) {}

PwmDriver::~PwmDriver() { stop(); }

bool PwmDriver::init() {
  if (_htim == nullptr) {
    return false;
  }

  // 设置 PWM 周期
  __HAL_TIM_SET_AUTORELOAD(_htim, _period);

  // 初始化三相占空比为 50%（空闲状态）
  setIdle();

  return true;
}

bool PwmDriver::start() {
  if (_htim == nullptr) {
    return false;
  }

  HAL_StatusTypeDef status;

  if (_complementaryEnabled) {
    // 启动带互补输出的 PWM
    status = HAL_TIMEx_PWMN_Start(_htim, kChannelU);
    if (status != HAL_OK)
      return false;

    status = HAL_TIMEx_PWMN_Start(_htim, kChannelV);
    if (status != HAL_OK)
      return false;

    status = HAL_TIMEx_PWMN_Start(_htim, kChannelW);
    if (status != HAL_OK)
      return false;
  }

  // 启动主 PWM 通道
  status = HAL_TIM_PWM_Start(_htim, kChannelU);
  if (status != HAL_OK)
    return false;

  status = HAL_TIM_PWM_Start(_htim, kChannelV);
  if (status != HAL_OK)
    return false;

  status = HAL_TIM_PWM_Start(_htim, kChannelW);
  if (status != HAL_OK)
    return false;

  // 启动 ADC 触发通道
  status = HAL_TIM_PWM_Start(_htim, kChannelAdcTrig);
  if (status != HAL_OK)
    return false;

  _isRunning = true;
  return true;
}

void PwmDriver::stop() {
  if (_htim == nullptr || !_isRunning) {
    return;
  }

  // 停止主 PWM 通道
  HAL_TIM_PWM_Stop(_htim, kChannelU);
  HAL_TIM_PWM_Stop(_htim, kChannelV);
  HAL_TIM_PWM_Stop(_htim, kChannelW);
  HAL_TIM_PWM_Stop(_htim, kChannelAdcTrig);

  if (_complementaryEnabled) {
    // 停止互补输出
    HAL_TIMEx_PWMN_Stop(_htim, kChannelU);
    HAL_TIMEx_PWMN_Stop(_htim, kChannelV);
    HAL_TIMEx_PWMN_Stop(_htim, kChannelW);
  }

  _isRunning = false;
}

void PwmDriver::setPeriod(uint32_t period) {
  _period = period;

  if (_htim != nullptr) {
    __HAL_TIM_SET_AUTORELOAD(_htim, _period);
  }
}

void PwmDriver::setDeadTime(uint32_t deadTime) {
  if (_htim == nullptr) {
    return;
  }
  // 设置死区时间 (需要根据具体硬件配置)
  // 这里提供接口，具体实现可能需要修改定时器寄存器
  // 通常在 CubeMX 中配置死区时间

  // 示例：修改 BDTR 寄存器的死区配置
  // _htim->Instance->BDTR &= ~TIM_BDTR_DTG;
  // _htim->Instance->BDTR |= (deadTime & TIM_BDTR_DTG);
}

void PwmDriver::emergencyBrake() {
  if (_htim == nullptr) {
    return;
  }

  // 方式1: 直接设置比较值为0（下桥臂导通）
  setCompare(kChannelU, 0);
  setCompare(kChannelV, 0);
  setCompare(kChannelW, 0);

  // 方式2: 使用定时器的刹车功能（如果硬件支持）
  // HAL_TIMEx_Break(_htim);
}

void PwmDriver::setIdle() {
  // 设置到中点电压（50% 占空比）
  setDuty(0.5f, 0.5f, 0.5f, 0.75);
}

void PwmDriver::enableComplementary(bool enable) {
  if (_complementaryEnabled == enable) {
    return;
  }

  bool wasRunning = _isRunning;

  // 如果正在运行，先停止
  if (wasRunning) {
    stop();
  }

  _complementaryEnabled = enable;

  // 如果之前在运行，重新启动
  if (wasRunning) {
    start();
  }
}

float PwmDriver::clampDuty(float duty) const {
  return std::max(kMinDuty, std::min(kMaxDuty, duty));
}

uint32_t PwmDriver::dutyToCompare(float duty) const {
  // 将占空比转换为定时器比较值
  // Compare = Period * Duty
  return static_cast<uint32_t>(_period * duty);
}

void PwmDriver::setDuty(float dutyU, float dutyV, float dutyW,
                        float dutyAdcTrig) {

  if (_htim == nullptr) {
    return;
  }

  // 限制占空比范围
  dutyU = clampDuty(dutyU);
  dutyV = clampDuty(dutyV);
  dutyW = clampDuty(dutyW);
  dutyAdcTrig = clampDuty(dutyAdcTrig);
  // 转换为定时器比较值
  uint32_t compareU = dutyToCompare(dutyU);
  uint32_t compareV = dutyToCompare(dutyV);
  uint32_t compareW = dutyToCompare(dutyW);
  uint32_t compareAdcTrig = dutyToCompare(dutyAdcTrig);
  // 设置比较值

  setCompare(kChannelU, compareU);
  setCompare(kChannelV, compareV);
  setCompare(kChannelW, compareW);
  setCompare(kChannelAdcTrig, compareAdcTrig);
}

} // namespace wibot::motor
