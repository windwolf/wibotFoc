#pragma once

#include "math/index.hpp"

namespace wibot::motor {

// SVPWM 扇区定义
enum class SvpwmSector : uint8_t {
  kSector1 = 1,
  kSector2 = 2,
  kSector3 = 3,
  kSector4 = 4,
  kSector5 = 5,
  kSector6 = 6,
};

// SVPWM 输出结果
struct SvpwmOutput {
  float dutyU;       // U相占空比 (0.0 ~ 1.0)
  float dutyV;       // V相占空比 (0.0 ~ 1.0)
  float dutyW;       // W相占空比 (0.0 ~ 1.0)
  float dutyAdcTrig; // ADC触发占空比 (0.0 ~ 1.0)
  uint8_t sector;    // 当前扇区 (1~6)
};

/**
 * @brief 空间矢量脉宽调制 (SVPWM) 算法类
 *
 * 硬件无关的 SVPWM 算法实现，负责将 α-β 坐标系的电压矢量
 * 转换为三相 PWM 占空比
 */
class Svpwm {
public:
  /**
   * @brief 构造函数
   * @param busVoltage 母线电压 (Volts)
   */
  explicit Svpwm(float busVoltage = 24.0f);
  ~Svpwm();

  /**
   * @brief 设置母线电压
   * @param voltage 母线电压 (Volts)
   */
  void setBusVoltage(float voltage);

  /**
   * @brief 获取母线电压
   * @return 母线电压 (Volts)
   */
  float getBusVoltage() const { return _busVoltage; }

  /**
   * @brief 计算 SVPWM 占空比
   *
   * 从 α-β 坐标系电压计算三相占空比
   *
   * @param vAlpha α轴电压 (Volts)
   * @param vBeta β轴电压 (Volts)
   * @return SVPWM 输出结果
   */
  SvpwmOutput calculate(float vAlpha, float vBeta);

  /**
   * @brief 计算 SVPWM 占空比（使用角度和幅值）
   *
   * @param angle 电压矢量角度 (弧度)
   * @param magnitude 电压矢量幅值 (Volts)
   * @return SVPWM 输出结果
   */
  SvpwmOutput calculatePolar(float angle, float magnitude);

  /**
   * @brief 判断电压矢量所在扇区
   * @param vAlpha α轴电压
   * @param vBeta β轴电压
   * @return 扇区编号 (1~6)
   */
  static uint8_t determineSector(float vAlpha, float vBeta);

  /**
   * @brief 获取最大可调制电压（线性区）
   * @return 最大电压 (Volts)
   */
  float getMaxModulationVoltage() const;

  /**
   * @brief 限制电压矢量幅值到线性区
   * @param vAlpha α轴电压引用
   * @param vBeta β轴电压引用
   */
  void limitVoltage(float &vAlpha, float &vBeta) const;

private:
  float _busVoltage; // 母线电压
  Math _math;        // 数学运算对象
};

} // namespace wibot::motor
