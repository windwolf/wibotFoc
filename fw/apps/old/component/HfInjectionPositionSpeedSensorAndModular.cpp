//
// Created by zhouj on 2022/12/5.
//

#include "HfInjectionPositionSpeedSensorAndModular.hpp"
#include "math.hpp"

namespace wibot::motor {
void HfInjectionPositionSpeedSensorAndModular::module(Motor &motor,
                                                      uint8_t &section,
                                                      Vector3f &d_abc,
                                                      Vector3f &u_abc,
                                                      float &d_sample) {
  _svpwm_modular.module(motor, section, d_abc, u_abc, d_sample);
  _theta += _config.sampleTime * k2PI / _config.injected_frequency;
  _theta = Math::circleNormalize(_theta);
  float sin, cos;
  Math::sincos(_theta, &sin, &cos);
}
void HfInjectionPositionSpeedSensorAndModular::setConfig(
    HfInjectionPositionSpeedSensorConfig &config) {
  _config = config;
}
} // namespace wibot::motor
