//
// Created by zhouj on 2022/11/17.
//

#include "VirtualPositionSpeedSensor.hpp"
#include "math.hpp"

namespace wibot::motor {
void VirtualPositionSpeedSensor::setPosition(float position) {
  _position = position;
}
void VirtualPositionSpeedSensor::setSpeed(float speed) { _speed = speed; }
Vector4f VirtualPositionSpeedSensor::getPositionSpeed(Motor &motor) {
  _position += _speed * _config.sampleTime;
  _position = Math::circleNormalize(_position);
  return Vector4f(_position * _config.polePairs, _speed * _config.polePairs,
                  _position, _speed);
}
void VirtualPositionSpeedSensor::calibrate(Motor &motor) {}
void VirtualPositionSpeedSensor::setConfig(
    VirtualPositionSpeedSensorConfig &config) {
  _config = config;
}
} // namespace wibot::motor
