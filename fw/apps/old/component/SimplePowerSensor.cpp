//
// Created by zhouj on 2022/11/16.
//

#include "SimplePowerSensor.hpp"
float wibot::motor::SimplePowerSensor::getBusVoltage(wibot::motor::Motor& motor) {
    return _config.u_bus;
}
float wibot::motor::SimplePowerSensor::getBusCurrent(wibot::motor::Motor& motor) {
    return 0;
}
void wibot::motor::SimplePowerSensor::setConfig(wibot::motor::SimplePowerSensorConfig& config) {
    _config = config;
}
