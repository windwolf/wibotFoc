//
// Created by zhouj on 2022/11/16.
//

#include "SamplingPowerSensor.hpp"
#include "base.hpp"

namespace wibot::motor {
void SamplingPowerSensor::setConfig(SamplingPowerSensorConfig& config) {
    ASSERT(config.uBusData == nullptr || config.iBusData == nullptr, "Invalid parameter");
        u_bus_mapper.config.zeroOffset    = 0;
        u_bus_mapper.config.valuePerUnit = config.u_bus_pu;

        i_bus_mapper.config.zeroOffset    = 0;
        i_bus_mapper.config.valuePerUnit = config.i_bus_pu;
    _config = config;
}

float SamplingPowerSensor::getBusVoltage(Motor& motor) {
    return u_bus_mapper.getValue(*_config.uBusData);

}

float SamplingPowerSensor::getBusCurrent(Motor& motor) {
    return i_bus_mapper.getValue(*_config.iBusData);
}

}  // namespace wibot::motor
