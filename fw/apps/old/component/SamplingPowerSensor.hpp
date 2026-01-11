//
// Created by zhouj on 2022/11/16.
//

#ifndef WWMOTOR_APP_MOTOR2_SAMPLINGPOWERSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_SAMPLINGPOWERSENSOR_HPP_

#include "motor/model/PowerSensor.hpp"
#include "control/mapper/linear-value-mapper.hpp"

namespace wibot::motor {

struct SamplingPowerSensorConfig {
    uint32_t* uBusData;
    uint32_t* iBusData;
    float     u_bus_pu;
    float     i_bus_pu;
};
class SamplingPowerSensor : public PowerSensor {
   public:
    void setConfig(SamplingPowerSensorConfig& config);
    float   getBusVoltage(Motor& motor) override;
    float   getBusCurrent(Motor& motor) override;

   private:
    SamplingPowerSensorConfig _config;
    LinearValueMapper u_bus_mapper;
    LinearValueMapper i_bus_mapper;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_SAMPLINGPOWERSENSOR_HPP_
