//
// Created by zhouj on 2022/11/16.
//

#ifndef WWMOTOR_APP_MOTOR2_POWERSENSOR_CPP_SIMPLEPOWERSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_POWERSENSOR_CPP_SIMPLEPOWERSENSOR_HPP_

#include "../model/PowerSensor.hpp"

namespace wibot::motor {

struct SimplePowerSensorConfig {
    float u_bus;
};
class SimplePowerSensor : public PowerSensor {
   public:
    void setConfig(SimplePowerSensorConfig& config);
    float getBusVoltage(Motor& motor) override;
    float getBusCurrent(Motor& motor) override;

   private:
    SimplePowerSensorConfig _config;

};

}  // namespace wibot::motor
#endif  // WWMOTOR_APP_MOTOR2_POWERSENSOR_CPP_SIMPLEPOWERSENSOR_HPP_
