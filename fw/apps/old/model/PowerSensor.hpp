//
// Created by zhouj on 2022/11/16.
//

#ifndef WWMOTOR_APP_MOTOR2_POWERSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_POWERSENSOR_HPP_

#include "motor/model/motor.hpp"
namespace wibot::motor {

class PowerSensor {
   public:
    virtual float getBusVoltage(Motor& motor) = 0;
    virtual float getBusCurrent(Motor& motor) = 0;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_POWERSENSOR_HPP_
