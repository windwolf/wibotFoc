//
// Created by zhouj on 2022/11/28.
//

#ifndef WWMOTOR_APP_MOTOR2_PHASEVOLTAGESENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_PHASEVOLTAGESENSOR_HPP_

#include "motor/model/motor.hpp"
namespace wibot::motor {

class PhaseVoltageSensor {
   public:
    /**
     * @brief Get the mid port voltage of the motor.
     * @param motor
     * @param u_abc
     */
    virtual Vector3f getAbcVoltage(wibot::motor::Motor& motor) = 0;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_PHASEVOLTAGESENSOR_HPP_
