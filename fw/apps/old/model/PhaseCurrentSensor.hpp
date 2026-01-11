//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_PHASECURRENTSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_PHASECURRENTSENSOR_HPP_

#include "motor/model/motor.hpp"
namespace wibot::motor {

class PhaseCurrentSensor {
   public:
    /**
     * @brief Get the phase current of the motor.
     * @param motor
     * @param i_abc
     */
    virtual Vector3f getAbcCurrent(wibot::motor::Motor& motor) = 0;
    virtual Vector2f getAbCurrent(wibot::motor::Motor& motor)   = 0;
    virtual void calibrate(wibot::motor::Motor& motor)                  = 0;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_PHASECURRENTSENSOR_HPP_
