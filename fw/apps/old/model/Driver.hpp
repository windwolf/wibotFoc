//
// Created by zhouj on 2022/11/18.
//

#ifndef WWMOTOR_APP_MOTOR2_DRIVER_HPP_
#define WWMOTOR_APP_MOTOR2_DRIVER_HPP_

#include "motor/model/motor.hpp"

namespace wibot::motor {

class Driver {
   public:
    virtual void setDuty(Motor& motor) = 0;
    virtual void breakdown()           = 0;
    virtual void resume()              = 0;
    virtual void prepareCharge()       = 0;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_DRIVER_HPP_
