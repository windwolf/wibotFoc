//
// Created by zhouj on 2022/12/13.
//

#ifndef WWMOTOR_APP_MOTOR2_SPEEDREFERENCEUPDATER_HPP_
#define WWMOTOR_APP_MOTOR2_SPEEDREFERENCEUPDATER_HPP_
#include "motor/model/motor.hpp"

namespace wibot::motor {
class SpeedController {
   public:
    virtual float updateSpeed(Motor& motor) = 0;
};

}  // namespace wibot::motor
#endif  // WWMOTOR_APP_MOTOR2_SPEEDREFERENCEUPDATER_HPP_
