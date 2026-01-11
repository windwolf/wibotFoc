//
// Created by zhouj on 2022/12/13.
//

#ifndef WWMOTOR_APP_MOTOR2_DQVOLTAGEREFERENCEUPDATER_HPP_
#define WWMOTOR_APP_MOTOR2_DQVOLTAGEREFERENCEUPDATER_HPP_
#include "motor/model/motor.hpp"

namespace wibot::motor {
class DqVoltageController {
   public:
    virtual Vector2f updateDqVoltage(Motor& motor) = 0;
};
}  // namespace wibot::motor
#endif  // WWMOTOR_APP_MOTOR2_DQVOLTAGEREFERENCEUPDATER_HPP_
