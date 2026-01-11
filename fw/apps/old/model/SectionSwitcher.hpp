//
// Created by zhouj on 2022/11/22.
//

#ifndef WWMOTOR_APP_MOTOR2_SECTIONSWITCHER_HPP_
#define WWMOTOR_APP_MOTOR2_SECTIONSWITCHER_HPP_

#include "motor/model/motor.hpp"
namespace wibot::motor {

/**
 * 6 step only
 */
class SectionSwitcher {
   public:
    virtual uint8_t switchSection(Motor& motor) = 0;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_SECTIONSWITCHER_HPP_
