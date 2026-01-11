//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_SECTIONSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_SECTIONSENSOR_HPP_

#include "motor.hpp"

namespace wibot::motor {

class SectionSensor {
   public:
    virtual uint8_t getSection(Motor& motor) = 0;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_SECTIONSENSOR_HPP_
