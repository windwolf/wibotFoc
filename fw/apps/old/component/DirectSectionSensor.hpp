//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_DIRECTSECTIONSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_DIRECTSECTIONSENSOR_HPP_

#include "../model/SectionSensor.hpp"
namespace wibot::motor {

class DirectSectionSensor : public SectionSensor {
   public:
    uint8_t getSection(Motor& motor) override;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_DIRECTSECTIONSENSOR_HPP_
