//
// Created by zhouj on 2022/11/17.
//

#include "DirectSectionSensor.hpp"

#include "../util/FocMath.hpp"

namespace wibot::motor {
uint8_t DirectSectionSensor::getSection(Motor& motor) {
    return FocMath::getSection(motor.state.position.v1);
}

}  // namespace wibot::motor
