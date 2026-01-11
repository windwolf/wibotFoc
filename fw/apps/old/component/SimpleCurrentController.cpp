//
// Created by zhouj on 2023/1/3.
//

#include "SimpleCurrentController.hpp"

namespace wibot::motor {
Vector2f SimpleCurrentController::updateDqVoltage(Motor& motor) {
    return motor.reference.iDq * _config.rs;
}
void SimpleCurrentController::setConfig(SimpleCurrentControllerConfig& config) {
    _config = config;
}

}  // namespace wibot::motor
