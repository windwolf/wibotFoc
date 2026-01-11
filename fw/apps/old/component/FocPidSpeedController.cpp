//
// Created by zhouj on 2022/11/17.
//

#include "FocPidSpeedController.hpp"
namespace wibot::motor {

void FocPidSpeedController::setConfig(FocPidSpeedControllerConfig& config) {
    float k = (float)config.motor_parameter->polePair * config.motor_parameter->flux /
              config.motor_parameter->interia * 3.0f / 2.0f;
    _pidSpeed.config.mode                  = PidControllerMode::kSerial;
    _pidSpeed.config.Kp                    = config.bandWidth / config.delta / k;
    _pidSpeed.config.Ki                    = config.bandWidth / config.delta / config.delta;
    _pidSpeed.config.Kd                    = 0;
    _pidSpeed.config.tau                   = 0;
    _pidSpeed.config.outputLimitEnable     = true;
    _pidSpeed.config.outputLimitMax        = config.motor_parameter->speedLimit / 60 * k2PI;
    _pidSpeed.config.outputLimitMin        = -config.motor_parameter->speedLimit / 60 * k2PI;
    _pidSpeed.config.integratorLimitEnable = true;
    _pidSpeed.config.integratorLimitMax    = config.motor_parameter->speedLimit / 60 * k2PI;
    _pidSpeed.config.integratorLimitMin    = -config.motor_parameter->speedLimit / 60 * k2PI;
    _pidSpeed.config.sampleTime            = config.sampleTime;

    _config = config;
}
Vector2f FocPidSpeedController::updateDqCurrent(Motor& motor) {
    Vector2f i_dq;
    i_dq.v1 = 0.0f;
    i_dq.v2 = _pidSpeed.update(motor.reference.speed, motor.state.speed.v2);
    return i_dq;
}

}  // namespace wibot::motor
