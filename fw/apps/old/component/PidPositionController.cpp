//
// Created by zhouj on 2022/11/17.
//

#include "PidPositionController.hpp"

namespace wibot::motor {
void PidPositionController::setConfig(PositionControllerConfig& config) {

    _pid_pos.config.mode                  = PidControllerMode::kSerial;
    _pid_pos.config.Kp                    = config.kp;
    _pid_pos.config.Ki                    = config.ki;
    _pid_pos.config.Kd                    = config.kd;
    _pid_pos.config.tau                   = 0;
    _pid_pos.config.outputLimitEnable     = true;
    _pid_pos.config.outputLimitMax        = config.motor_parameter->speedLimit / 60 * k2PI;
    _pid_pos.config.outputLimitMin        = -config.motor_parameter->speedLimit / 60 * k2PI;
    _pid_pos.config.integratorLimitEnable = true;
    _pid_pos.config.integratorLimitMax    = config.motor_parameter->speedLimit / 60 * k2PI;
    _pid_pos.config.integratorLimitMin    = -config.motor_parameter->speedLimit / 60 * k2PI;
    _pid_pos.config.sampleTime            = config.sampleTime;

    _config                          = config;
};

float PidPositionController::updateSpeed(Motor& motor) {
    return _pid_pos.update(motor.reference.position, motor.state.position.v2);
}

}  // namespace wibot::motor
