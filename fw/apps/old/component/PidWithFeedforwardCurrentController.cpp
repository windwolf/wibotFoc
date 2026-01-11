//
// Created by zhouj on 2022/11/17.
//

#include "PidWithFeedforwardCurrentController.hpp"

namespace wibot::motor {

Vector2f PidWithFeedforwardCurrentController::updateDqVoltage(Motor &motor) {
    Vector2f udq;
    udq.v1 = pid_d.update(motor.reference.iDq.v1, motor.state.iDq.v1);
    udq.v2 = pid_q.update(motor.reference.iDq.v2, motor.state.iDq.v2);

    if (!_config.disableFeedforward) {
        udq.v1 += -motor.state.iDq.v2 * _config.motor_parameter->lq * motor.state.speed.v1;
        udq.v2 +=
            +(motor.state.iDq.v1 * _config.motor_parameter->ld + _config.motor_parameter->flux) *
            motor.state.speed.v1;
    }
    return udq;
}

Result PidWithFeedforwardCurrentController::setConfig(DqCurrentControllerConfig& config) {
    Result rst;
    pid_d.config.mode = PidControllerMode::kSerial;
    if (config.useParams) {
        pid_d.config.Kp = config.params.bandWidth * config.motor_parameter->ld;
        pid_d.config.Ki = config.motor_parameter->rs / config.motor_parameter->ld;
        pid_d.config.Kd = 0;
    } else {
        pid_d.config.Kp = config.simple.p;
        pid_d.config.Ki = config.simple.i;
        pid_d.config.Kd = config.simple.d;
    }
    pid_d.config.tau                   = 0;
    pid_d.config.outputLimitEnable     = true;
    pid_d.config.outputLimitMax        = config.motor_parameter->uBusMax;
    pid_d.config.outputLimitMin        = -config.motor_parameter->uBusMax;
    pid_d.config.integratorLimitEnable = true;
    pid_d.config.integratorLimitMax    = config.motor_parameter->uBusMax;
    pid_d.config.integratorLimitMin    = -config.motor_parameter->uBusMax;
    pid_d.config.sampleTime            = config.sampleTime;

    pid_q.config.mode = PidControllerMode::kSerial;
    if (config.useParams) {
        pid_q.config.Kp = config.params.bandWidth * config.motor_parameter->lq;
        pid_q.config.Ki = config.motor_parameter->rs / config.motor_parameter->lq;
        pid_q.config.Kd = 0;
    } else {
        pid_q.config.Kp = config.simple.p;
        pid_q.config.Ki = config.simple.i;
        pid_q.config.Kd = config.simple.d;
    }

    pid_q.config.tau                   = 0;
    pid_q.config.outputLimitEnable     = true;
    pid_q.config.outputLimitMax        = config.motor_parameter->uBusMax;
    pid_q.config.outputLimitMin        = -config.motor_parameter->uBusMax;
    pid_q.config.integratorLimitEnable = true;
    pid_q.config.integratorLimitMax    = config.motor_parameter->uBusMax;
    pid_q.config.integratorLimitMin    = -config.motor_parameter->uBusMax;
    pid_q.config.sampleTime            = config.sampleTime;

    _config = config;
}

}  // namespace wibot::motor
