//
// Created by zhouj on 2022/11/17.
//

#include "FluxObserverPositionSpeedSensor.hpp"
#include "../util/FocMath.hpp"

namespace wibot::motor {
void FluxObserverPositionSpeedSensor::setConfig(FluxObserverPositionSpeedSensorConfig& config) {

    _a = exp(-1.0f * _config.motor_parameter->rs / _config.motor_parameter->ld * _config.sampleTime);
    _b = (1 - _a) / _config.motor_parameter->rs;
    _pid.config.mode                  = PidControllerMode::kSerial;
    _pid.config.Kp                    = _config.pllKp;
    _pid.config.Ki                    = _config.pllPi;
    _pid.config.Kd                    = 0;
    _pid.config.tau                   = 0;
    _pid.config.outputLimitEnable     = true;
    _pid.config.outputLimitMax        = _config.motor_parameter->speedLimit / 60 * k2PI;
    _pid.config.outputLimitMin        = -_config.motor_parameter->speedLimit / 60 * k2PI;
    _pid.config.integratorLimitEnable = true;
    _pid.config.integratorLimitMax    = _config.motor_parameter->speedLimit / 60 * k2PI;
    _pid.config.integratorLimitMin    = -_config.motor_parameter->speedLimit / 60 * k2PI;
    _pid.config.sampleTime            = _config.sampleTime;

    FirstOrderLowPassFilterConfig lpCfg;

    lpCfg.sampleTime = _config.sampleTime;
    lpCfg.cutoffFreq = _config.cutoffFreq;
    _filter.setConfig(lpCfg);

    _config = config;
}
Vector4f FluxObserverPositionSpeedSensor::getPositionSpeed(Motor& motor) {

    // SMO
    FocMath::abc2ab(motor.state.iAbc, _i);
    FocMath::abc2ab(motor.state.uAbc, _u);

    Vector2f i_err_now = _i_obs - _i;
    Vector2f zk        = Math::sign(i_err_now) * _config.currentGain;
    _e_obs             = (i_err_now - _i_err * _a + _zk) * _b + _e_obs;

    _i_obs = (_u - _e_obs) * _b - zk + _i_obs * _a;
    _zk    = zk;
    _i_err = i_err_now;

    // PLL
    float e_alpha = _filter.filter(_e_obs.v1);
    float e_beta  = _filter.filter(_e_obs.v2);

    float sin, cos;
    Math::sincos(motor.state.position.v1, &sin, &cos);

    float err = e_alpha * cos + e_beta * sin;  // 下一步pid作用在0-sum上, 所以这里不用取负值了
    auto speed = _pid.update(0, err);
    auto pos = motor.state.position.v1 + speed * _config.sampleTime;

    return Vector4f (
        pos,
        speed,
        pos / _config.motor_parameter->polePair,
        speed / _config.motor_parameter->polePair
        );
}
void FluxObserverPositionSpeedSensor::calibrate(Motor& motor) {
}

}  // namespace wibot::motor
