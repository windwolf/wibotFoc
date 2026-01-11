//
// Created by zhouj on 2022/11/28.
//

#include "SamplingPhaseVoltageSensor.hpp"
#include "base.hpp"

namespace wibot::motor {
void SamplingPhaseVoltageSensor::setConfig(SamplingPhaseVoltageSensorConfig& config) {
    ASSERT(config.u_a == nullptr || config.u_b == nullptr || config.u_c == nullptr, "Invalid parameter");

    _a_mapper.config.zeroOffset   = 0;
    _a_mapper.config.valuePerUnit = config.u_pu;

    _b_mapper.config.zeroOffset   = 0;
    _b_mapper.config.valuePerUnit = config.u_pu;

    _c_mapper.config.zeroOffset   = 0;
    _c_mapper.config.valuePerUnit = config.u_pu;

    FirstOrderLowPassFilterConfig lpCfg;
    lpCfg.cutoffFreq = config.cutoffFreq;
    lpCfg.sampleTime = config.sampleTime;
    _a_filter.setConfig(lpCfg);

    lpCfg.cutoffFreq = config.cutoffFreq;
    lpCfg.sampleTime = config.sampleTime;
    _b_filter.setConfig(lpCfg);

    lpCfg.cutoffFreq = config.cutoffFreq;
    lpCfg.sampleTime = config.sampleTime;
    _c_filter.setConfig(lpCfg);

    _config = config;
}

Vector3f SamplingPhaseVoltageSensor::getAbcVoltage(wibot::motor::Motor& motor) {
    Vector3f u;
    u.v1     = _a_mapper.getValue(*_config.u_a);
    u.v2     = _b_mapper.getValue(*_config.u_b);
    u.v3     = _c_mapper.getValue(*_config.u_c);
    u.v1 = _a_filter.filter(u.v1);
    u.v2 = _b_filter.filter(u.v2);
    u.v3 = _c_filter.filter(u.v3);
    return u;
}
}  // namespace wibot::motor
