//
// Created by zhouj on 2022/11/17.
//

#include "AbsoluteEncoderAngleSpeedSensor.hpp"
#include "os.hpp"

namespace wibot::motor {

#define CALIBRATION_SAMPLING_ROUND 50

void AbsoluteEncoderAngleSpeedSensor::setConfig(
    AbsoluteEncoderAngleSpeedSensorConfig& config) {
    ASSERT(_config.codex == nullptr, "codex should be nullptr when set config.");

    this->_2PI_RES = k2PI / (float)_config.resolution;
    this->_1_TS    = 1.0f / _config.sampleTime;
    this->_PP_TS   = (float)_config.polePairs * this->_1_TS;

    _mapper.config.inWrap  = _config.resolution;
    _mapper.config.outWrap = k2PI;

    FirstOrderLowPassFilterConfig lpcfg;
    lpcfg.sampleTime = _config.sampleTime;
    lpcfg.cutoffFreq = _config.mechanicalPostionCutoffFreq * k2PI;
    lpcfg.wrapValue  = kPI;
    _filteredMechanicalPosition.setConfig(lpcfg);

    lpcfg.sampleTime = _config.sampleTime;
    lpcfg.cutoffFreq = _config.mechanicalPostionCutoffFreq * k2PI * (float)_config.polePairs;
    lpcfg.wrapValue  = kPI * _config.polePairs;
    _filteredElectricalPosition.setConfig(lpcfg);

    lpcfg.sampleTime = _config.sampleTime;
    lpcfg.cutoffFreq = _config.mechanicalSpeedCutoffFreq;
    lpcfg.wrapValue  = 0;
    _filteredMechanicalSpeed.setConfig(lpcfg);

    lpcfg.sampleTime = _config.sampleTime;
    lpcfg.cutoffFreq = _config.mechanicalSpeedCutoffFreq * (float)_config.polePairs;
    lpcfg.wrapValue  = 0;
    _filteredElectricalSpeed.setConfig(lpcfg);

    _config = config;
}

Vector2f AbsoluteEncoderAngleSpeedSensor::getAngleSpeed(Motor& motor) {
    Vector4f rtn;
    // TODO: 电角度计算是否有误.
    auto pos_m_raw_i = _config.codex->getData();

    // process mech pos
    float pos_raw_m       = _mapper.getValue(pos_m_raw_i);  // in rad. 0 ~ 2pi
    float pos_raw_shift_m = pos_raw_m - kPI;                // in rad. -pi ~ pi
    if (_config.direction == EncoderDirection::Reverse) {
        pos_raw_shift_m = -pos_raw_shift_m;
    }

    float pos_m_raw_diff = pos_raw_shift_m - _lastRawMechanicalPosition;
    _lastRawMechanicalPosition = pos_raw_shift_m;
    if (pos_m_raw_diff > kPI) {
        pos_m_raw_diff -= k2PI;
        _mechanicalRound--;
    } else if (pos_m_raw_diff < -kPI) {
        pos_m_raw_diff += k2PI;
        _mechanicalRound++;
    }
    _lastMechanicalPosition = pos_raw_shift_m + kPI;
    pos_raw_shift_m = _filteredMechanicalPosition.filter(pos_raw_shift_m);
    rtn.v3     = Math::circleNormalize(pos_raw_shift_m + kPI);
    rtn.v4        = _filteredMechanicalSpeed.filter(pos_m_raw_diff * _1_TS);

    // process elec pos
    float pos_raw_e       = pos_raw_m * (float)_config.polePairs;  // in rad. 0 ~ 2pi*PP
    float pos_raw_shift_e = pos_raw_e - kPI * _config.polePairs;   // in rad. -pi*PP ~ pi*PP
    if (_config.direction == EncoderDirection::Reverse) {
        pos_raw_shift_e = -pos_raw_shift_e;
    }
    pos_raw_shift_e = _filteredElectricalPosition.filter(pos_raw_shift_e);
    rtn.v1     = Math::circleNormalize(pos_raw_shift_e + kPI * _config.polePairs);
    rtn.v2       = _filteredElectricalSpeed.filter(pos_m_raw_diff * _PP_TS);

    return rtn;
}

float AbsoluteEncoderAngleSpeedSensor::getPositionNowrap() const {
    return _mechanicalRound * k2PI + _lastRawMechanicalPosition;
}
float AbsoluteEncoderAngleSpeedSensor::getPositionWithoutFilter() const {
    return _lastMechanicalPosition;
}
void AbsoluteEncoderAngleSpeedSensor::calibrate(Motor& motor) {
    Vector2f o_u_dq = motor.reference.uDq;
    Vector2f o_pos  = motor.state.position;
    Vector2f o_spd  = motor.state.speed;

    motor.reference.uDq.v1 = _config.calibration_voltage;
    motor.reference.uDq.v2 = 0.0f;
    motor.state.position.v1 = 0.0f;
    Thread::sleep(100);

    // set zero offset
    rotate(motor, 0.0f, k2PI, 200, 10);
    Thread::sleep(10);
    rotate(motor, 0.0f, -k2PI, 200, 10);
    Thread::sleep(10);

    uint32_t zero_offset = 0;
    for (int i = 0; i < CALIBRATION_SAMPLING_ROUND; i++) {
        zero_offset += _config.codex->getData();
        Thread::sleep(2);
    }
    zero_offset /= CALIBRATION_SAMPLING_ROUND;
    this->_mapper.setZeroOffset(zero_offset);

    this->_mapper.beginCalirate();
    _mapper.calibrate(0, zero_offset, 0);

    float output_res = k2PI / ENCODER_MAPPER_POINT_COUNT;
    int   i          = 1;
    // rotate 360 degree forward
    for (; i < ENCODER_MAPPER_POINT_COUNT; ++i) {
        rotate(motor, output_res * (i - 1), output_res, 10, 10);
        sampleData(motor, i);
    }
    // i = PC - 1; rotate to 0 degree
    rotate(motor, output_res * i, output_res, 10, 10);

    // rotate 360 degree backward
    for (; i > 0; --i) {
        rotate(motor, output_res * (i + 1), -output_res, 10, 10);
        sampleData(motor, i);
    }
    _mapper.endCalibrate();

    motor.reference.uDq  = o_u_dq;
    motor.state.angle = o_pos;
    motor.state.speed    = o_spd;
}

void AbsoluteEncoderAngleSpeedSensor::sampleData(const Motor& motor, uint32_t step) {
    for (uint32_t r = 0; r < CALIBRATION_SAMPLING_ROUND; r++) {
        auto raw = _config.codex->getData();
        _mapper.calibrate(step, raw, motor.state.position.v2);
        Thread::sleep(10);
    }
}

void AbsoluteEncoderAngleSpeedSensor::rotate(Motor& motor, float startPosition, float travel,
                                                uint32_t step, uint32_t stepDelay) const {
    for (uint32_t j = 0; j < step + 1; j++) {
        motor.state.position.v2 = Math::circleNormalize(startPosition + travel / step * j);
        motor.state.position.v1 =
            Math::circleNormalize(motor.state.position.v2 * _config.polePairs);
        Thread::sleep(stepDelay);
    }
}

}  // namespace wibot::motor
