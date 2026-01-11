//
// Created by zhouj on 2022/11/17.
//

#include "Shunt3PhaseCurrentSensor.hpp"
#include "os.hpp"

namespace wibot::motor {

/**
 * sec 1: a > b > c;
 * sec 2: b > a > c;
 * sec 3: b > c > a;
 * sec 4: c > b > a;
 * sec 5: c > a > b;
 * sec 6: a > c > b;
 * @param motor
 * @param i_abc
 */
Vector3f Shunt3PhaseCurrentSensor::getAbcCurrent(wibot::motor::Motor& motor) {
    Vector3f i_abc;
    i_abc.v1 = _a_mapper.getValue(*_config.i_a);
    i_abc.v2 = _b_mapper.getValue(*_config.i_b);
    i_abc.v3 = _c_mapper.getValue(*_config.i_c);
    if (motor.reference.dAbc.v1 > _config.low_duty_skip_threshold ||
        motor.reference.dAbc.v2 > _config.low_duty_skip_threshold ||
        motor.reference.dAbc.v3 > _config.low_duty_skip_threshold) {
        switch (motor.reference.section) {
            case 1:
            case 6:
                i_abc.v1 = -i_abc.v2 - i_abc.v3;
                break;
            case 2:
            case 3:
                i_abc.v2 = -i_abc.v1 - i_abc.v3;
                break;
            case 4:
            case 5:
                i_abc.v3 = -i_abc.v1 - i_abc.v2;
                break;
        }
    } else {
        // 根据a+b+c=0的原则, 做信号滤波.
        float mid = (1.f / 3) * (i_abc.v1 + i_abc.v2 + i_abc.v3);
        i_abc.v1 -= mid;
        i_abc.v2 -= mid;
        i_abc.v3 -= mid;
    }
    return i_abc;
}

void Shunt3PhaseCurrentSensor::setConfig(Shunt3PhaseCurrentSensorConfig& config) {
    ASSERT(_config.i_a == nullptr || _config.i_b == nullptr || _config.i_c == nullptr, "Current pointer is null.");

    _a_mapper.config.zeroOffset   = _config.i_a_offset;
    _a_mapper.config.valuePerUnit = _config.i_pu;

    _b_mapper.config.zeroOffset   = _config.i_b_offset;
    _b_mapper.config.valuePerUnit = _config.i_pu;

    _c_mapper.config.zeroOffset   = _config.i_c_offset;
    _c_mapper.config.valuePerUnit = _config.i_pu;

    _config = config;
}

void Shunt3PhaseCurrentSensor::calibrate(Motor& motor) {
    const uint16_t calibrationRound = 100;
    Vector3f       o_d_abc          = motor.reference.dAbc;
    float          o_d_s            = motor.reference.dSample;
    motor.reference.dAbc           = {0, 0, 0};
    motor.reference.dSample        = 0.5;

    uint32_t i_a_sum = 0;
    uint32_t i_b_sum = 0;
    uint32_t i_c_sum = 0;

    for (int i = 0; i < calibrationRound; ++i) {
        i_a_sum += *_config.i_a;
        i_b_sum += *_config.i_b;
        i_c_sum += *_config.i_c;
        Thread::sleep(1);
    }

    _a_mapper.config.zeroOffset = _config.i_a_offset = i_a_sum / calibrationRound;

    _b_mapper.config.zeroOffset = _config.i_b_offset = i_b_sum / calibrationRound;

    _c_mapper.config.zeroOffset = _config.i_c_offset = i_c_sum / calibrationRound;

    motor.reference.dAbc    = o_d_abc;
    motor.reference.dSample = o_d_s;
}
/**
 * sec 1: a > b > c;
 * sec 2: b > a > c;
 * sec 3: b > c > a;
 * sec 4: c > b > a;
 * sec 5: c > a > b;
 * sec 6: a > c > b;
 * @param motor
 * @param i_dq
 */
Vector2f Shunt3PhaseCurrentSensor::getAbCurrent(wibot::motor::Motor& motor) {
    Vector3f abc;
    Vector2f ab;
    abc.v1 = _a_mapper.getValue(*_config.i_a);
    abc.v2 = _b_mapper.getValue(*_config.i_b);
    abc.v3 = _c_mapper.getValue(*_config.i_c);
    if (motor.reference.dAbc.v1 > _config.low_duty_skip_threshold ||
        motor.reference.dAbc.v2 > _config.low_duty_skip_threshold ||
        motor.reference.dAbc.v3 > _config.low_duty_skip_threshold) {
        switch (motor.reference.section) {
            case 1:
            case 6:
                abc.v1 = -abc.v3 - abc.v2;
                break;
            case 2:
            case 3:
                // if only two measured currents
                abc.v2 = -abc.v1 - abc.v3;
            case 4:
            case 5:
                break;
        }
        ab.v1 = abc.v1;
        ab.v2 = abc.v2;
    } else {
        float mid = k1_3 * (abc.v1 + abc.v2 + abc.v3);
        ab.v1     = abc.v1 - mid;
        ab.v2     = abc.v2 - mid;
    }
    return Vector2f(ab.v1,
                    k1_SQRT3 * ab.v1 + k2_SQRT3 * ab.v2);
}

}  // namespace wibot::motor
