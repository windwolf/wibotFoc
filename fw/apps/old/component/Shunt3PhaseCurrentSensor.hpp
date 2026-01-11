//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_SHUNT3PHASECURRENTSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_SHUNT3PHASECURRENTSENSOR_HPP_

#include "motor/model/PhaseCurrentSensor.hpp"
#include "control/mapper/linear-value-mapper.hpp"

namespace wibot::motor {

struct Shunt3PhaseCurrentSensorConfig {
    uint32_t* i_a;
    uint32_t* i_b;
    uint32_t* i_c;

    uint32_t i_a_offset;
    uint32_t i_b_offset;
    uint32_t i_c_offset;
    /**
     * 每刻度电流值.
     * 参考电压 / 最大量程 / 放大器倍数 / 采样电阻
     * 例如: 3.3 / 4096 / 1 / 0.1 = 0.0081
     * 放大倍数的选择, 尽量使可用的最大电流范围满量程.
     * 放大倍数 = 参考电压 / 采样电阻 / 最大电流
     */
    float    i_pu;
    /**
     * 当某相的占空比大于该阈值时, 将根据所在扇区忽略该相采用, 使用数值重构.
     */
    float    low_duty_skip_threshold;
};
class Shunt3PhaseCurrentSensor : public PhaseCurrentSensor {
   public:
    void setConfig(Shunt3PhaseCurrentSensorConfig& config);

    Vector3f getAbcCurrent(wibot::motor::Motor& motor) override;
    Vector2f getAbCurrent(wibot::motor::Motor& motor) override;

    /**
     * Calibrate the phase current sensor.
     * @note Make sure the Controll is in CALIBRATE state, during the calibration.
     * @param motor
     */
    void calibrate(wibot::motor::Motor& motor) override;

   private:
    Shunt3PhaseCurrentSensorConfig _config;
    LinearValueMapper _a_mapper;
    LinearValueMapper _b_mapper;
    LinearValueMapper _c_mapper;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_SHUNT3PHASECURRENTSENSOR_HPP_
