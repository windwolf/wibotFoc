//
// Created by zhouj on 2022/11/28.
//

#ifndef WWMOTOR_APP_MOTOR2_SAMPLINGPHASEVOLTAGESENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_SAMPLINGPHASEVOLTAGESENSOR_HPP_

#include "motor/model/motor.hpp"
#include "motor/model/PhaseVoltageSensor.hpp"
#include "control/mapper/linear-value-mapper.hpp"
#include "control/filter/lp.hpp"

namespace wibot::motor {

struct SamplingPhaseVoltageSensorConfig {
    uint32_t* u_a;
    uint32_t* u_b;
    uint32_t* u_c;

    float cutoffFreq;  // 务必大于最大电频率.
    float sampleTime;

    float u_pu;
};

class SamplingPhaseVoltageSensor : public PhaseVoltageSensor{
   public:
    void setConfig(SamplingPhaseVoltageSensorConfig& config);

    Vector3f getAbcVoltage(wibot::motor::Motor& motor) override;

   private:
    SamplingPhaseVoltageSensorConfig _config;
    LinearValueMapper       _a_mapper;
    LinearValueMapper       _b_mapper;
    LinearValueMapper       _c_mapper;
    FirstOrderLowPassFilter _a_filter;
    FirstOrderLowPassFilter _b_filter;
    FirstOrderLowPassFilter _c_filter;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_SAMPLINGPHASEVOLTAGESENSOR_HPP_
