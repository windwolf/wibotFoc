//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_FLUXOBSERVERPOSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_FLUXOBSERVERPOSITIONSPEEDSENSOR_HPP_

#include "motor/model/AngleSpeedSensor.hpp"
#include "filter/lp.hpp"
#include "pid/pid.hpp"

namespace wibot::motor {

struct FluxObserverPositionSpeedSensorConfig {
    float           sampleTime;
    float           currentGain;
    float           disturbanceGain;
    float           cutoffFreq;
    float           pllKp;
    float           pllPi;
    MotorParameter* motor_parameter;
};

class FluxObserverPositionSpeedSensor : public AngleSpeedSensor {
   public:
    void setConfig(FluxObserverPositionSpeedSensorConfig& config);

    Vector4f getPositionSpeed(Motor& motor) override;

    void calibrate(Motor& motor);

   private:
    FluxObserverPositionSpeedSensorConfig _config;
    FirstOrderLowPassFilter _filter;
    PidController           _pid;

    Vector2f _i;
    Vector2f _u;

    Vector2f _i_obs;
    Vector2f _zk;
    Vector2f _i_err;

    Vector2f _e_obs;

    Vector2f _positionSpeed;
    float    _a;
    float    _b;

    void smo();
    void pll();
    void lpf();
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_FLUXOBSERVERPOSITIONSPEEDSENSOR_HPP_
