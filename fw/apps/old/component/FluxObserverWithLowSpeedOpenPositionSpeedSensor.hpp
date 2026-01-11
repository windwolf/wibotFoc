//
// Created by zhouj on 2022/11/21.
//

#ifndef WWMOTOR_APP_MOTOR2_FLUXOBSERVERWITHLOWSPEEDOPENPOSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_FLUXOBSERVERWITHLOWSPEEDOPENPOSITIONSPEEDSENSOR_HPP_

#include "FluxObserverPositionSpeedSensor.hpp"
namespace wibot::motor {

struct FluxObserverWithLowSpeedOpenPositionSpeedSensorConfig {
    FluxObserverPositionSpeedSensorConfig config;
    float                                 low_speed_threshold;
};
class FluxObserverWithLowSpeedOpenPositionSpeedSensor
    : public AngleSpeedSensor {

public:
    void setConfig(FluxObserverWithLowSpeedOpenPositionSpeedSensorConfig& config);

   private:
    FluxObserverWithLowSpeedOpenPositionSpeedSensorConfig _config;

    FluxObserverPositionSpeedSensor _inner_sensor;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_FLUXOBSERVERWITHLOWSPEEDOPENPOSITIONSPEEDSENSOR_HPP_
