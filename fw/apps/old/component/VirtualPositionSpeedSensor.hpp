//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_VIRTUALPOSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_VIRTUALPOSITIONSPEEDSENSOR_HPP_

#include "motor/model/AngleSpeedSensor.hpp"

namespace wibot::motor {
struct VirtualPositionSpeedSensorConfig {
    uint8_t polePairs;
    float   sampleTime;
};

class VirtualPositionSpeedSensor : public AngleSpeedSensor {
   public:
    void setConfig(VirtualPositionSpeedSensorConfig& config);

    void setPosition(float position);

    void setSpeed(float speed);

    Vector4f getPositionSpeed(Motor& motor) override;

    void calibrate(Motor& motor) override;

   private:
    VirtualPositionSpeedSensorConfig _config;
    float _position;
    float _speed;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_VIRTUALPOSITIONSPEEDSENSOR_HPP_
