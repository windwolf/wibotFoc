//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_POSITIONCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_POSITIONCONTROLLER_HPP_

#include "../model/motor.hpp"
#include "pid/pid.hpp"
#include "../model/SpeedController.hpp"
namespace wibot::motor {

struct PositionControllerConfig {
    float           kp;
    float           ki;
    float           kd;
    float           sampleTime;
    MotorParameter* motor_parameter;
};
class PidPositionController : public SpeedController {
   public:
    void setConfig(PositionControllerConfig& config);
    float updateSpeed(Motor& motor) override;

   private:
    PositionControllerConfig _config;
    PidController _pid_pos;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_POSITIONCONTROLLER_HPP_
