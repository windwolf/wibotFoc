//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_SPEEDCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_SPEEDCONTROLLER_HPP_

#include "motor/model/motor.hpp"
#include "pid/pid.hpp"
#include "motor/model/DqCurrentController.hpp"
namespace wibot::motor {

struct FocPidSpeedControllerConfig {
    /**
     * 电流环的控制带宽. 没错, 是电流环, 而不是速度环的带宽.
     */
    float bandWidth;

    /**
     * 阻尼比. 速度环零极点间隔的对数表征
     */
    float           delta;
    float           sampleTime;
    MotorParameter* motor_parameter;
};
class FocPidSpeedController : public DqCurrentController {
   public:
    void setConfig(FocPidSpeedControllerConfig& config);
    Vector2f updateDqCurrent(Motor& motor) override;

   private:
    FocPidSpeedControllerConfig _config;

    PidController          _pidSpeed;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_SPEEDCONTROLLER_HPP_
