//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_DQCURRENTCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_DQCURRENTCONTROLLER_HPP_

#include "motor/model/DqVoltageController.hpp"
#include "motor/model/motor.hpp"
#include "pid/pid.hpp"
namespace wibot::motor {

struct DqCurrentControllerConfig {
    union {
        struct {
            float p;
            float i;
            float d;
        } simple;
        struct {
            /**
             * 电流环的控制带宽. 典型值为: Fs*2PI/20.
             */
            float bandWidth;
        } params;
    };
    bool            useParams;
    /**
     * 是否禁用前馈控制. 默认为: false.
     */
    bool            disableFeedforward;
    float           sampleTime;
    MotorParameter* motor_parameter;
};
class PidWithFeedforwardCurrentController : public DqVoltageController {
   public:
    Result setConfig(DqCurrentControllerConfig& config);
    Vector2f   updateDqVoltage(Motor& motor) override;

   private:
    DqCurrentControllerConfig _config;

    PidController pid_d;
    PidController pid_q;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_DQCURRENTCONTROLLER_HPP_
