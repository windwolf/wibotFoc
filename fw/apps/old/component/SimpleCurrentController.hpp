//
// Created by zhouj on 2023/1/3.
//

#ifndef WWMOTOR_APP_MOTOR2_IMPL_SIMPLECURRENTCONTROLLER_HPP_
#define WWMOTOR_APP_MOTOR2_IMPL_SIMPLECURRENTCONTROLLER_HPP_

#include "../model/DqVoltageController.hpp"
namespace wibot::motor {
struct SimpleCurrentControllerConfig {
    float rs;
};

class SimpleCurrentController : public DqVoltageController {
   public:

    void setConfig(SimpleCurrentControllerConfig& config);

    Vector2f updateDqVoltage(Motor& motor) override;

   private:
    SimpleCurrentControllerConfig _config;
};

}  // namespace wibot.motor

#endif  // WWMOTOR_APP_MOTOR2_IMPL_SIMPLECURRENTCONTROLLER_HPP_
