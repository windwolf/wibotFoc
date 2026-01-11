#ifndef __WWMOTOR_PLATFORM_PWM_DRIVER_EXECUTOR_HPP__
#define __WWMOTOR_PLATFORM_PWM_DRIVER_EXECUTOR_HPP__

#include "../model/motor.hpp"
#include "../model/Driver.hpp"
#include "pwm.hpp"

namespace wibot::motor {

struct PwmDriverConfig {
    PwmChannel channelA;
    PwmChannel channelB;
    PwmChannel channelC;
    PwmChannel channelS;
};

class PwmDriver : public Driver {
   public:
    PwmDriver(PwmGroup* pwm) : _pwm(pwm) {
    }

    void setConfig(PwmDriverConfig& config);

    void setDuty(Motor& motor) override;
    void breakdown() override;
    void resume() override;
    void prepareCharge() override;
    void controlChannel(PwmChannel channel);

   private:
    PwmDriverConfig _config;
    PwmGroup*       _pwm;
    bool            _breakdown_flag = false;
};

}  // namespace wibot::motor

#endif  // __WWMOTOR_PLATFORM_PWM_DRIVER_EXECUTOR_HPP__
