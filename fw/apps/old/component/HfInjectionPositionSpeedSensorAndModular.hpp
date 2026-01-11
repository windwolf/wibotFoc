//
// Created by zhouj on 2022/12/5.
//

#ifndef WWMOTOR_APP_MOTOR2_HFINJECTIONPOSITIONSPEEDSENSORANDMODULAR_HPP_
#define WWMOTOR_APP_MOTOR2_HFINJECTIONPOSITIONSPEEDSENSORANDMODULAR_HPP_

#include "motor/model/AngleSpeedSensor.hpp"
#include "SvpwmModular.hpp"
namespace wibot::motor {
struct HfInjectionPositionSpeedSensorConfig {
    float           injected_frequency;
    float           injected_amplitude;
    float           sampleTime;
    MotorParameter* motor_parameter;
};

class HfInjectionPositionSpeedSensorAndModular
    : public AngleSpeedSensor,
      public Modular {
   public:
    void setConfig(HfInjectionPositionSpeedSensorConfig& config);

    Vector4f getPositionSpeed(Motor& motor) override;
    void calibrate(Motor& motor) override;
    void module(Motor& motor, uint8_t& section, Vector3f& d_abc, Vector3f& u_abc,
                float& d_sample) override;

   private:
    HfInjectionPositionSpeedSensorConfig _config;
    SvpwmModular _svpwm_modular;
    float        _theta;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_HFINJECTIONPOSITIONSPEEDSENSORANDMODULAR_HPP_
