//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_MODULAR_HPP_
#define WWMOTOR_APP_MOTOR2_MODULAR_HPP_

#include "motor/model/motor.hpp"
namespace wibot::motor {


//using ModuleChannel = uint8_t;
//constexpr ModuleChannel ModuleChannelA = 0b0001;
//constexpr ModuleChannel ModuleChannelB = 0b0010;
//constexpr ModuleChannel ModuleChannelC = 0b0100;
//constexpr ModuleChannel ModuleChannelS = 0b1000;

class Modular {
   public:
    
   /**
    * @brief This function is responsible for performing a specific module operation on a motor.
    * 
    * @param motor The motor object on which the module operation will be performed.
    * @param section The section of the motor on which the module operation will be performed.
    * @param d_abc The input vector representing the three-phase currents.
    * @param u_abc The output vector representing the three-phase voltages.
    * @param channels The number of channels used for the module operation.
    * @param d_sample The sampling time for the module operation.
    */
    virtual void module(wibot::motor::Motor& motor, uint8_t& section, Vector3f& d_abc,
                        Vector3f& u_abc, float& d_sample) = 0;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_MODULAR_HPP_
