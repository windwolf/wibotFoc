//
// Created by zhouj on 2024/6/28.
//

#ifndef WWMOTOR_APP_MOTOR_UTIL_FOCMATH_HPP_
#define WWMOTOR_APP_MOTOR_UTIL_FOCMATH_HPP_

#include "../model/motor.hpp"

namespace wibot::motor {

class FocMath {
   public:
    static void    abc2ab(Vector3f abc, Vector2f& ab);
    static void    ab2dq(Vector2f ab, float theta, Vector2f& dq);
    static void    dq2ab(Vector2f dq, float theta, Vector2f& ab);
    static uint8_t getSection(float theta);
};

}  // namespace wibot

#endif  //WWMOTOR_APP_MOTOR_UTIL_FOCMATH_HPP_
