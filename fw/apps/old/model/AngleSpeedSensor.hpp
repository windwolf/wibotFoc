//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_POSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_POSITIONSPEEDSENSOR_HPP_

#include "motor/model/motor.hpp"

namespace wibot::motor {

class AngleSpeedSensor {
   public:
    /**
     *
     * @param motor
     * @param position
     * @return v1: electric angle, v2: electric speed
     */
    virtual Vector2f getAngleSpeed(Motor& motor) = 0;

    virtual void calibrate(Motor& motor) = 0;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_POSITIONSPEEDSENSOR_HPP_
