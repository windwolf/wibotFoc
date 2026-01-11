//
// Created by zhouj on 2022/11/17.
//

#ifndef WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_
#define WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_

#include "motor/model/AngleSpeedSensor.hpp"
#include "filter/lp.hpp"
#include "mapper/piecewise-linear-value-mapper.hpp"
#include "rls/rls.hpp"

namespace wibot::motor {

enum class EncoderDirection {
    Forward = 1,
    Reverse = -1
};
#define ENCODER_MAPPER_POINT_COUNT 64

struct AbsoluteEncoderAngleSpeedSensorConfig {
    Rls*             codex;
    uint32_t         resolution;  // 12bit = 4096
    uint8_t          polePairs;
    EncoderDirection direction = EncoderDirection::Forward;
    float            mechanicalPostionCutoffFreq;
    float            mechanicalSpeedCutoffFreq;
    float            sampleTime;

    float calibration_voltage;
};

class AbsoluteEncoderAngleSpeedSensor : public AngleSpeedSensor {
   public:
    AbsoluteEncoderAngleSpeedSensor(){};
    void setConfig(AbsoluteEncoderAngleSpeedSensorConfig& config);

    /**
     * @brief get angle and speed in electrical and mechanical domain.
     * speed is in rad/s, position is in rad.
     * @param motor
     * @param position
     * @return v1: electric angle, v2: electric speed, v3: mechanical angle, v4: mechanical speed
     */
    Vector2f getAngleSpeed(Motor& motor) override;

    float getMechanicalAngleNowrap() const;

    /**
     * for test only
     * @return
     */
    float getPositionWithoutFilter() const;

    /**
     * Calibrate the zero index of the encoder, and the linear mapping between encoder value and
     * mechanical angle.
     *
     * @note Make sure the motor controller is in CALIBERATION mode, and the innerloop and outerloop
     * task is Started, before calling this function.
     * @note During calibration, the motor will rotate at forward direction 1.5 rounds, and at
     * reverse direction 1.5 rounds.
     * @param motor
     */
    void calibrate(Motor& motor) override;

   private:
    AbsoluteEncoderAngleSpeedSensorConfig _config;

    float                                                  _lastRawMechanicalPosition;
    int32_t                                                _mechanicalRound;
    float                                                  _lastMechanicalPosition;
    PiecewiseLinearValueMapper<ENCODER_MAPPER_POINT_COUNT> _mapper;
    FirstOrderLowPassFilter                                _filteredMechanicalPosition;
    FirstOrderLowPassFilter                                _filteredElectricalPosition;
    FirstOrderLowPassFilter                                _filteredMechanicalSpeed;
    FirstOrderLowPassFilter                                _filteredElectricalSpeed;

    float _2PI_RES;
    float _1_TS;
    float _PP_TS;
    void  calibrateRotate(Motor& motor);
    void  rotate(Motor& motor, float startPosition, float travel, uint32_t step,
                 uint32_t stepDelay) const;
    void  sampleData(const Motor& motor, uint32_t step);
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_ABSOLUTEENCODERPOSITIONSPEEDSENSOR_HPP_
