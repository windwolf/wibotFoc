//
// Created by zhouj on 2022/11/16.
//

#ifndef WWMOTOR_APP_MOTOR2_BASE_HPP_
#define WWMOTOR_APP_MOTOR2_BASE_HPP_

#include "math.hpp"
#include "type.hpp"


using Vector2f = wibot::Vector2<float>;
using Vector3f = wibot::Vector3<float>;

namespace wibot::motor {
struct MotorParameter {
public:
  enum class FluxSetMode {
    Flux,
    BackEmfConstant,
  };

public:
  uint8_t polePair;
  float rs;
  float ld;
  float lq;
  float flux;

  float interia;
  float friction;

  // Limits
  /**
   * @brief Speed limit (rpm)
   */
  float speedLimit;
  float uBusMax;
  float iBusLimit;
  float iPhaseLimit;
};

enum class ControlMode {
  kStop = 0,
  //    /**
  //     * Calibrate mode.
  //     * Everything is obtained from reference.
  //     */
  //    kCalibrate,
  /**
   * Open loop.
   * Voltage reference is provided by user,
   * position information is obtained from sensor or observer.
   */
  kOpenLoop,
  /**
   * Current loop.
   * Current reference is provided by user,
   * voltage reference is obtained from controller,
   * position information is obtained from sensor or observer.
   */
  kCurrent,
  /**
   * Speed loop.
   * Speed reference is provided by user,
   * current, voltage reference are obtained from controller,
   * position information is obtained from sensor or observer.
   */
  kSpeed,
  /**
   * Position loop.
   * Position reference is provided by user,
   * speed, current, voltage reference are obtained from controller,
   * position information is obtained from sensor or observer.
   */
  kPosition,
};

enum class RunState {
  /**
   * Motor is idle or fly free.
   */
  kIdle,
  /**
   * Searching for the position and speed, preparing for run.
   * - Switch to kRun, if position and speed are clearly got;
   * - Otherwise, switch to kStartup.
   */
  kAlign,
  /**
   * First, wait if speed is not near zero.
   * - Switch to kError if timeout.
   * Then revup the current on current loop mode until current reaches the
   * threshold.
   * - Switch to kRun if speed is clearly detected and reaches the threshold.
   * - Switch to kError if current has reached the limit and speed is still not
   * detected clearly.
   */
  kStartup,
  /**
   * run.
   */
  kRun,
  kStop,
  kError,
};

struct Motor {
  ControlMode mode;
  RunState runState;
  /**
   * Updated by sensor or observer.
   */
  struct State {
    float uBus; // bus voltage
    float iBus; // bus current

    float angle;     // v1: electrical angle, v2: mechanical angle
    float speed;     // v1: electrical speed, v2: mechanical speed
    uint8_t section; // section of electrical position
    Vector3f uAbc;   // port voltage
    Vector3f iAbc;   // phase current
    Vector2f iAb;
    /**
     * For FOC only.
     */
    Vector2f iDq;

  } state;

  /**
   * Updated by controller.
   */
  struct Reference {
    Vector2f iDq;
    float speed;
    float angle;

    Vector2f uDq;
    Vector2f uAb;
    uint8_t section;
    Vector3f dAbc;
    float dSample;

    Vector3f uAbc;

  } reference;
};

} // namespace wibot::motor

#endif // WWMOTOR_APP_MOTOR2_BASE_HPP_
