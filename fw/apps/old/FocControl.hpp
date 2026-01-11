//
// Created by zhouj on 2022/11/16.
//

#ifndef WWMOTOR_APP_MOTOR2_FOCCONTROL_HPP_
#define WWMOTOR_APP_MOTOR2_FOCCONTROL_HPP_

#include "motor/model/Driver.hpp"
#include "motor/model/Modular.hpp"
#include "motor/model/PhaseCurrentSensor.hpp"
#include "motor/model/AngleSpeedSensor.hpp"
#include "motor/model/PowerSensor.hpp"
#include "motor/model/SectionSensor.hpp"
#include "motor/model/motor.hpp"
#include "motor/component/AbsoluteEncoderAngleSpeedSensor.hpp"
#include "motor/component/FocPidSpeedController.hpp"
#include "motor/component/PidPositionController.hpp"
#include "motor/component/PidWithFeedforwardCurrentController.hpp"
#include "motor/component/simpleCurrentController.hpp"
#include "motor/component/SamplingPowerSensor.hpp"
#include "motor/component/Shunt3PhaseCurrentSensor.hpp"
#include "motor/component/SvpwmModular.hpp"
#include "motor/component/VirtualPositionSpeedSensor.hpp"
#include "motor/platform/PwmDriver.hpp"
#include "motor/component/DirectSectionSensor.hpp"
#include "motor/component/SamplingPhaseVoltageSensor.hpp"

namespace wibot::motor {


struct FocCommand {
    ;

    FocCommand(){};
    FocCommand(ControlMode mode) : mode(mode){};

    ControlMode mode = ControlMode::kStop;
    union {
        float    position;
        float    speed;
        Vector2f current;
        Vector2f voltage;
    };
};

struct FocControlConfig {
    struct {
        uint32_t* u_bus;
        float     u_bus_pu;
    } power_sensor;

    struct {
        uint32_t* i_a;
        uint32_t* i_b;
        uint32_t* i_c;
        float     i_pu;
        float     low_duty_skip_threshold;
    } current_sensor;

    struct {
        bool allow_over_module;
    } modular;

    struct {
        Rls*             codex;
        uint32_t         resolution;
        EncoderDirection direction;
        float            calibration_voltage;
    } encoder;

    struct {
        float bw;
        bool  disableFeedforward;
        float kp;
        float ki;
        float kd;
        bool  useParams;
    } current_controller;

    struct {
        float delta;
    } speed_controller;

    struct {
        float kp;
        float ki;
        float kd;
    } position_controller;

    MotorParameter* motor_parameter;
    float           high_frequency_samlpe_time;
    float           low_frequency_samlpe_time;
};

class FocControl {
   public:
    FocControl(PwmDriver* driver)
        : _driver(driver)
          {
          };

    void setConfig(FocControlConfig& config);

    void set_command(Motor& motor, FocCommand& cmd);

    /**
     * @brief 校准电机机控制器的各环节
     *
     * @param motor
     */
    void calibrate(Motor& motor);

    /**
     * 命令解析循环. 主要将外部命令通过设定的轨迹生成器, 生成参考.
     * @param motor
     */
    void command_loop(Motor& motor);

    /**
     * @brief 高频控制循环. 主要为电流环和驱动器调制.
     *
     * @param motor
     */
    void hf_loop(Motor& motor);

    /**
     * 低频控制循环, 主要为维护和速度环
     * @param motor
     */
    void lf_loop(Motor& motor);

    void setCurrentPid(float p, float i, float d) {
        // DqCurrentControllerConfig cfg;

        // cfg.useParams = false;
        // cfg.simple.p  = p;
        // cfg.simple.i  = i;
        // cfg.simple.d  = d;
        // _currentController.setConfig(cfg);
    }
    Vector3f getCurrentPid() {
        return Vector3f(0, 0, 0);
        // return Vector3f(_currentController.config.simple.p, _currentController.config.simple.i,
        //                 _currentController.config.simple.d);
    }

   private:

    SamplingPowerSensor                _powerSensor;
    Shunt3PhaseCurrentSensor           _phaseCurrentSensor;
    SamplingPhaseVoltageSensor         _phaseVoltageSensor;
    AbsoluteEncoderAngleSpeedSensor    _positionSpeedSensor;
    SvpwmModular                       _modular;
    DirectSectionSensor _sectionSensor;

    //PidWithFeedforwardCurrentController _currentController;
    VirtualPositionSpeedSensor _speedSensor;
    SimpleCurrentController _currentController;
    FocPidSpeedController   _speedController;
    PidPositionController   _positionController;

   private:
    PwmDriver*                         _driver;

   private:

    FocCommand     _cmd;
};

}  // namespace wibot::motor

#endif  // WWMOTOR_APP_MOTOR2_FOCCONTROL_HPP_
