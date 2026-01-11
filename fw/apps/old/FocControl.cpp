//
// Created by zhouj on 2022/11/16.
//

#include "FocControl.hpp"

#include "os.hpp"
#include "motor/util/FocMath.hpp"

namespace wibot::motor {

void FocControl::lf_loop(Motor& motor) {
    auto ps = _positionSpeedSensor.getPositionSpeed(motor);
    motor.state.position = Vector2f(ps.v1, ps.v3);
    motor.state.speed    = Vector2f(ps.v2, ps.v4);


    motor.state.section = _sectionSensor.getSection(motor);

    // 故意不break, 为了让后面的代码也执行.
    switch (motor.mode) {
        case ControlMode::kPosition:
            motor.reference.speed = _positionController.updateSpeed(motor);
        case ControlMode::kSpeed:
            motor.reference.iDq = _speedController.updateDqCurrent(motor);
        case ControlMode::kCurrent:
        case ControlMode::kOpenLoop:
        case ControlMode::kStop:
            break;
    }
}
void FocControl::hf_loop(Motor& motor) {
    motor.state.uBus = _powerSensor.getBusVoltage(motor);
    motor.state.iBus = _powerSensor.getBusCurrent(motor);
    motor.state.uAbc = _phaseVoltageSensor.getAbcVoltage(motor);
    motor.state.iAb = _phaseCurrentSensor.getAbCurrent(motor);

    FocMath::ab2dq(motor.state.iAb, motor.state.position.v1, motor.state.iDq);
    //  故意不break, 为了让后面的代码也执行.
    switch (motor.mode) {
        case ControlMode::kPosition:
        case ControlMode::kSpeed:
        case ControlMode::kCurrent:
            motor.reference.uDq = _currentController.updateDqVoltage(motor);
        case ControlMode::kOpenLoop:
            _modular.module(motor, motor.reference.section, motor.reference.dAbc, motor.reference.uAbc,
                             motor.reference.dSample);
            break;
        case ControlMode::kStop:
            motor.reference.dAbc      = Vector3f(0, 0, 0);
            motor.reference.dSample = 0.5f;
            motor.reference.uAbc      = Vector3f(0, 0, 0);
            break;
    }

    _driver->setDuty(motor);
}

void FocControl::set_command(Motor& motor, FocCommand& cmd) {
    _cmd = cmd;

    // TODO: 根据配置, 生成控制轨迹.
}
void FocControl::command_loop(Motor& motor) {
    switch (_cmd.mode) {
        case ControlMode::kStop:
            motor.mode = ControlMode::kStop;
            break;

        case ControlMode::kOpenLoop:
            motor.mode           = ControlMode::kOpenLoop;
            motor.reference.uDq  = _cmd.voltage;
            break;
        case ControlMode::kCurrent:
            motor.mode           = ControlMode::kCurrent;
            motor.reference.iDq  = _cmd.current;
            break;
        case ControlMode::kSpeed:
            motor.mode            = ControlMode::kSpeed;
            motor.reference.speed = _cmd.speed;
            break;
        case ControlMode::kPosition:
            motor.mode               = ControlMode::kPosition;
            motor.reference.position = _cmd.position;
            break;
    }
}

void FocControl::calibrate(Motor& motor) {
    auto original_mode     = motor.mode;
    auto original_u_dq     = motor.reference.uDq;
    auto original_d_sample = motor.reference.dSample;
    auto original_pos      = motor.state.position;
    auto original_spd      = motor.state.speed;

    motor.mode = ControlMode::kOpenLoop;
    Thread::sleep(100);

    motor.reference.uDq      = Vector2f(0, 0);
    motor.reference.dSample = 0.0f;
    motor.state.position     = Vector2f(0, 0);
    motor.state.speed        = Vector2f(0, 0);

    _phaseCurrentSensor.calibrate(motor);

    _positionSpeedSensor.calibrate(motor);

    motor.state.position     = original_pos;
    motor.state.speed        = original_spd;
    motor.reference.uDq      = original_u_dq;
    motor.reference.dSample = original_d_sample;
    motor.mode               = original_mode;
}
void FocControl::setConfig(FocControlConfig& config) {

    SamplingPowerSensorConfig spsCfg;
    spsCfg.uBusData     = config.power_sensor.u_bus;
    spsCfg.u_bus_pu = config.power_sensor.u_bus_pu;
    _powerSensor.setConfig(spsCfg);

    Shunt3PhaseCurrentSensorConfig scsCfg;
    scsCfg.i_a                     = config.current_sensor.i_a;
    scsCfg.i_b                     = config.current_sensor.i_b;
    scsCfg.i_c                     = config.current_sensor.i_c;
    scsCfg.i_pu                    = config.current_sensor.i_pu;
    scsCfg.low_duty_skip_threshold = config.current_sensor.low_duty_skip_threshold;
    _phaseCurrentSensor.setConfig(scsCfg);

    SvpwmModularConfig smCfg;
    smCfg.motor_parameter = config.motor_parameter;
    if (config.modular.allow_over_module) {
        smCfg.max_module_rate   = 1.0f;
        smCfg.max_d_module_rate = 1.0f;
    } else {
        smCfg.max_module_rate   = k1_SQRT3;
        smCfg.max_d_module_rate = k1_SQRT3;
    }
    _modular.setConfig(smCfg);

    AbsoluteEncoderAngleSpeedSensorConfig apsCfg;
    apsCfg.codex                 = config.encoder.codex;
    apsCfg.resolution            = config.encoder.resolution;
    apsCfg.polePairs             = config.motor_parameter->polePair;
    apsCfg.direction             = config.encoder.direction;
    apsCfg.mechanicalPostionCutoffFreq = config.motor_parameter->speedLimit / 60 * 1.2f;
    apsCfg.mechanicalSpeedCutoffFreq   = config.motor_parameter->speedLimit / 60 * 1.2f;
    apsCfg.calibration_voltage   = config.encoder.calibration_voltage;
    apsCfg.sampleTime            = config.low_frequency_samlpe_time;
    _positionSpeedSensor.setConfig(apsCfg);

//    DqCurrentControllerConfig dqccCfg;
//    dqccCfg.motor_parameter = config.motor_parameter;
//    dqccCfg.sampleTime      = config.high_frequency_samlpe_time;
//    dqccCfg.useParams       = config.current_controller.useParams;
//    if (dqccCfg.useParams) {
//        dqccCfg.params.bandWidth = config.current_controller.bw;
//    } else {
//        dqccCfg.simple.p = config.current_controller.kd;
//        dqccCfg.simple.i = config.current_controller.ki;
//        dqccCfg.simple.d = config.current_controller.kd;
//    }
//    dqccCfg.disableFeedforward = config.current_controller.disableFeedforward;
//    _currentController.setConfig(dqccCfg);

    SimpleCurrentControllerConfig sccCfg;
    sccCfg.rs = config.motor_parameter->rs;
    _currentController.setConfig(sccCfg);

    FocPidSpeedControllerConfig fpcCfg;
    fpcCfg.motor_parameter = config.motor_parameter;
    fpcCfg.sampleTime      = config.low_frequency_samlpe_time;
    fpcCfg.bandWidth       = config.current_controller.bw;
    fpcCfg.delta           = config.speed_controller.delta;
    _speedController.setConfig(fpcCfg);

    PositionControllerConfig pcCfg;
    pcCfg.motor_parameter = config.motor_parameter;
    pcCfg.sampleTime      = config.low_frequency_samlpe_time;
    pcCfg.kp              = config.position_controller.kp;
    pcCfg.ki              = config.position_controller.ki;
    pcCfg.kd              = config.position_controller.kd;
    _positionController.setConfig(pcCfg);
}
}  // namespace wibot::motor
// wibot::motor
