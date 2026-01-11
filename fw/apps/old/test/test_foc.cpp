//
// Created by zhouj on 2022/11/17.
//

#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "tim.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_tim.h"

#include "tx_api.h"

#include "../FocControl.hpp"
#include "motor/model/PhaseCurrentSensor.hpp"
#include "motor/model/PowerSensor.hpp"
#include "motor/model/motor.hpp"
#include "../component/AbsoluteEncoderAngleSpeedSensor.hpp"
#include "../component/SamplingPowerSensor.hpp"
#include "../component/Shunt3PhaseCurrentSensor.hpp"
#include "../component/SimplePowerSensor.hpp"
#include "../component/SvpwmModular.hpp"
#include "../platform/PwmDriver.hpp"
#include "../comm/ControlProtocol.hpp"

#include "hal-i2c.hpp"
#include "hal-tim.hpp"

#include "math.hpp"
#include "rls/AS5047SPI.hpp"
#include "rls/MT6816SPI.hpp"
#include "rls/AS5600I2C.hpp"

#include "test.hpp"

using namespace wibot;
using namespace wibot::motor;

static MotorParameter mp = {

    .polePair = 7,
    .rs        = 3.465f,
    .ld        = 1.568e-3 / 2,
    .lq        = 1.64e-3 / 2,
    .flux      = 1.4109 / 2.0 * 0.034145 / k2PI / kSQRT3,  // Te=34.145ms Vlpp=1.4109.

    .interia  = 4.5e-6f,
    .friction = 0.0001f,

    // Limits
    .speedLimit    = 5000,
    .uBusMax       = 5.0f,
    .iBusLimit     = 0.2f,
    .iPhaseLimit = 0.5f,
};

static Motor mtr;

static Timer pwm(&htim1);

DEF_BUFFER32(u_bus_abc, 4);
DEF_BUFFER32(i_bus_abc, 4);
// BUFFER32_DECLARE_STATIC(pos_buf, 1)


static WaitSource ctrlLoopTrigger;

static WaitSource commLoopTrigger;
static WaitHandler ctrlLoop = ctrlLoopTrigger.getHandler();
static WaitHandler commLoop = commLoopTrigger.getHandler();
static HardI2cMaster i2c1(&hi2c1);
static Spi           spi(&hspi3);
// static AS5600I2C as5600(i2c1, eg2);
// static AS5047SPI as5047(spi, eg4);
static Mt6816Spi     mt6816(spi);
static uint32_t*     i_a(&i_bus_abc.data[1]);
static uint32_t*     i_b(&i_bus_abc.data[2]);
static uint32_t*     i_c(&i_bus_abc.data[3]);
static uint32_t*     u_bus(&u_bus_abc.data[0]);
static PwmDriver     driver(&pwm);
static Uart          uart1(&huart1, "uart1");

static FocControlConfig cfg = {
    .power_sensor{
        .u_bus    = u_bus,
        .u_bus_pu = 3.3f / 4096.0f / 12.0f * (float)(180 + 12),
    },
    .current_sensor{
        .i_a                     = i_a,
        .i_b                     = i_b,
        .i_c                     = i_c,
        .i_pu                    = 3.3f / 4096.0f / 0.33f / 1.53f,
        .low_duty_skip_threshold = 0.75f,
    },
    .modular{
        .allow_over_module = false,
    },
    .encoder{
        .codex               = &mt6816,
        .resolution          = 1 << 14,
        .direction           = EncoderDirection::Forward,
        .calibration_voltage = 1.0f,
    },
    .current_controller{
        .bw                 = 10000.0f * k2PI / 300.0f,
        .disableFeedforward = false,
        .useParams          = true,
    },
    .speed_controller{
        .delta = 11,
    },
    .position_controller{
        .kp = 0.1f,
        .ki = 0.1f,
        .kd = 0.0f,
    },
    .motor_parameter            = &mp,
    .high_frequency_samlpe_time = 0.0001f,
    .low_frequency_samlpe_time  = 0.001f,

};

static PwmDriverConfig pwm_cfg = {
    .channelA = kPwmChannel1 | kPwmChannel1N,
    .channelB = kPwmChannel2 | kPwmChannel2N,
    .channelC = kPwmChannel3 | kPwmChannel3N,
    .channelS = kPwmChannel4,
};

static FocControl foc(&driver);

static uint8_t   ctrlLoopThdStack[6000];
//static uint8_t   commTxThdStack[3000];
//static uint8_t   commRxThdStack[3000];
static TX_THREAD ctrlLoopThd;
//static TX_THREAD commRxThd;
//static TX_THREAD commTxThd;
static void      config_init() {
    driver.setConfig(pwm_cfg);
    foc.setConfig(cfg);
    mtr.reference.dSample   = 1.0f;  // for foc
}

static void init_periph() {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    LL_TIM_OC_SetCompareCH4(htim1.Instance, 8498);
    // LL_TIM_SetClockDivision(htim1.Instance, 1);
    //	PwmConfig pCfg;
    //	pCfg.fullScaleDuty = 8500;
    //	pCfg.channelsEnable = PwmChannel_1P | PwmChannel_2P | PwmChannel_3P | PwmChannel_1N |
    // PwmChannel_2N | PwmChannel_3N; 	pwm.channel_enable(PwmChannel_1P | PwmChannel_2P |
    // PwmChannel_3P | PwmChannel_1N | PwmChannel_2N | PwmChannel_3N); 	pwm.config.fullScaleDuty =
    // 8500;
    pwm.enableChannel(kPwmChannel1 | kPwmChannel1N | kPwmChannel2 | kPwmChannel2N | kPwmChannel3 |
                      kPwmChannel3N | kPwmChannel4);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);


    HAL_ADC_RegisterCallback(
        &hadc1, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, [](ADC_HandleTypeDef* hadc) {
            HAL_GPIO_TogglePin(SYNC_SIG_GPIO_Port, SYNC_SIG_Pin);
            u_bus_abc.data[0] = LL_ADC_INJ_ReadConversionData32(hadc1.Instance, LL_ADC_INJ_RANK_1);
            i_bus_abc.data[1] = LL_ADC_INJ_ReadConversionData32(hadc1.Instance, LL_ADC_INJ_RANK_2);
            i_bus_abc.data[2] = LL_ADC_INJ_ReadConversionData32(hadc1.Instance, LL_ADC_INJ_RANK_3);
            i_bus_abc.data[3] = LL_ADC_INJ_ReadConversionData32(hadc1.Instance, LL_ADC_INJ_RANK_4);
            u_bus_abc.data[1] = LL_ADC_INJ_ReadConversionData32(hadc2.Instance, LL_ADC_INJ_RANK_1);
            u_bus_abc.data[2] = LL_ADC_INJ_ReadConversionData32(hadc2.Instance, LL_ADC_INJ_RANK_2);
            u_bus_abc.data[3] = LL_ADC_INJ_ReadConversionData32(hadc2.Instance, LL_ADC_INJ_RANK_3);
            // pos_buf.data[0] = LL_ADC_INJ_ReadConversionData32(hadc2.Instance, LL_ADC_INJ_RANK_4);
            // u_bus_abc.data[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
            // i_bus_abc.data[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
            // i_bus_abc.data[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
            // i_bus_abc.data[3] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
            //
            // u_bus_abc.data[1] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
            // u_bus_abc.data[2] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
            // u_bus_abc.data[3] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
            // pos_buf.data[0] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4);
            ctrlLoopTrigger.setDone();
        });
    // HAL_ADC_Start(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc2);

    // HAL_ADC_Start(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    //    HAL_TIM_RegisterCallback(&htim16, HAL_TIM_PERIOD_ELAPSED_CB_ID,
    //                             [](TIM_HandleTypeDef* htim) { wh_outerloop.done_set(nullptr); });

    HAL_TIM_Base_Start_IT(&htim16);

    //	uint8_t data[2] = { 0, 0 };
    //	HAL_I2C_Mem_Read_IT(&hi2c1, 0x36 << 1, 0x0e, 1, data, 2);
    //	HAL_Delay(1000);
    //	HAL_I2C_Mem_Read_IT(&hi2c1, 0x36 << 1, 0x0e, 1, data, 2);
    //	HAL_Delay(1000);

    // as5600.init();
}

using namespace wibot;

static void ctrlLoop_task(uint32_t arg) {
    ctrlLoopTrigger.setDone();
    StopWatch                      sw;
    [[maybe_unused]] volatile auto duration = 0;

    uint32_t r = 0;
    while (true) {
        ctrlLoop.wait(TIMEOUT_FOREVER);
        sw.start();
        
        foc.hf_loop(mtr);
        [[maybe_unused]] volatile auto d3 = sw.tick();

        if ((r % 20) == 0) {
            foc.command_loop(mtr);
            [[maybe_unused]] volatile auto d1 = sw.tick();
            foc.lf_loop(mtr);
            [[maybe_unused]] volatile auto d2 = sw.tick();
        }
        
        r++;
    }
}
//ControlProtocol cp(uart1, foc, mtr);

//static void comm_rx_task(uint32_t arg) {
//    cp.startRx();
//}
//static void comm_tx_task(uint32_t arg) {
//    cp.startTx();
//}

[[maybe_unused]] static uint16_t encoder_raw_data;

void foc_test() {
    config_init();
    init_periph();

    FocCommand cmd;

    tx_thread_create(&ctrlLoopThd, (char*)"ctrlLoop", ctrlLoop_task, 0, ctrlLoopThdStack,
                     sizeof(ctrlLoopThdStack), 2, 1, 0, TX_AUTO_START);
//    tx_thread_create(&commRxThd, (char*)"commRx", comm_rx_task, 0, commRxThdStack,
//                     sizeof(commRxThdStack), 1, 1, 0, TX_AUTO_START);
//    tx_thread_create(&commTxThd, (char*)"commTx", comm_tx_task, 0, commTxThdStack,
//                     sizeof(commTxThdStack), 1, 1, 0, TX_AUTO_START);

    Thread::sleep(100);

    cmd.mode = wibot::motor::ControlMode::kCalibrate;
    foc.set_command(mtr, cmd);

    // while (1) {
    //     encoder_raw_data = mt6816.get_angle();
    // }

    // [[maybe_unused]] auto cfg = as5600.get_config();
    // [[maybe_unused]] volatile static uint8_t sta;
    // sta = as5600.get_status();
    // [[maybe_unused]] auto zpos = as5600.get_zpos();
    // [[maybe_unused]] auto mpos = as5600.get_mpos();

    foc.calibrate(mtr);

    cmd.mode       = wibot::motor::ControlMode::kCalibrate;
    cmd.voltage.v1 = mtr.state.uBus * 0.1;
    cmd.voltage.v2 = 0.0f;
    foc.set_command(mtr, cmd);
    Thread::sleep(1000);
    for (int i = 0; i < 100; ++i) {
        mtr.state.position.v1 = i * k2PI / 100.0;
        mtr.state.position.v2 = Math::circleNormalize(i * k2PI / 100.0 * 5);
        Thread::sleep(1);
    }

    // test stay in open loop.
    while (0) {
        cmd.mode       = ControlMode::kOpenLoop;
        cmd.voltage.v1 = mtr.state.uBus * 0.1;
        cmd.voltage.v2 = 0.0f;
        // cmd.speed = 20.0f;
        foc.set_command(mtr, cmd);
        Thread::sleep(2000);
    }

    // Test rotation in open loop
    while (0) {
        cmd.mode       = ControlMode::kOpenLoop;
        cmd.voltage.v1 = 0.0f;
        cmd.voltage.v2 = mtr.state.uBus * 0.1;
        // cmd.speed = 20.0f;
        foc.set_command(mtr, cmd);
        Thread::sleep(1000);
    }

    // Test stay in current close loop
    while (0) {
        cmd.mode       = ControlMode::kCurrent;
        cmd.current.v1 = 0.1f;
        cmd.current.v2 = 0.0f;
        foc.set_command(mtr, cmd);
        Thread::sleep(2000);
    }

    // Test rotate in current close loop
    while (0) {
        cmd.mode       = ControlMode::kCurrent;
        cmd.current.v1 = 0.0f;
        cmd.current.v2 = 0.1f;
        foc.set_command(mtr, cmd);
        Thread::sleep(2000);
    }
    while (0) {
        cmd.mode  = ControlMode::kSpeed;
        cmd.speed = k2PI * 1;
        foc.set_command(mtr, cmd);
        Thread::sleep(2000);
    }
    cmd.mode     = ControlMode::kPosition;
    cmd.position = kPI;
    foc.set_command(mtr, cmd);
    Thread::sleep(2000);
}


