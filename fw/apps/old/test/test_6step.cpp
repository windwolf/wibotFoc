////
//// Created by zhouj on 2022/11/17.
////
//
// #include "test.hpp"
// #include "main.h"
// #include "stm32g4xx_hal.h"
// #include "stm32g4xx_hal_tim.h"
// #include "stm32g4xx_hal_tim_ex.h"
// #include "stm32g4xx_ll_tim.h"
// #include "stm32g4xx_hal_adc.h"
// #include "stm32g4xx_hal_adc_ex.h"
// #include "stm32g4xx_ll_adc.h"
// #include "tx_api.h"
// #include "peripheral/pwm.hpp"
// #include "peripheral/misc.hpp"
//
// #include "motor2/FocControl.hpp"
// #include "motor2/base.hpp"
// #include "motor2/PowerSensor.hpp"
// #include "motor2/impl/SamplingPowerSensor.hpp"
// #include "motor2/impl/SimplePowerSensor.hpp"
// #include "motor2/PhaseCurrentSensor.hpp"
// #include "motor2/impl/Shunt3PhaseCurrentSensor.hpp"
// #include "motor2/platform/PwmDriver.hpp"
// #include "motor2/impl/SvpwmModular.hpp"
// #include "motor2/impl/AbsoluteEncoderPositionSpeedSensor.hpp"
// #include "motor2/impl/DirectSectionSensor.hpp"
// #include "i2c.hpp"
// #include "device/AS5600I2C.hpp"
// #include "tim.h"
// #include "adc.h"
// #include "i2c.h"
// #include "spi.h"
// #include "utils.hpp"
// #include "math_shared.hpp"
// #include "MT6835SPI.hpp"
// #include "motor2/SixStepControl.hpp"
//
// using namespace wibot;
// using namespace wibot::os;
// using namespace wibot::peripheral;
// using namespace wibot::accessor;
// using namespace wibot::device;
// using namespace wibot::motor;
//
// static MotorParameter mp = {
//
//	.pole_pair = 7,
//	.rs = 3.465f,
//	.ld = 1.568e-3 / 2,
//	.lq = 1.64e-3 / 2,
//	.flux = 1.4109 / 2.0 * 0.034145 / k2PI / _SQRT3, // Te=34.145ms Vlpp=1.4109.
//
//	.interia = 4.5e-6f,
//	.friction = 0.0001f,
//
//	// Limits
//	.speed_limit = 5000,
//	.u_bus_max = 5.0f,
//	.i_bus_limit = 0.2f,
//	.i_phase_limit = 0.5f,
//};
//
// static Motor mtr;
//
// static Pwm pwm(htim1);
//
// BUFFER32_DECLARE_STATIC(u_bus_abc, 4)
// BUFFER32_DECLARE_STATIC(i_bus_abc, 4)
// BUFFER32_DECLARE_STATIC(pos_buf, 1)
// static EventGroup eg1("eg_curr");
// static EventGroup eg3("eg_out");
// static EventGroup eg4("eg_mt6816");
// static WaitHandler wh_innerloop(eg1);
// static WaitHandler wh_outerloop(eg3);
// static I2cMaster i2c1(hi2c1);
// static Spi spi(hspi3);
////AS5600I2C as5600(i2c1, eg2);
// static MT6835SPI mt6835(spi, eg4);
// static MemoryDataSource i_a(&i_bus_abc.data[1]);
// static MemoryDataSource i_b(&i_bus_abc.data[2]);
// static MemoryDataSource i_c(&i_bus_abc.data[3]);
// static MemoryDataSource u_bus(&u_bus_abc.data[0]);
// static MemoryDataSource u_a(&u_bus_abc.data[1]);
// static MemoryDataSource u_b(&u_bus_abc.data[2]);
// static MemoryDataSource u_c(&u_bus_abc.data[3]);
// static PwmDriver driver(pwm);
//
//
// static ClassicSixStepControlConfig cfg = {
//	.power_sensor
//	{
//		.u_bus = &u_bus,
//         .i_bus = &u_bus, //TODO:
//		.u_bus_pu = 3.3f / 4096.0f / 12.0f * (float)(180 + 12), //TODO:
//         .i_bus_pu = 3.3f / 4096.0f / 0.33f / 1.53f, //TODO:
//	},
//	.phase_voltage_sensor
//	{
//		.u_a = &u_a,
//		.u_b = &u_b,
//		.u_c = &u_c,
//		.u_pu = 3.3f / 4096.0f / 0.33f / 1.53f, //TODO:
//	},
//	.zero_cross_detector
//	{
//         .blank_count = 0, //TODO：
//		.switch_delay_count = 1, //TODO：
//	},
////	.current_controller
////	{
////		.bw = 10000.0f * k2PI / 100.0f,
////		.disableFeedforward= false,
////	},
//	.speed_controller
//	{
//		.kp = 11,
//        .ki = 1,
//        .kd = 1,
//	},
//	.motor_parameter = &mp,
//	.sampleTime = 0.0001f,
//
//};
//
// static PwmDriverConfig pwm_cfg{
//	.channel_a = PwmChannel_1P | PwmChannel_1N,
//	.channel_b = PwmChannel_2P | PwmChannel_2N,
//	.channel_c = PwmChannel_3P | PwmChannel_3N,
//	.channel_s = PwmChannel_4,
//	.fullScaleDuty = 8500,
//};
//
// static ClassicSixStepControl s6(&driver);
//
// static uint8_t innerloopThdStack[5000];
// static TX_THREAD innerloopThd;
//
// static void config_init()
//{
//	driver.config = pwm_cfg;
//	driver.apply_config();
//	s6.config = cfg;
//    s6.apply_config();
//	mtr.reference.sw_channel = 0x0f; // for foc
//	mtr.reference.d_sample = 1.0f; // for foc
//}
// static void init_periph()
//{
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//	LL_TIM_OC_SetCompareCH4(htim1.Instance, 8498);
//	//LL_TIM_SetClockDivision(htim1.Instance, 1);
////	PwmConfig pCfg;
////	pCfg.fullScaleDuty = 8500;
////	pCfg.channelsEnable = PwmChannel_1P | PwmChannel_2P | PwmChannel_3P | PwmChannel_1N |
/// PwmChannel_2N | PwmChannel_3N; /	pwm.channel_enable(PwmChannel_1P | PwmChannel_2P |
/// PwmChannel_3P | PwmChannel_1N | PwmChannel_2N | PwmChannel_3N); /	pwm.config.fullScaleDuty =
///8500;
//	pwm.all_enable();
//
//	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
//
//	HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, [](ADC_HandleTypeDef*
// hadc)
//	{
//	  HAL_GPIO_TogglePin(SYNC_SIG_GPIO_Port, SYNC_SIG_Pin);
//	  u_bus_abc.data[0] = LL_ADC_INJ_ReadConversionData32(hadc1.Instance, LL_ADC_INJ_RANK_1);
//	  i_bus_abc.data[1] = LL_ADC_INJ_ReadConversionData32(hadc1.Instance, LL_ADC_INJ_RANK_2);
//	  i_bus_abc.data[2] = LL_ADC_INJ_ReadConversionData32(hadc1.Instance, LL_ADC_INJ_RANK_3);
//	  i_bus_abc.data[3] = LL_ADC_INJ_ReadConversionData32(hadc1.Instance, LL_ADC_INJ_RANK_4);
//	  u_bus_abc.data[1] = LL_ADC_INJ_ReadConversionData32(hadc2.Instance, LL_ADC_INJ_RANK_1);
//	  u_bus_abc.data[2] = LL_ADC_INJ_ReadConversionData32(hadc2.Instance, LL_ADC_INJ_RANK_2);
//	  u_bus_abc.data[3] = LL_ADC_INJ_ReadConversionData32(hadc2.Instance, LL_ADC_INJ_RANK_3);
////    pos_buf.data[0] = LL_ADC_INJ_ReadConversionData32(hadc2.Instance, LL_ADC_INJ_RANK_4);
////	  u_bus_abc.data[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
////	  i_bus_abc.data[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
////	  i_bus_abc.data[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
////	  i_bus_abc.data[3] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
////
////	  u_bus_abc.data[1] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
////	  u_bus_abc.data[2] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
////	  u_bus_abc.data[3] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
////	  pos_buf.data[0] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4);
//	  wh_innerloop.done_set(nullptr);
//	});
//	// HAL_ADC_Start(&hadc2);
//	HAL_ADCEx_InjectedStart_IT(&hadc2);
//
//	// HAL_ADC_Start(&hadc1);
//	HAL_ADCEx_InjectedStart_IT(&hadc1);
//
//	HAL_TIM_RegisterCallback(&htim16, HAL_TIM_PERIOD_ELAPSED_CB_ID, [](TIM_HandleTypeDef* htim)
//	{
//	  wh_outerloop.done_set(nullptr);
//	});
//
//	HAL_TIM_Base_Start_IT(&htim16);
//
////	uint8_t data[2] = { 0, 0 };
////	HAL_I2C_Mem_Read_IT(&hi2c1, 0x36 << 1, 0x0e, 1, data, 2);
////	HAL_Delay(1000);
////	HAL_I2C_Mem_Read_IT(&hi2c1, 0x36 << 1, 0x0e, 1, data, 2);
////	HAL_Delay(1000);
//
//	//as5600.init();
//    mt6835.init();
//}
//
// using namespace wibot;
//
// static void innerloop_task(uint32_t arg)
//{
//	wh_innerloop.done_set(nullptr);
//	StopWatch sw;
//	[[maybe_unused]] volatile auto duration = 0;
//
//	while (true)
//	{
//		wh_innerloop.wait(TIMEOUT_FOREVER);
//		sw.start();
//		s6.hf_loop(mtr);
//		duration = sw.tick();
//
//	}
//}
// static float ref_angle;
// static float encoder_angle;
// static float encoder_nofilter_angle;
// static float diff_angle;
//
// void six_test()
//{
//    config_init();
//    init_periph();
//
//    SixStepCommand cmd;
//
//	tx_thread_create(&innerloopThd,
//		(char*)"innerloop",
//		innerloop_task,
//		0,
//		innerloopThdStack,
//		sizeof(innerloopThdStack),
//		2,
//		1,
//		0,
//		TX_AUTO_START);
//
//	Thread::sleep(100);
//
//    cmd.mode= wibot::motor::MotorRunMode::Calibrate;
//    s6.set_command(mtr, cmd);
//    s6.calibrate(mtr);
//
//	//[[maybe_unused]] auto cfg = as5600.get_config();
//	//[[maybe_unused]] volatile static uint8_t sta;
//	//sta = as5600.get_status();
//	//[[maybe_unused]] auto zpos = as5600.get_zpos();
//	//[[maybe_unused]] auto mpos = as5600.get_mpos();
//
//	cmd.mode = wibot::motor::MotorRunMode::Calibrate;
//	cmd.duty = 0.1;
//    mtr.reference.speed = 1;
//    s6.set_command(mtr, cmd);
//	os::Utils::delay(1000);
//    for (int i = 0; i < 100; ++i)
//    {
//        Thread::sleep(1);
//    }
//
//	// test stay in open loop
//	while (0)
//	{
//		cmd.mode = MotorRunMode::OpenLoop;
//        cmd.duty = 0.1;
//        mtr.reference.speed = 1;
//        s6.set_command(mtr, cmd);
//		os::Utils::delay(2000);
//	}
//
//
//
//	// Test stay in current close loop
//	while (0)
//	{
//		cmd.mode = MotorRunMode::Current;
//		cmd.current = 0.2f;
//        mtr.reference.speed = 1;
//        s6.set_command(mtr, cmd);
//		os::Utils::delay(2000);
//	}
//
//	while (0)
//	{
//		cmd.mode = MotorRunMode::Speed;
//		cmd.speed = k2PI * 1;
//        s6.set_command(mtr, cmd);
//		os::Utils::delay(5000);
//        cmd.mode = MotorRunMode::Stop;
//        s6.set_command(mtr, cmd);
//        os::Utils::delay(1000);
//        cmd.mode = MotorRunMode::Speed;
//        cmd.speed = k2PI * -1;
//        s6.set_command(mtr, cmd);
//        os::Utils::delay(5000);
//        cmd.mode = MotorRunMode::Stop;
//        s6.set_command(mtr, cmd);
//        os::Utils::delay(1000);
//	}
//}
//
