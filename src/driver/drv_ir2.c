// freeze
#include "drv_local.h"
#include "../new_common.h"

#if PLATFORM_BEKEN


#include "include.h"
#include "arm_arch.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
#include "../obk_config.h"
#include "../cmnds/cmd_public.h"
#include "bk_timer_pub.h"
#include "drv_model_pub.h"

#include <gpio_pub.h>
//#include "pwm.h"
#include "pwm_pub.h"

#include "../../beken378/func/include/net_param_pub.h"
#include "../../beken378/func/user_driver/BkDriverPwm.h"
#include "../../beken378/func/user_driver/BkDriverI2c.h"
#include "../../beken378/driver/i2c/i2c1.h"
#include "../../beken378/driver/gpio/gpio.h"
#include "../../beken378/driver/pwm/pwm.h"
#include "../../beken378/driver/pwm/pwm_new.h"


#elif WINDOWS
void bk_gpio_output(int x, int y) {

}
void bk_gpio_config_output(int x) {

}
#endif


#define REG_PWM_BASE_ADDR                   	(0x00802B00UL)
#define PWM_CHANNEL_NUMBER_ALL              	6
#define PWM_CHANNEL_NUMBER_MAX              	(PWM_CHANNEL_NUMBER_ALL - 1)
#define PWM_CHANNEL_NUMBER_MIN              	0

#define REG_PWM_GROUP_ADDR(x)               	(REG_PWM_BASE_ADDR + (0x40 * x))

#define REG_PWM_GROUP_CTRL_ADDR(x)          	(REG_PWM_GROUP_ADDR(x) + 0x00 * 4)
#define REG_PWM_CTRL_MASK                   	0xFFFFFFFFUL
#define REG_PWM_GROUP_CTRL(x)               	(*((volatile unsigned long *) REG_PWM_GROUP_CTRL_ADDR(x)))

#define PWM_GROUP_MODE_SET_BIT(x)				(8*x)

#define PWM_GROUP_PWM_ENABLE_BIT(x)				(8*x + 3)
#define PWM_GROUP_PWM_ENABLE_MASK(x)			(1 <<PWM_GROUP_PWM_ENABLE_BIT(x))

#define PWM_GROUP_PWM_INT_ENABLE_BIT(x)			(8*x + 4)
#define PWM_GROUP_PWM_STOP_BIT(x)				(8*x + 5)

#define PWM_GROUP_PWM_INT_LEVL_BIT(x)			(8*x + 6)
#define PWM_GROUP_PWM_INT_LEVL_MASK(x)			(1<< PWM_GROUP_PWM_INT_LEVL_BIT(x))

#define PWM_GROUP_PWM_CFG_UPDATA_BIT(x)			(8*x + 7)
#define PWM_GROUP_PWM_CFG_UPDATA_MASK(x)		(1<< PWM_GROUP_PWM_CFG_UPDATA_BIT(x))


#define PWM_GROUP_PWM_PRE_DIV_BIT				16
#define PWM_GROUP_PWM_PRE_DIV_MASK				(0xFF << 16)

#define PWM_GROUP_PWM_GROUP_MODE_BIT			24
#define PWM_GROUP_PWM_GROUP_MODE_MASK			1 << 24

#define PWM_GROUP_PWM_GROUP_MODE_ENABLE_BIT		25	
#define PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK	1 << 25	

#define PWM_GROUP_PWM_INT_STAT_BIT(x)			(x + 30)
#define PWM_GROUP_PWM_INT_STAT_CLEAR(x)			(1 << PWM_GROUP_PWM_INT_STAT_BIT(x))
#define PWM_GROUP_PWM_INT_STAT_MASK(x)			(1 << PWM_GROUP_PWM_INT_STAT_BIT(x))


#define REG_GROUP_PWM0_T1_ADDR(x)           (REG_PWM_GROUP_ADDR(x) + 0x01 * 4)
#define REG_GROUP_PWM0_T1(x)                (*((volatile unsigned long *) REG_GROUP_PWM0_T1_ADDR(x) ))

#define REG_GROUP_PWM0_T2_ADDR(x)           (REG_PWM_GROUP_ADDR(x) + 0x02 * 4)
#define REG_GROUP_PWM0_T2(x)                (*((volatile unsigned long *) REG_GROUP_PWM0_T2_ADDR(x) ))

#define REG_GROUP_PWM0_T3_ADDR(x)           (REG_PWM_GROUP_ADDR(x) + 0x03 * 4)
#define REG_GROUP_PWM0_T3(x)                (*((volatile unsigned long *) REG_GROUP_PWM0_T3_ADDR(x) ))

#define REG_GROUP_PWM0_T4_ADDR(x)           (REG_PWM_GROUP_ADDR(x) + 0x04 * 4)
#define REG_GROUP_PWM0_T4(x)                (*((volatile unsigned long *) REG_GROUP_PWM0_T4_ADDR(x) ))



#define REG_GROUP_PWM1_T1_ADDR(x)           (REG_PWM_GROUP_ADDR(x) + 0x05 * 4)
#define REG_GROUP_PWM1_T1(x)                (*((volatile unsigned long *) REG_GROUP_PWM1_T1_ADDR(x) ))

#define REG_GROUP_PWM1_T2_ADDR(x)           (REG_PWM_GROUP_ADDR(x) + 0x06 * 4)
#define REG_GROUP_PWM1_T2(x)                (*((volatile unsigned long *) REG_GROUP_PWM1_T2_ADDR(x) ))

#define REG_GROUP_PWM1_T3_ADDR(x)           (REG_PWM_GROUP_ADDR(x) + 0x07 * 4)
#define REG_GROUP_PWM1_T3(x)                (*((volatile unsigned long *) REG_GROUP_PWM1_T3_ADDR(x) ))

#define REG_GROUP_PWM1_T4_ADDR(x)           (REG_PWM_GROUP_ADDR(x) + 0x08 * 4)
#define REG_GROUP_PWM1_T4(x)                (*((volatile unsigned long *) REG_GROUP_PWM1_T4_ADDR(x) ))


#define REG_GROUP_PWM_CPU_ADDR(x)           (REG_PWM_GROUP_ADDR(x) + 0x09 * 4)
#define REG_GROUP_PWM_CPU(x)                (*((volatile unsigned long *) REG_GROUP_PWM_CPU_ADDR(x) ))

#define PWM_CPU_RD0							 1 << 0
#define PWM_CPU_RD1							 1 << 1


#define REG_GROUP_PWM0_RD_DATA_ADDR(x)      (REG_PWM_GROUP_ADDR(x) + 0x0a * 4)
#define REG_GROUP_PWM0_RD_DATA_(x)          (*((volatile unsigned long *) REG_GROUP_PWM0_RD_DATA_ADDR(x) ))

#define REG_GROUP_PWM1_RD_DATA_ADDR(x)      (REG_PWM_GROUP_ADDR(x) + 0x0b * 4)
#define REG_GROUP_PWM1_RD_DATA_(x)          (*((volatile unsigned long *) REG_GROUP_PWM1_RD_DATA_ADDR(x) ))



static UINT32 ir_chan
#ifndef WINDOWS
= BKTIMER0
#endif
;

static UINT32 ir_div = 1;
static UINT32 ir_periodus = 50;
static UINT32 duty_on, duty_off;

int txpin = 26;

int times[512];
int maxTimes = 512;
int *cur;
int *stop;
int myPeriodUs = 50;
int curTime = 0;
int state = 0;

UINT8 group, channel;

void Send_ISR(UINT8 t) {
	if (cur == 0)
		return;
	curTime += myPeriodUs;
	int tg = *cur;
	if (tg <= curTime) {
		curTime -= tg;
		state = !state; 
#if DEBUG_WAVE_WITH_GPIO
		bk_gpio_output(txpin, state);
#else
		unsigned int duty_cycle;
		if (state == 0) {
			duty_cycle = duty_on;
		}
		else {
			duty_cycle = duty_off;
		}
		if (channel == 0)
		{
			REG_WRITE(REG_GROUP_PWM0_T1_ADDR(group), duty_cycle);
		}
		else
		{
			REG_WRITE(REG_GROUP_PWM1_T1_ADDR(group), duty_cycle);
		}
		UINT32 level = 1;
		// cfg_updata and initial level update enable
		UINT32  value = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
		value &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(channel));
		value |= PWM_GROUP_PWM_CFG_UPDATA_MASK(channel)
			| (level << PWM_GROUP_PWM_INT_LEVL_BIT(channel));
		REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), value);
#endif
		cur++;
		if (cur == stop) {
			// done
			cur = 0;
		}
	}
}
/*
// start the driver
startDriver IR2
// start timer 50us
// arguments: duty_on_fraction, duty_off_ fraction
StartTimer 50 0.5 0
// send data
Send 1000,5000,1000,5000,1000
// 100 hi, 500 low, 100 hi, 500 low, 100 hi
*/
static commandResult_t CMD_IR2_Send(const void* context, const char* cmd, const char* args, int cmdFlags) {
	float frequency = 38000;
	float duty_cycle = 0.330000f;
	stop = times;
	// parse string like 10,12,432,432,432,432,432
	char *token = strtok(args, ",");
	*stop = 500; // prepend 500us zero
	stop++;
	while (token) {
		if (stop - times < maxTimes) {
			int x = atoi(token);
			*stop = x;
			ADDLOG_INFO(LOG_FEATURE_IR, "Value: %i",x);
			stop++;
		}
		else {
			break;
		}
		token = strtok(NULL, ",");
	}
	state = 0;
	ADDLOG_INFO(LOG_FEATURE_IR, "Queue size:",(stop - times));

#if PLATFORM_BK7231N
	// OSStatus bk_pwm_initialize(bk_pwm_t pwm, uint32_t frequency, uint32_t duty_cycle);
	bk_pwm_initialize((bk_pwm_t)pwmIndex, period, duty_off, 0, 0);
#else
	bk_pwm_initialize((bk_pwm_t)pwmIndex, period, duty_off);
#endif

	cur = times;
#if WINDOWS
	while (cur) {
		Send_ISR(0);
	}
#endif
	return CMD_RES_OK;
}
int pwmIndex = -1;
unsigned int period;
static commandResult_t CMD_IR2_Test1(const void* context, const char* cmd, const char* args, int cmdFlags) {
	pwmIndex = PIN_GetPWMIndexForPinIndex(txpin);
	// is this pin capable of PWM?
	if (pwmIndex != -1) {
		group = get_set_group(pwmIndex);
		channel = get_set_channel(pwmIndex);
		uint32_t pwmfrequency = 38000;
		period = (26000000 / pwmfrequency);
		uint32_t duty = period / 2;
#ifndef WINDOWS
#if PLATFORM_BK7231N
		// OSStatus bk_pwm_initialize(bk_pwm_t pwm, uint32_t frequency, uint32_t duty_cycle);
		bk_pwm_initialize((bk_pwm_t)pwmIndex, period, duty, 0, 0);
#else
		bk_pwm_initialize((bk_pwm_t)pwmIndex, period, duty);
#endif
		bk_pwm_start((bk_pwm_t)pwmIndex);
#endif
	}

	return CMD_RES_OK;
}

static commandResult_t CMD_IR2_StartTimer(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, 0);

	myPeriodUs = Tokenizer_GetArgIntegerDefault(0, 50);
	float duty_on_frac = Tokenizer_GetArgFloatDefault(1, 0.5f);
	float duty_off_frac = Tokenizer_GetArgFloatDefault(2, 0.0f);


#if DEBUG_WAVE_WITH_GPIO
	bk_gpio_config_output(txpin);
#else
	pwmIndex = PIN_GetPWMIndexForPinIndex(txpin);
	// is this pin capable of PWM?
	if (pwmIndex != -1) {
		uint32_t pwmfrequency = 38000;
		period = (26000000 / pwmfrequency);
		duty_on = period * duty_on_frac;
		duty_off = period * duty_off_frac;
#ifndef WINDOWS
#if PLATFORM_BK7231N
		// OSStatus bk_pwm_initialize(bk_pwm_t pwm, uint32_t frequency, uint32_t duty_cycle);
		bk_pwm_initialize((bk_pwm_t)pwmIndex, period, duty_off, 0, 0);
#else
		bk_pwm_initialize((bk_pwm_t)pwmIndex, period, duty_off);
#endif
		bk_pwm_start((bk_pwm_t)pwmIndex);
#endif
	}
#endif

#ifndef WINDOWS
	timer_param_t params = {
	 (unsigned char)ir_chan,
	 (unsigned char)ir_div, // div
	 myPeriodUs, // us
	 Send_ISR
	};
	//GLOBAL_INT_DECLARATION();


	UINT32 res;
	// test what error we get with an invalid command
	res = sddev_control((char *)TIMER_DEV_NAME, -1, 0);

	if (res == 1) {
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"bk_timer already initialised");
	}
	else {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"bk_timer driver not initialised?");
		if ((int)res == -5) {
			ADDLOG_INFO(LOG_FEATURE_IR, (char *)"bk_timer sddev not found - not initialised?");
			return;
		}
		return;
	}


	//ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer init");
	// do not need to do this
	//bk_timer_init();
	//ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer init done");
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"will ir timer setup %u", res);
	res = sddev_control((char *)TIMER_DEV_NAME, CMD_TIMER_INIT_PARAM_US, &params);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer setup %u", res);
	res = sddev_control((char *)TIMER_DEV_NAME, CMD_TIMER_UNIT_ENABLE, &ir_chan);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer enabled %u", res);
#endif
	return CMD_RES_OK;
}


static commandResult_t CMD_IR2_TestDuty(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, 0);

	float fduty = Tokenizer_GetArgFloatDefault(0, 0.5f);
#ifndef WINDOWS
	uint32 duty_cycle = period * fduty;

#if 1

	UINT8 group, channel;

	group = get_set_group(pwmIndex);
	channel = get_set_channel(pwmIndex);
	if (channel == 0)
	{
		REG_WRITE(REG_GROUP_PWM0_T1_ADDR(group), duty_cycle);
	}
	else
	{
		REG_WRITE(REG_GROUP_PWM1_T1_ADDR(group), duty_cycle);
	}
	///pwm_single_update_param_enable(pwmIndex, 1);


	UINT32 level = 1;
	// cfg_updata and initial level update enable
	UINT32  value = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	value &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(channel));
	value |= PWM_GROUP_PWM_CFG_UPDATA_MASK(channel)
		| (level << PWM_GROUP_PWM_INT_LEVL_BIT(channel));
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), value);


	//pwm_param_t param;

	///*init pwm*/
	//param.channel = (uint8_t)pwmIndex;
	//param.cfg.bits.en = PWM_INT_EN;
	//param.cfg.bits.int_en = PWM_INT_DIS;//PWM_INT_EN;
	//param.cfg.bits.mode = PWM_PWM_MODE;
	//param.cfg.bits.clk = PWM_CLK_26M;
	//param.p_Int_Handler = 0;
	//param.duty_cycle1 = duty_cycle;
	//param.duty_cycle2 = 0;
	//param.duty_cycle3 = 0;
	//param.end_value = period;  // ?????
	//pwm_single_update_param(&param);
	///REG_WRITE(REG_APB_BK_PWMn_DC_ADDR(pwmIndex), duty_cycle);
#else
#define REG_APB_BK_PWMn_CNT_ADDR(n)         (PWM_BASE + 0x08 + 2 * 0x04 * (n))
	UINT32 value;
	value = (((UINT32)duty_cycle & 0x0000FFFF) << 16)
		+ ((UINT32)period & 0x0000FFFF);
	REG_WRITE(REG_APB_BK_PWMn_CNT_ADDR(pwmIndex), value);
#endif
#endif
	return CMD_RES_OK;
}


static commandResult_t CMD_IR2_TestSet(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, 0);

	int on = Tokenizer_GetArgIntegerDefault(0, 1);

	//pwm_single_update_param_enable(pwmIndex, on);

	return CMD_RES_OK;
}
static commandResult_t CMD_IR2_TestDuty2(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, 0);

	float fduty = Tokenizer_GetArgFloatDefault(0, 0.5f);
#ifndef WINDOWS
	uint32 duty_cycle = period * fduty;
	bk_pwm_update_param((bk_pwm_t)pwmIndex, period, duty_cycle, 0, 0);
#endif

	return CMD_RES_OK;
}
void DRV_IR2_Init() {
	CMD_RegisterCommand("Test1", CMD_IR2_Test1, NULL);
	CMD_RegisterCommand("StartTimer", CMD_IR2_StartTimer, NULL);
	CMD_RegisterCommand("TestDuty", CMD_IR2_TestDuty, NULL);
	CMD_RegisterCommand("TestDuty2", CMD_IR2_TestDuty2, NULL);
	CMD_RegisterCommand("TestSet", CMD_IR2_TestSet, NULL);
	CMD_RegisterCommand("Send", CMD_IR2_Send, NULL);

	

}
