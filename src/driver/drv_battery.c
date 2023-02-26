#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../logging/logging.h"
#include "drv_local.h"
#include "drv_uart.h"
#include "../httpserver/new_http.h"
#include "../hal/hal_pins.h"
#include "../hal/hal_adc.h"

#include "drv_battery.h"

static int g_pin_adc = 0, channel_adc = 0, channel_rel = 0, g_pin_rel = 0, g_battcycle = 1, g_battcycleref = 10;
static float g_battvoltage = 0.0, g_battlevel = 0.0;
static float g_vref = 2400, g_vdivider = 2.29, g_maxbatt = 3000, g_minbatt = 2000, g_adcbits = 4096;

static void Batt_Measure() {
	//this command has only been tested on CBU
	float batt_ref, batt_res, vref;
	ADDLOG_INFO(LOG_FEATURE_DRV, "DRV_BATTERY : Measure Battery volt en perc");
	g_pin_adc = PIN_FindPinIndexForRole(IOR_BAT_ADC, g_pin_adc);
	if (PIN_FindPinIndexForRole(IOR_BAT_Relay, -1) == -1) {
		g_vdivider = 1;
	}
	// if divider equal to 1 then no need for relay activation
	if (g_vdivider > 1) {
		g_pin_rel = PIN_FindPinIndexForRole(IOR_BAT_Relay, g_pin_rel);
		channel_rel = g_cfg.pins.channels[g_pin_rel];
	}
	HAL_ADC_Init(g_pin_adc);
	g_battlevel = HAL_ADC_Read(g_pin_adc);
	if (g_battlevel < 1024) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "DRV_BATTERY : ADC Value low device not on battery");
	}
	if (g_vdivider > 1) {
		CHANNEL_Set(channel_rel, 1, 0);
	}
	g_battvoltage = HAL_ADC_Read(g_pin_adc);
	ADDLOG_DEBUG(LOG_FEATURE_DRV, "DRV_BATTERY : ADC binary Measurement : %f and channel %i", g_battvoltage, channel_adc);
	if (g_vdivider > 1) {
		CHANNEL_Set(channel_rel, 0, 0);
	}
	ADDLOG_DEBUG(LOG_FEATURE_DRV, "DRV_BATTERY : Calculation with param : %f %f %f", g_vref, g_adcbits, g_vdivider);
	// batt_value = batt_value / vref / 12bits value should be 10 un doc ... but on CBU is 12 ....
	vref = g_vref / g_adcbits;
	g_battvoltage = g_battvoltage * vref;
	// multiply by 2 cause ADC is measured after the Voltage Divider
	g_battvoltage = g_battvoltage * g_vdivider;
	batt_ref = g_maxbatt - g_minbatt;
	batt_res = g_battvoltage - g_minbatt;
	ADDLOG_DEBUG(LOG_FEATURE_DRV, "DRV_BATTERY : Ref battery: %f, rest battery %f", batt_ref, batt_res);
	g_battlevel = (batt_res / batt_ref) * 100;

	MQTT_PublishMain_StringInt("voltage", (int)g_battvoltage);
	MQTT_PublishMain_StringInt("battery", (int)g_battlevel);
	ADDLOG_INFO(LOG_FEATURE_DRV, "DRV_BATTERY : battery voltage : %f and percentage %f%%", g_battvoltage, g_battlevel);
}

commandResult_t Battery_Setup(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	if (Tokenizer_GetArgsCount() < 2) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "Battery Setup : need at least min and max battery args");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	//cmddetail:"descr":"measure battery based on ADC args minbatt and maxbatt in mv. optional Vref(default 2400), ADC bits(4096) and  V_divider(2) ",

	g_minbatt = Tokenizer_GetArgFloat(0);
	g_maxbatt = Tokenizer_GetArgFloat(1);
	if (Tokenizer_GetArgsCount() > 2) {
		g_vdivider = Tokenizer_GetArgFloat(2);
	}
	if (Tokenizer_GetArgsCount() > 3) {
		g_vref = Tokenizer_GetArgFloat(3);
	}
	if (Tokenizer_GetArgsCount() > 4) {
		g_adcbits = Tokenizer_GetArgFloat(4);
	}

	ADDLOG_INFO(LOG_FEATURE_CMD, "Battery Setup : Min %i Max %i Vref %i adcbits %i vdivider %i", g_minbatt, g_maxbatt, g_vref, g_adcbits, g_vdivider);

	return CMD_RES_OK;
}


commandResult_t Battery_cycle(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	if (Tokenizer_GetArgsCount() < 1) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Battery Cycle : Need integer args for seconds cycle");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	g_battcycleref = Tokenizer_GetArgFloat(0);

	ADDLOG_INFO(LOG_FEATURE_CMD, "Battery Cycle : Measurement will run every %i seconds", g_battcycleref);

	return CMD_RES_OK;
}


// startDriver Battery
void Batt_Init() {

	//cmddetail:{"name":"Battery_Setup","args":"[int][int][float][int][int]",
	//cmddetail:"descr":"measure battery based on ADC args minbatt and maxbatt in mv. optional V_divider(2), Vref(default 2400) and ADC bits(4096) and   ",
	//cmddetail:"fn":"Battery_Setup","file":"drv/drv_battery.c","requires":"",
	//cmddetail:"examples":"Battery_Setup 1500 3000 2 2400 4096"}
	CMD_RegisterCommand("Battery_Setup", Battery_Setup, NULL);

	//cmddetail:{"name":"Battery_cycle","args":"[int]",
	//cmddetail:"descr":"change cycle of measurement by default every 10 seconds",
	//cmddetail:"fn":"Battery_cycle","file":"drv/drv_battery.c","requires":"",
	//cmddetail:"examples":"Battery_Setup 60"}
	CMD_RegisterCommand("Battery_cycle", Battery_cycle, NULL);

}

void Batt_OnEverySecond() {

	if (g_battcycle == 0) {
		Batt_Measure();
		g_battcycle = g_battcycleref;
	}
	if (g_battcycle > 0) {
		--g_battcycle;
	}
	ADDLOG_DEBUG(LOG_FEATURE_DRV, "DRV_BATTERY : Measurement will run in  %i cycle", g_battcycle);


}


void Batt_StopDriver() {

}
void Batt_AppendInformationToHTTPIndexPage(http_request_t* request)
{
	hprintf255(request, "<h2>Battery level=%f, voltage=%f</h2>", g_battlevel, g_battvoltage);
}

