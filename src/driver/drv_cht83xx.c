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
#include "drv_cht83xx.h"

static byte g_cht_secondsUntilNextMeasurement = 1, g_cht_secondsBetweenMeasurements = 1;
static byte channel_temp = 0, channel_humid = 0;
static float g_temp = 0.0, g_humid = 0.0;
static softI2C_t g_softI2C;
static float g_calTemp = 0, g_calHum = 0;
static uint16_t sensor_id=0;
static char* g_cht_sensor = "CHT8305";

static void CHT83XX_ReadEnv(float* temp, float* hum)
{
	uint8_t buff[4];

	if(IS_CHT831X)
	{
		//Oneshot measurement
		Soft_I2C_Start(&g_softI2C, CHT83XX_I2C_ADDR);
		Soft_I2C_WriteByte(&g_softI2C, 0x0F);
		Soft_I2C_WriteByte(&g_softI2C, 0x00);
		Soft_I2C_WriteByte(&g_softI2C, 0x00);
		Soft_I2C_Stop(&g_softI2C);
	}

	Soft_I2C_Start(&g_softI2C, CHT83XX_I2C_ADDR);
	Soft_I2C_WriteByte(&g_softI2C, 0x00);
	Soft_I2C_Stop(&g_softI2C);

	rtos_delay_milliseconds(20);	//give the sensor time to do the conversion

	Soft_I2C_Start(&g_softI2C, CHT83XX_I2C_ADDR | 1);
	Soft_I2C_ReadBytes(&g_softI2C, buff, 4);
	Soft_I2C_Stop(&g_softI2C);

	//In case we have the new sensor 8310, overwrite humidity data reading it from 0x01, as it cannot be directrly read from 0x00, there is parity
	if(IS_CHT831X)
	{
		Soft_I2C_Start(&g_softI2C, CHT83XX_I2C_ADDR);
		Soft_I2C_WriteByte(&g_softI2C, 0x01);
		Soft_I2C_Stop(&g_softI2C);

		Soft_I2C_Start(&g_softI2C, CHT83XX_I2C_ADDR | 1);
		Soft_I2C_ReadBytes(&g_softI2C, buff+2, 2);
		Soft_I2C_Stop(&g_softI2C);
		int16_t temp_val = (buff[0] << 8 | buff[1]);
		// >> 3 = 13bit resolution
		(*temp) = (float)(temp_val >> 3) * 0.03125 + g_calTemp;
		(*hum) = ((((buff[2] << 8 | buff[3]) & 0x7fff) / 32768.0) * 100.0) + g_calHum;
		return;
	}

	(*temp) = ((buff[0] << 8 | buff[1]) * 165.0 / 65535.0 - 40.0) + g_calTemp;
	(*hum) = ((buff[2] << 8 | buff[3]) * 100.0 / 65535.0) + g_calHum;
}

commandResult_t CHT83XX_Calibrate(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	g_calTemp = Tokenizer_GetArgFloat(0);
	g_calHum = Tokenizer_GetArgFloat(1);

	ADDLOG_INFO(LOG_FEATURE_SENSOR, "Calibrate CHT: Calibration done temp %f and humidity %f ", g_calTemp, g_calHum);

	return CMD_RES_OK;
}

commandResult_t CHT83XX_Cycle(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);

	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	g_cht_secondsBetweenMeasurements = Tokenizer_GetArgInteger(0);

	ADDLOG_INFO(LOG_FEATURE_CMD, "Measurement will run every %i seconds", g_cht_secondsBetweenMeasurements);

	return CMD_RES_OK;
}

// startDriver CHT83XX
void CHT83XX_Init()
{
	uint8_t buff[4];

	g_softI2C.pin_clk = 9;
	g_softI2C.pin_data = 14;
	g_softI2C.pin_clk = PIN_FindPinIndexForRole(IOR_CHT83XX_CLK, g_softI2C.pin_clk);
	g_softI2C.pin_data = PIN_FindPinIndexForRole(IOR_CHT83XX_DAT, g_softI2C.pin_data);

	Soft_I2C_PreInit(&g_softI2C);

	Soft_I2C_Start(&g_softI2C, CHT83XX_I2C_ADDR);
	Soft_I2C_WriteByte(&g_softI2C, 0xfe);			//manufacturer ID 2 bytes
	Soft_I2C_Stop(&g_softI2C);

	Soft_I2C_Start(&g_softI2C, CHT83XX_I2C_ADDR | 1);
	Soft_I2C_ReadBytes(&g_softI2C, buff, 2);
	Soft_I2C_Stop(&g_softI2C);

	//Read Sensor version separately on the last 2 bytes of the buffer
	Soft_I2C_Start(&g_softI2C, CHT83XX_I2C_ADDR);
	Soft_I2C_WriteByte(&g_softI2C, 0xff);
	Soft_I2C_Stop(&g_softI2C);

	Soft_I2C_Start(&g_softI2C, CHT83XX_I2C_ADDR | 1);
	Soft_I2C_ReadBytes(&g_softI2C, buff+2, 2);
	Soft_I2C_Stop(&g_softI2C);

	//Identify chip ID and keep if for later use
	sensor_id=(buff[2] << 8 | buff[3]);

	ADDLOG_INFO(LOG_FEATURE_SENSOR, "DRV_CHT83XX_Init: ID: %02X %02X %04X", buff[0], buff[1], sensor_id);

	if(IS_CHT831X)
	{
		//it should be 8310 id is 0x8215, we enable low power mode, so only 50nA are drawn from sensor,
		//but need to write something to one shot register to trigger new measurement
		Soft_I2C_Start(&g_softI2C, CHT83XX_I2C_ADDR);
		Soft_I2C_WriteByte(&g_softI2C, 0x03);	//config register
		Soft_I2C_WriteByte(&g_softI2C, 0x48);	//enable shutdown default is 0x08
												//setting bit 6 enables low power mode,
												//to get new measurement need to write to one shot register
		Soft_I2C_WriteByte(&g_softI2C, 0x80);	//as default 0x80
		Soft_I2C_Stop(&g_softI2C);
	}

	switch(sensor_id)
	{
	case 0x8215:
		g_cht_sensor = "CHT8310";
		break;
	case 0x8315:
		g_cht_sensor = "CHT8315";
		break;
	default:
		g_cht_sensor = "CHT8305";
		break;
	}

	//cmddetail:{"name":"CHT_Calibrate","args":"[DeltaTemp][DeltaHumidity]",
	//cmddetail:"descr":"Calibrate the CHT Sensor as Tolerance is +/-2 degrees C.",
	//cmddetail:"fn":"CHT_Calibrate","file":"driver/drv_cht8305.c","requires":"",
	//cmddetail:"examples":"CHT_Calibrate -4 10 <br /> meaning -4 on current temp reading and +10 on current humidity reading"}
	CMD_RegisterCommand("CHT_Calibrate", CHT83XX_Calibrate, NULL);
	//cmddetail:{"name":"CHT_Cycle","args":"[IntervalSeconds]",
	//cmddetail:"descr":"This is the interval between measurements in seconds, by default 1. Max is 255.",
	//cmddetail:"fn":"CHT_cycle","file":"drv/drv_cht8305.c","requires":"",
	//cmddetail:"examples":"CHT_Cycle 60 <br /> measurement is taken every 60 seconds"}
	CMD_RegisterCommand("CHT_Cycle", CHT83XX_Cycle, NULL);

}

void CHT83XX_Measure()
{
	CHT83XX_ReadEnv(&g_temp, &g_humid);

	channel_temp = g_cfg.pins.channels[g_softI2C.pin_data];
	channel_humid = g_cfg.pins.channels2[g_softI2C.pin_data];
	// don't want to loose accuracy, so multiply by 10
	// We have a channel types to handle that
	CHANNEL_Set(channel_temp, (int)(g_temp * 10), 0);
	CHANNEL_Set(channel_humid, (int)(g_humid), 0);

	ADDLOG_INFO(LOG_FEATURE_SENSOR, "DRV_CHT83XX_ReadEnv: Temperature:%fC Humidity:%f%%", g_temp, g_humid);
}

void CHT83XX_OnEverySecond()
{
	if (g_cht_secondsUntilNextMeasurement <= 0)
	{
		CHT83XX_Measure();
		g_cht_secondsUntilNextMeasurement = g_cht_secondsBetweenMeasurements;
	}
	if (g_cht_secondsUntilNextMeasurement > 0)
	{
		g_cht_secondsUntilNextMeasurement--;
	}
}

void CHT83XX_AppendInformationToHTTPIndexPage(http_request_t* request)
{
	hprintf255(request, "<h2>%s Temperature=%f, Humidity=%f</h2>", g_cht_sensor, g_temp, g_humid);
	if(channel_humid == channel_temp)
	{
		hprintf255(request, "WARNING: You don't have configured target channels for temp and humid results, set the first and second channel index in Pins!");
	}
}
