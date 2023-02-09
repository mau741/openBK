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

#include "drv_bp1658cj.h"

// Some platforms have less pins than BK7231T.
// For example, BL602 doesn't have pin number 26.
// The pin code would crash BL602 while trying to access pin 26.
// This is why the default settings here a per-platform.
#if PLATFORM_BEKEN
static int g_i2c_pin_clk = 26;
static int g_i2c_pin_data = 24;
#else
static int g_i2c_pin_clk = 0;
static int g_i2c_pin_data = 1;
#endif

// Mapping between RGBCW to current BP1658CJ channels
static byte g_channelOrder[5] = { 1, 0, 2, 3, 4 }; //in our case: Hama 5.5W GU10 RGBCW the channel order is: [Green][Red][Blue][Warm][Cold]

const int BP1658CJ_DELAY = 1; //delay*10 --> nops


void usleep(int r) //delay function do 10*r nops, because rtos_delay_milliseconds is too much
{
#ifdef WIN32
	// not possible on Windows port
#else
  for(volatile int i=0; i<r; i++)
    __asm__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
#endif
}

static void BP1658CJ_Stop() {
	HAL_PIN_SetOutputValue(g_i2c_pin_clk, 1);
	usleep(BP1658CJ_DELAY);
	HAL_PIN_SetOutputValue(g_i2c_pin_data, 1);
	usleep(BP1658CJ_DELAY);
}


static void BP1658CJ_WriteByte(uint8_t value) {
	int bit_idx;
	bool bit;

	for (bit_idx = 7; bit_idx >= 0; bit_idx--) {
		bit = BIT_CHECK(value, bit_idx);
		HAL_PIN_SetOutputValue(g_i2c_pin_data, bit);
		usleep(BP1658CJ_DELAY);
		HAL_PIN_SetOutputValue(g_i2c_pin_clk, 1);
		usleep(BP1658CJ_DELAY);
		HAL_PIN_SetOutputValue(g_i2c_pin_clk, 0);
		usleep(BP1658CJ_DELAY);
	}
	// Wait for ACK
	// TODO: pullup?
	HAL_PIN_Setup_Input(g_i2c_pin_data);
	HAL_PIN_SetOutputValue(g_i2c_pin_clk, 1);
	usleep(BP1658CJ_DELAY);
	HAL_PIN_SetOutputValue(g_i2c_pin_clk, 0);
	usleep(BP1658CJ_DELAY);
	HAL_PIN_Setup_Output(g_i2c_pin_data);
}

static void BP1658CJ_Start(uint8_t addr) {
	HAL_PIN_SetOutputValue(g_i2c_pin_data, 0);
	usleep(BP1658CJ_DELAY);
	HAL_PIN_SetOutputValue(g_i2c_pin_clk, 0);
	usleep(BP1658CJ_DELAY);
	BP1658CJ_WriteByte(addr);
}

static void BP1658CJ_PreInit() {
	HAL_PIN_Setup_Output(g_i2c_pin_clk);
	HAL_PIN_Setup_Output(g_i2c_pin_data);

	BP1658CJ_Stop();

	usleep(BP1658CJ_DELAY);
}



void BP1658CJ_Write(float *rgbcw) {
  unsigned short cur_col_10[5];
  int i;

  //ADDLOG_DEBUG(LOG_FEATURE_CMD, "Writing to Lamp: %f %f %f %f %f", rgbcw[0], rgbcw[1], rgbcw[2], rgbcw[3], rgbcw[4]);

	for(i = 0; i < 5; i++){
		// convert 0-255 to 0-1023
		//cur_col_10[i] = rgbcw[g_channelOrder[i]] * 4;
		cur_col_10[i] = MAP(rgbcw[g_channelOrder[i]], 0, 255.0f, 0, 1023.0f);
	}
  //ADDLOG_DEBUG(LOG_FEATURE_CMD, "Writing to Lamp (hex): #%02X%02X%02X%02X%02X", cur_col_10[0], cur_col_10[1], cur_col_10[2], cur_col_10[3], cur_col_10[4]);
	// If we receive 0 for all channels, we'll assume that the lightbulb is off, and activate BP1658CJ's sleep mode ([0x80] ).
	if (cur_col_10[0]==0 && cur_col_10[1]==0 && cur_col_10[2]==0 && cur_col_10[3]==0 && cur_col_10[4]==0) {
		BP1658CJ_Start(BP1658CJ_ADDR_SLEEP);
                BP1658CJ_WriteByte(BP1658CJ_SUBADDR);
                for(i = 0; i<10; ++i) //set all 10 channels to 00
                    BP1658CJ_WriteByte(0x00);
		BP1658CJ_Stop();
		return;
	}

	// Even though we could address changing channels only, in practice we observed that the lightbulb always sets all channels.
	BP1658CJ_Start(BP1658CJ_ADDR_OUT);

  // The First Byte is the Subadress
  BP1658CJ_WriteByte(BP1658CJ_SUBADDR);
	// Brigtness values are transmitted as two bytes. The light-bulb accepts a 10-bit integer (0-1023) as an input value.
	// The first 5bits of this input are transmitted in second byte, the second 5bits in the first byte.
	BP1658CJ_WriteByte((uint8_t)(cur_col_10[0] & 0x1F));  //Red
	BP1658CJ_WriteByte((uint8_t)(cur_col_10[0] >> 5));
	BP1658CJ_WriteByte((uint8_t)(cur_col_10[1] & 0x1F)); //Green
	BP1658CJ_WriteByte((uint8_t)(cur_col_10[1] >> 5));
	BP1658CJ_WriteByte((uint8_t)(cur_col_10[2] & 0x1F)); //Blue
	BP1658CJ_WriteByte((uint8_t)(cur_col_10[2] >> 5));
	BP1658CJ_WriteByte((uint8_t)(cur_col_10[4] & 0x1F)); //Cold
	BP1658CJ_WriteByte((uint8_t)(cur_col_10[4] >> 5));
	BP1658CJ_WriteByte((uint8_t)(cur_col_10[3] & 0x1F)); //Warm
	BP1658CJ_WriteByte((uint8_t)(cur_col_10[3] >> 5));

	BP1658CJ_Stop();
}


static commandResult_t BP1658CJ_RGBCW(const void *context, const char *cmd, const char *args, int flags){
	const char *c = args;
	float col[5] = { 0, 0, 0, 0, 0 };
	int ci;
	int val;

	ci = 0;

	// some people prefix colors with #
	if(c[0] == '#')
		c++;
	while (*c){
		char tmp[3];
		int r;
		tmp[0] = *(c++);
		if (!*c)
			break;
		tmp[1] = *(c++);
		tmp[2] = '\0';
		r = sscanf(tmp, "%x", &val);
		if (!r) {
			ADDLOG_ERROR(LOG_FEATURE_CMD, "BP1658CJ_RGBCW no sscanf hex result from %s", tmp);
			break;
		}

		ADDLOG_DEBUG(LOG_FEATURE_CMD, "BP1658CJ_RGBCW found chan %d -> val255 %d (from %s)", ci, val, tmp);

		col[ci] = val;

		// move to next channel.
		ci ++;
		if(ci>=5)
			break;
	}

	BP1658CJ_Write(col);

	return CMD_RES_OK;
}
// BP1658CJ_Map is used to map the RGBCW indices to BP1658CJ indices
// This is how you uset RGB CW order:
// BP1658CJ_Map 0 1 2 3 4

static commandResult_t BP1658CJ_Map(const void *context, const char *cmd, const char *args, int flags){

	Tokenizer_TokenizeString(args,0);

	if(Tokenizer_GetArgsCount()==0) {
		ADDLOG_DEBUG(LOG_FEATURE_CMD, "BP1658CJ_Map current order is %i %i %i    %i %i! ",
			(int)g_channelOrder[0],(int)g_channelOrder[1],(int)g_channelOrder[2],(int)g_channelOrder[3],(int)g_channelOrder[4]);
		return 0;
	}

	g_channelOrder[0] = Tokenizer_GetArgIntegerRange(0, 0, 4);
	g_channelOrder[1] = Tokenizer_GetArgIntegerRange(1, 0, 4);
	g_channelOrder[2] = Tokenizer_GetArgIntegerRange(2, 0, 4);
	g_channelOrder[3] = Tokenizer_GetArgIntegerRange(3, 0, 4);
	g_channelOrder[4] = Tokenizer_GetArgIntegerRange(4, 0, 4);

	ADDLOG_DEBUG(LOG_FEATURE_CMD, "BP1658CJ_Map new order is %i %i %i    %i %i! ",
		(int)g_channelOrder[0],(int)g_channelOrder[1],(int)g_channelOrder[2],(int)g_channelOrder[3],(int)g_channelOrder[4]);

	return CMD_RES_OK;
}


// startDriver BP1658CJ
// BP1658CJ_RGBCW FF00000000
void BP1658CJ_Init() {

	g_i2c_pin_clk = PIN_FindPinIndexForRole(IOR_BP1658CJ_CLK,g_i2c_pin_clk);
	g_i2c_pin_data = PIN_FindPinIndexForRole(IOR_BP1658CJ_DAT,g_i2c_pin_data);

    BP1658CJ_PreInit();

	//cmddetail:{"name":"BP1658CJ_RGBCW","args":"[HexColor]",
	//cmddetail:"descr":"Don't use it. It's for direct access of BP1658CJ driver. You don't need it because LED driver automatically calls it, so just use led_basecolor_rgb",
	//cmddetail:"fn":"BP1658CJ_RGBCW","file":"driver/drv_bp1658cj.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("BP1658CJ_RGBCW", "", BP1658CJ_RGBCW, NULL, NULL);
	//cmddetail:{"name":"BP1658CJ_Map","args":"[Ch0][Ch1][Ch2][Ch3][Ch4]",
	//cmddetail:"descr":"Maps the RGBCW values to given indices of BP1658CJ channels. This is because BP5758D channels order is not the same for some devices. Some devices are using RGBCW order and some are using GBRCW, etc, etc. Example usage: BP1658CJ_Map 0 1 2 3 4",
	//cmddetail:"fn":"BP1658CJ_Map","file":"driver/drv_bp1658cj.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("BP1658CJ_Map", "", BP1658CJ_Map, NULL, NULL);
}
