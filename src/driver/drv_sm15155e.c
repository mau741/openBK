#if PLATFORM_BK7231N || WINDOWS


#include "../new_cfg.h"
#include "../new_common.h"
#include "../new_pins.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../hal/hal_pins.h"
#include "../httpserver/new_http.h"
#include "../logging/logging.h"
#include "../mqtt/new_mqtt.h"

#include "drv_spiLED.h"
#include "drv_local.h"





void SM15155E_Write(float *rgbcw) {
	byte packet[16];
	int i;
	unsigned short *cur_col_16 = (unsigned short)&packet[0];

	for (i = 0; i < 5; i++) {
		// convert 0-255 to 0-1023
		cur_col_16[i] = MAP(rgbcw[g_cfg.ledRemap.ar[i]], 0, 255.0f, 0, 65535.0f);

	}
	packet[10] = 0x73;
	packet[11] = 0x9C;
	packet[12] = 0xE7;
	packet[13] = 0x1F;
	packet[14] = 0x00;

	SPILED_SetRawBytes(0, packet, 15, 1);
}


// startDriver SM15155E
void SM15155E_Init() {
	CMD_RegisterCommand("LED_Map", CMD_LEDDriver_Map, NULL);

	SPILED_Init();

	// FF00 0000 0000 0000 0000 73 9C E7 1F 00
	SPILED_InitDMA(15);
}

#endif
