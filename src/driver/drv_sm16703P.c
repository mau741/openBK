#if PLATFORM_BK7231N

#include "arm_arch.h"
#include "drv_model_pub.h"
#include "gpio.h"
#include "gpio_pub.h"
#include "icu_pub.h"
#include "include.h"
#include "intc_pub.h"
#include "sys_ctrl_pub.h"
#include "uart_pub.h"

#include "../new_cfg.h"
#include "../new_common.h"
#include "../new_pins.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../hal/hal_pins.h"
#include "../httpserver/new_http.h"
#include "../logging/logging.h"
#include "../mqtt/new_mqtt.h"

#include "drv_spidma.h"

static uint8_t data_translate[4] = {0b10001000, 0b10001110, 0b11101000, 0b11101110};

UINT8 *send_buf;
struct spi_message *spi_msg;
BOOLEAN initialized = false;
uint32_t pixel_count = 0;
uint32_t pixel_offset = 0;
uint32_t pixel_padding = 64;
bool format_grb = false; // swap R/G for SK6812

static uint8_t translate_2bit(uint8_t input) {
	//ADDLOG_INFO(LOG_FEATURE_CMD, "Translate 0x%02x to 0x%02x", (input & 0b00000011), data_translate[(input & 0b00000011)]);
	return data_translate[(input & 0b00000011)];
}

static void translate_byte(uint8_t input, uint8_t *dst) {
	// return 0x00000000 |
	// 	   translate_2bit((input >> 6)) |
	// 	   translate_2bit((input >> 4)) |
	// 	   translate_2bit((input >> 2)) |
	// 	   translate_2bit(input);
	*dst++ = translate_2bit((input >> 6));
	*dst++ = translate_2bit((input >> 4));
	*dst++ = translate_2bit((input >> 2));
	*dst++ = translate_2bit(input);
}

void SM16703P_setMultiplePixel(uint32_t pixel, uint8_t *data, bool push) {

	// Return if driver is not loaded
	if (!initialized)
		return;

	// Check max pixel
	if (pixel > pixel_count)
		pixel = pixel_count;

	// Iterate over pixel
	uint8_t *dst = spi_msg->send_buf + pixel_offset;
	for (uint32_t i = 0; i < pixel; i++) {
		uint8_t r, g, b;
		if (format_grb) {
			g = *data++;
			r = *data++;
		} else {
			r = *data++;
			g = *data++;
		}
		b = *data++;
		*dst++ = translate_2bit((r >> 6));
		*dst++ = translate_2bit((r >> 4));
		*dst++ = translate_2bit((r >> 2));
		*dst++ = translate_2bit(r);
		*dst++ = translate_2bit((g >> 6));
		*dst++ = translate_2bit((g >> 4));
		*dst++ = translate_2bit((g >> 2));
		*dst++ = translate_2bit(g);
		*dst++ = translate_2bit((b >> 6));
		*dst++ = translate_2bit((b >> 4));
		*dst++ = translate_2bit((b >> 2));
		*dst++ = translate_2bit(b);
	}
	if (push) {
		SPIDMA_StartTX(spi_msg);
	}
}
void SM16703P_setPixel(int pixel, int r, int g, int b) {
	if (format_grb) {
		translate_byte(g, spi_msg->send_buf + (pixel_offset + 0 + (pixel * 3 * 4)));
		translate_byte(r, spi_msg->send_buf + (pixel_offset + 4 + (pixel * 3 * 4)));
	} else {
		translate_byte(r, spi_msg->send_buf + (pixel_offset + 0 + (pixel * 3 * 4)));
		translate_byte(g, spi_msg->send_buf + (pixel_offset + 4 + (pixel * 3 * 4)));
	}
	translate_byte(b, spi_msg->send_buf + (pixel_offset + 8 + (pixel * 3 * 4)));
}

commandResult_t SM16703P_CMD_setPixel(const void *context, const char *cmd, const char *args, int flags) {
	int pixel, i, r, g, b;
	Tokenizer_TokenizeString(args, 0);

	if (Tokenizer_GetArgsCount() != 4) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Not Enough Arguments for init SM16703P: Amount of LEDs missing");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	pixel = Tokenizer_GetArgIntegerRange(0, 0, 255);
	r = Tokenizer_GetArgIntegerRange(1, 0, 255);
	g = Tokenizer_GetArgIntegerRange(2, 0, 255);
	b = Tokenizer_GetArgIntegerRange(3, 0, 255);

	ADDLOG_INFO(LOG_FEATURE_CMD, "Set Pixel %i to R %i G %i B %i", pixel, r, g, b);

	if (pixel < 0) {
		for (i = 0; i < pixel_count; i++) {
			SM16703P_setPixel(i, r, g, b);
		}
	}
	else {
		SM16703P_setPixel(pixel, r, g, b);

		ADDLOG_INFO(LOG_FEATURE_CMD, "Raw Data 0x%02x 0x%02x 0x%02x 0x%02x - 0x%02x 0x%02x 0x%02x 0x%02x - 0x%02x 0x%02x 0x%02x 0x%02x",
			spi_msg->send_buf[pixel_offset + 0 + (pixel * 3 * 4)],
			spi_msg->send_buf[pixel_offset + 1 + (pixel * 3 * 4)],
			spi_msg->send_buf[pixel_offset + 2 + (pixel * 3 * 4)],
			spi_msg->send_buf[pixel_offset + 3 + (pixel * 3 * 4)],
			spi_msg->send_buf[pixel_offset + 4 + (pixel * 3 * 4)],
			spi_msg->send_buf[pixel_offset + 5 + (pixel * 3 * 4)],
			spi_msg->send_buf[pixel_offset + 6 + (pixel * 3 * 4)],
			spi_msg->send_buf[pixel_offset + 7 + (pixel * 3 * 4)],
			spi_msg->send_buf[pixel_offset + 8 + (pixel * 3 * 4)],
			spi_msg->send_buf[pixel_offset + 9 + (pixel * 3 * 4)],
			spi_msg->send_buf[pixel_offset + 10 + (pixel * 3 * 4)],
			spi_msg->send_buf[pixel_offset + 11 + (pixel * 3 * 4)]);
	}


	return CMD_RES_OK;
}

static void SM16703P_Send(byte *data, int dataSize) {
	// ToDo
}

commandResult_t SM16703P_Start(const void *context, const char *cmd, const char *args, int flags) {

	Tokenizer_TokenizeString(args, 0);

	if (Tokenizer_GetArgsCount() == 0) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Not Enough Arguments for init SM16703P: Amount of LEDs missing");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	pixel_count = Tokenizer_GetArgIntegerRange(0, 0, 255);
	if (Tokenizer_GetArgsCount() > 1) {
		const char *format = Tokenizer_GetArg(1);
		if (!stricmp(format, "GRB")) {
			format_grb = true;
		}
	}
	if (Tokenizer_GetArgsCount() > 2) {
		pixel_offset = Tokenizer_GetArgIntegerRange(2, 0, 255);
	}
	if (Tokenizer_GetArgsCount() > 3) {
		pixel_padding = Tokenizer_GetArgIntegerRange(3, 0, 255);
	}

	ADDLOG_INFO(LOG_FEATURE_CMD, "Register driver with %i LEDs", pixel_count);

	// Prepare buffer
	uint32_t buffer_size = pixel_offset + (pixel_count * 3 * 4) + pixel_padding; //Add `pixel_offset` bytes for "Reset"
	send_buf = (UINT8 *)os_malloc(sizeof(UINT8) * (buffer_size)); //18LEDs x RGB x 4Bytes
	int i;

	for (i = 0; i < pixel_offset; i++) {
		send_buf[i] = 0;
	}
	for (i = pixel_offset; i < buffer_size - pixel_padding; i++) {
		send_buf[i] = 0b10001000;
	}
	for (i = buffer_size - pixel_padding; i < buffer_size; i++) {
		send_buf[i] = 0;
	}

	spi_msg = os_malloc(sizeof(struct spi_message));
	spi_msg->send_buf = send_buf;
	spi_msg->send_len = buffer_size;

	SPIDMA_Init(spi_msg);

	initialized = true;

	return CMD_RES_OK;
}

static commandResult_t SM16703P_StartTX(const void *context, const char *cmd, const char *args, int flags) {
	if (!initialized)
		return CMD_RES_ERROR;

	SPIDMA_StartTX(spi_msg);
	return CMD_RES_OK;
}

// startDriver SM16703P
// backlog startDriver SM16703P; SM16703P_Test
void SM16703P_Init() {

	initialized = false;

	uint32_t val;

	val = GFUNC_MODE_SPI_USE_GPIO_14;
	sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &val);

	val = GFUNC_MODE_SPI_USE_GPIO_16_17;
	sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &val);

	UINT32 param;
	param = PCLK_POSI_SPI;
	sddev_control(ICU_DEV_NAME, CMD_CONF_PCLK_26M, &param);

	param = PWD_SPI_CLK_BIT;
	sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, &param);

	//cmddetail:{"name":"SM16703P_Init","args":"SM16703P_Start",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"NULL);","file":"driver/drv_sm16703P.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("SM16703P_Init", SM16703P_Start, NULL);
	//cmddetail:{"name":"SM16703P_Start","args":"SM16703P_StartTX",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"NULL);","file":"driver/drv_sm16703P.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("SM16703P_Start", SM16703P_StartTX, NULL);
	//cmddetail:{"name":"SM16703P_SetPixel","args":"SM16703P_CMD_setPixel",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"NULL);","file":"driver/drv_sm16703P.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("SM16703P_SetPixel", SM16703P_CMD_setPixel, NULL);
}
#endif
