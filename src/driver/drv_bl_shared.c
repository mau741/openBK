static int consumption_matrix [24] = {0};
static int export_matrix[24] = {0};
static int net_matrix[24] = {0};
static int old_export_energy = 0;
static int old_real_consumption = 0;

#include "drv_bl_shared.h"

#include "../new_cfg.h"
#include "../new_pins.h"
#include "../cJSON/cJSON.h"
#include "../hal/hal_flashVars.h"
#include "../logging/logging.h"
#include "../mqtt/new_mqtt.h"
#include "../ota/ota.h"
#include "drv_local.h"
#include "drv_ntp.h"
#include "drv_public.h"
#include "drv_uart.h"
#include "../cmnds/cmd_public.h" //for enum EventCode
#include <math.h>
#include <time.h>
int stat_updatesSkipped = 0;
int stat_updatesSent = 0;


static byte first_run = 0;
static byte savetoflash = 0;
static byte min_reset = 0;
static byte hour_reset = 0;
static float net_energy = 0;
static float real_export = 0;
static float real_consumption = 0;
// Variables for the solar dump load timer
static byte old_hour = 0;
static byte time_hour_reset = 0;
static byte time_min_reset = 0;
static byte old_time = 0;
#define max_power_bypass_off 1000
#define dump_load_hysteresis 3	// This is shortest time the relay will turn on or off. Recommended 1/4 of the netmetering period. Never use less than 1min as this stresses the relay/load.
//int min_production = -50;	// The minimun instantaneous solar production that will trigger the dump load.
#define dump_load_on 15		// The ammount of 'excess' energy stored over the period. Above this, the dump load will be turned on.
#define dump_load_off 1		// The minimun 'excess' energy stored over the period. Below this, the dump load will be turned off.

// These variables are used to program the bypass load, for example turn it on late afternoon if there was no sun for the day
//#define bypass_timer_reset 23	// Just so it doesn't accidentally reset when the device is rebooted (0)...
#define bypass_on_time 15
#define bypass_off_time 18
#define min_daily_time_on 120	// Runs the diversion load up to this specified ammount of time, if there wasn't enough sun over the day.
int time_on = 0;		// Variable to count how long the Bypass load ran during the day
int dump_load_relay = 3;	// Variable to Indicate on the Webpage if the Bypass load is on
int lastsync = 0; 		// Variable to run the bypass relay loop. It's used to take note of the last time it run
byte check_time = 0; 		// Variable for Minutes
byte check_hour = 0;		// Variable for Hour	
byte check_time_power = 0; 		// Variable for Minutes
byte check_hour_power = 0;		

//Command to turn remote plug on/off
//const char* rem_relay_on = "http://<ip>/cm?cmnd=Power%20on";
//const char* rem_relay_off = "http://<ip>/cm?cmnd=Power%20off";
//-----------------------------------------------------------
	
// Order corrsponds to enums OBK_VOLTAGE - OBK__LAST
// note that Wh/kWh units are overridden in hass_init_energy_sensor_device_info()
const char UNIT_WH[] = "Wh";
struct {
	energySensorNames_t names;
	byte rounding_decimals;
	// Variables below are for optimization
	// We can't send a full MQTT update every second.
	// It's too much for Beken, and it's too much for LWIP 2 MQTT library,
	// especially when actively browsing site and using JS app Log Viewer.
	// It even fails to publish with -1 error (can't alloc next packet)
	// So we publish when value changes from certain threshold or when a certain time passes.
	float changeSendThreshold;
	double lastReading; //double only needed for energycounter i.e. OBK_CONSUMPTION_TOTAL to avoid rounding errors as value becomes high
	double lastSentValue; // what are the last values we sent over the MQTT?
	int noChangeFrame; // how much update frames has passed without sending MQTT update of read values?
} sensors[OBK__NUM_SENSORS] = { 
	//.hass_dev_class, 	.units,		.name_friendly,			.name_mqtt,		 .hass_uniq_id_suffix, .rounding_decimals, .changeSendThreshold		
	{{"voltage",		"V",		"Voltage",			"voltage",			"0",		},  	1,			0.25,		},	// OBK_VOLTAGE
	{{"current",		"A",		"Current",			"current",			"1",		},	3,			0.002,		},	// OBK_CURRENT
	{{"power",		"W",		"Power",			"power",			"2",		},	2,			0.25,		},	// OBK_POWER
	{{"apparent_power",	"VA",		"Apparent Power",		"power_apparent",		"9",		},	2,			0.25,		},	// OBK_POWER_APPARENT
	{{"reactive_power",	"var",		"Reactive Power",		"power_reactive",		"10",		},	2,			0.25,		},	// OBK_POWER_REACTIVE
	{{"power_factor",	"",		"Power Factor",			"power_factor",			"11",		},	2,			0.05,		},	// OBK_POWER_FACTOR
	{{"energy",		UNIT_WH,	"Total Consumption",		"energycounter",		"3",		},	3,			0.1,		},	// OBK_CONSUMPTION_TOTAL
	{{"energy",		UNIT_WH,	"Total Generation",		"energycounter_generation",	"14",		},	3,			0.1,		},	// OBK_GENERATION_TOTAL	
	{{"energy",		UNIT_WH,	"Energy Last Hour",		"energycounter_last_hour",	"4",		},	3,			0.1,		},	// OBK_CONSUMPTION_LAST_HOUR
	//{{"",			"",		"Consumption Stats",		"consumption_stats",		"5",		},	0,			0,		},	// OBK_CONSUMPTION_STATS
	{{"energy",		UNIT_WH,	"Energy Today",			"energycounter_today",		"7",		},	3,			0.1,		},	// OBK_CONSUMPTION_TODAY
	{{"energy",		UNIT_WH,	"Energy Yesterday",		"energycounter_yesterday",	"6",		},	3,			0.1,		},	// OBK_CONSUMPTION_YESTERDAY
	{{"energy",		UNIT_WH,	"Energy 2 Days Ago",		"energycounter_2_days_ago",	"12",		},	3,			0.1,		},	// OBK_CONSUMPTION_2_DAYS_AGO
	{{"energy",		UNIT_WH,	"Energy 3 Days Ago",		"energycounter_3_days_ago",	"13",		},	3,			0.1,		},	// OBK_CONSUMPTION_3_DAYS_AGO
	{{"timestamp",		"",		"Energy Clear Date",		"energycounter_clear_date",	"8",		},	0,			86400,		},	// OBK_CONSUMPTION_CLEAR_DATE	
}; 

float lastReadingFrequency = NAN;

portTickType energyCounterStamp;

bool energyCounterStatsEnable = false;
int energyCounterSampleCount = 60;
int energyCounterSampleInterval = 60;
float *energyCounterMinutes = NULL;
portTickType energyCounterMinutesStamp;
long energyCounterMinutesIndex;
bool energyCounterStatsJSONEnable = false;

int actual_mday = -1;
float lastSavedEnergyCounterValue = 0.0f;
float lastSavedGenerationCounterValue = 0.0f;
float changeSavedThresholdEnergy = 10.0f;
long ConsumptionSaveCounter = 0;
portTickType lastConsumptionSaveStamp;
time_t ConsumptionResetTime = 0;

int changeSendAlwaysFrames = 60;
int changeDoNotSendMinFrames = 5;

void BL09XX_AppendInformationToHTTPIndexPage(http_request_t *request)
{
    int i;
    const char *mode;
    struct tm *ltm;

    if(DRV_IsRunning("BL0937")) {
        mode = "BL0937";
    } else if(DRV_IsRunning("BL0942")) {
        mode = "BL0942";
    } else if (DRV_IsRunning("BL0942SPI")) {
        mode = "BL0942SPI";
    } else if(DRV_IsRunning("CSE7766")) {
        mode = "CSE7766";
    } else if(DRV_IsRunning("RN8209")) {
        mode = "RN8209";
    } else {
        mode = "PWR";
    }

    poststr(request, "<hr><table style='width:100%'>");

    if (!isnan(lastReadingFrequency)) {
        poststr(request,
                "<tr><td><b>Frequency</b></td><td style='text-align: right;'>");
        hprintf255(request, "%.2f</td><td>Hz</td>", lastReadingFrequency);
    }

	for (int i = (OBK__FIRST); i <= (OBK_CONSUMPTION__DAILY_LAST); i++) {
		if (i == OBK_GENERATION_TOTAL && (!CFG_HasFlag(OBK_FLAG_POWER_ALLOW_NEGATIVE))){i++;}
		//if (i == 7){i++;}
		if (i <= OBK__NUM_MEASUREMENTS || NTP_IsTimeSynced()) {
			poststr(request, "<tr><td><b>");
			poststr(request, sensors[i].names.name_friendly);
			poststr(request, "</b></td><td style='text-align: right;'>");
			if ((i == OBK_CONSUMPTION_TOTAL) || (i == OBK_GENERATION_TOTAL))
			{
				hprintf255(request, "%.*f</td><td>kWh</td>", sensors[i].rounding_decimals, (0.001*sensors[i].lastReading));
			}
			else
			{
				hprintf255(request, "%.*f</td><td>%s</td>", sensors[i].rounding_decimals, sensors[i].lastReading, sensors[i].names.units);
			}
		}
	};
	
	// Close the table
	poststr(request, "</table>");
	// print saving interval in small text
	hprintf255(request, "<font size=1>Saving Interval: %.2fW</font>", changeSavedThresholdEnergy);

	// Aditional code for power monitoring. Creates a table with 24h stats
        // ------------------------------------------------------------------------------------------------------------------------------------------
	
	if (CFG_HasFlag(OBK_FLAG_POWER_ALLOW_NEGATIVE))
	{
	poststr(request, "<table style='width:100%'>");
	poststr(request, "<table style='text-align: center'></style>");
	poststr(request, " <h2>Energy Stats</h2>");
			
	poststr(request, "<table>");
	
	// Table Format and headers
	//poststr(request, "<tr>");
	poststr(request, "<th>Time </th>");
	poststr(request, "<th>Consumption </th>");	
	poststr(request, "<th>Export </th>");
	poststr(request, "<th>Net Metering </th></tr><hr>");

	// First field: Time
	// Initialize temp variables
	int total_net_consumption = 0;
	int total_net_export = 0;
	int total_consumption = 0;
	int total_export = 0;
	for (int q=0; q<=check_hour; q++)
		{
		if (q == check_hour)
			{
			int calculate_net_energy = (net_matrix[q]+(int)net_energy);
			hprintf255(request, "<tr><td> <b> %i:00 </td> ", q);
			hprintf255(request, "<td> <b> %dW </td> ", (int)consumption_matrix[q]);
			hprintf255(request, "<td> <b> %dW </td>", (int)export_matrix[q]);
			hprintf255(request, "<td> <b> %dW </td> </tr>", calculate_net_energy);			
			}
		else
			{
			hprintf255(request, "<tr><td> %i:00 </td> ", q);
			hprintf255(request, "<td> %dW </td> ", (int)consumption_matrix[q]);
			hprintf255(request, "<td> %dW </td>", (int)export_matrix[q]);
			hprintf255(request, "<td> %dW </td> </tr>", net_matrix[q]);	
			}
		// Summ  all the data on the table to summarize below.
		// Real Grid Consumption / Export
		total_consumption += consumption_matrix[q];
		total_export += export_matrix[q];	
		// Calculated Net Values
		if (net_matrix[q]<0)	{total_net_export -= net_matrix[q];}
		else	{total_net_consumption += net_matrix[q];}
		// -----------------------------------------------------
		}
	// Add the values for this metering period (not yet saved)
	if (net_energy<0) {total_net_export -= net_energy;}
	else {total_net_consumption += net_energy;}
	poststr(request, "</tr></table><br>");
	poststr(request, "Totals: <br>");
	hprintf255(request, "Consumption: %iW, Export: %iW (Metering) <br>", total_consumption, total_export);
	hprintf255(request, "Consumption: %iW, Export: %iW (Net Metering)  <br>", total_net_consumption, total_net_export);
	//--------------------------------------------------------------------------------------------------
		// Update status of the diversion relay on webpage		
		//-------------------------------------------------------------------------------------------------------------------------------------------------
		
		//-----------------------------------------------------------------------------------------------------
		//hprintf255(request, "<font size=1>Last sync at minute: %dmin. Boosting from %dh to %dh<br> Relay Thresholds: On: %d Wh, Off: %dWh<br> Instant Power: %dW, Consumption: %dW, Generation: %dW <br></font>", 
		//	lastsync, bypass_on_time, bypass_off_time, dump_load_on, dump_load_off, (int)sensors[OBK_POWER].lastReading, (int)sensors[OBK_CONSUMPTION_TOTAL].lastReading, (int)real_export);
		// -------------------------------------------------------------------------------------------------------------------
		// This was the original loop 'energyCounterStatsEnable == true'
		/********************************************************************************************************************/
	//------------------------------------------------------------------------------------------------------------------------------------------
	}
	// Some other stats...
    	hprintf255(request, "<p><br><h5>Changes: %i sent, %i Skipped, %li Saved. <br> %s<hr></p>",
               stat_updatesSent, stat_updatesSkipped, ConsumptionSaveCounter,
               mode);

	poststr(request, "<h5>Energy Clear Date: ");
	if (ConsumptionResetTime) {
		ltm = gmtime(&ConsumptionResetTime);
		hprintf255(request, "%04d-%02d-%02d %02d:%02d:%02d",
					ltm->tm_year+1900, ltm->tm_mon+1, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
	} else {
		poststr(request, "(not set)");
	}
	
	/********************************************************************************************************************/
	hprintf255(request, "<br>");
	if(DRV_IsRunning("NTP")==false) {
		hprintf255(request,"NTP driver is not started, daily energy stats disbled.");
	} else if (!NTP_IsTimeSynced()) {
		hprintf255(request,"Daily energy stats awaiting NTP driver to sync real time...");
	}
	hprintf255(request, "</h5>");
	/********************************************************************************************************************/
    	if (energyCounterStatsEnable == true)
	{	
    	
       		hprintf255(request,"<hr><h2>Periodic Statistics</h2>");
		//If we are measuring negative power, we can run the commands to get the netmetering stats
		// We need NTP enabled for this, as well as the statistics. They need to be manually configured because of duration and time zone.
		if (CFG_HasFlag(OBK_FLAG_POWER_ALLOW_NEGATIVE))
		{
		
		// We print some stats, mainly for debugging
		hprintf255(request, "<font size=1>Diversion relay total on-time today was %d min.<br> Next sync in %d min. ", 
				time_on, (dump_load_hysteresis-lastsync));
			// Print Status of relay)
			if (dump_load_relay == 1){poststr(request," ON - Solar Power <br></font>");}
			else if (dump_load_relay == 2) {poststr(request," ON - Timer <br></font>");}
			else if (dump_load_relay == 3) {poststr(request," OFF <br></font>");}
			else {poststr(request," OFF - Temporary bypass (High AC load or other Fault) <br></font>");}
			//----------------------
		hprintf255(request,"<font size=1> Last NetMetering reset occured at: %d:%d<br></font>", time_hour_reset, time_min_reset); // Save the value at which the counter was synchronized
		hprintf255(request,"<font size=1> Last diversion Load Bypass: %d:%d </font><br>", check_hour_power, check_time_power);	
		// Print out periodic statistics and Total Generation at the bottom of the page.
		hprintf255(request,"<h5>NetMetering (Last %d min out of %d): %.3f Wh</h5>", energyCounterMinutesIndex, energyCounterSampleCount, net_energy); //Net metering shown in Wh (Small value)    
		}	
	
		/********************************************************************************************************************/
	        hprintf255(request,"<h5>Consumption (during this period): ");
	        hprintf255(request,"%1.*f Wh<br>", sensors[OBK_CONSUMPTION_LAST_HOUR].rounding_decimals, DRV_GetReading(OBK_CONSUMPTION_LAST_HOUR));
	        hprintf255(request,"Sampling interval: %d sec<br>History length: ",energyCounterSampleInterval);
	        hprintf255(request,"%d samples<br>History per samples:<br>",energyCounterSampleCount);
	        if (energyCounterMinutes != NULL)
	        {
	            for(i=0; i<energyCounterSampleCount; i++)
	            {
	                if ((i%20)==0) {
	                    hprintf255(request, "%1.1f", energyCounterMinutes[i]);
	                } 
			else {
	                    hprintf255(request, ", %1.1f", energyCounterMinutes[i]);
	                }
	                if ((i%20)==19){
	                    hprintf255(request, "<br>");
	                }
	            }
				// energyCounterMinutesIndex is a long type, we need to use %ld instead of %d
	            if ((i%20)!=0)
	                hprintf255(request, "<br>");
	            hprintf255(request, "History Index: %ld<hr><br>JSON Stats: %s <br>", energyCounterMinutesIndex,
	                    (energyCounterStatsJSONEnable == true) ? "enabled" : "disabled");
	        }
	        hprintf255(request, "</h5>");
	    } 
    else {
        hprintf255(request,"<h5>Periodic Statistics disabled. Use startup command SetupEnergyStats to enable function.</h5>");
    }
    /********************************************************************************************************************/	
}

void BL09XX_SaveEmeteringStatistics()
{
    ENERGY_METERING_DATA data;

    memset(&data, 0, sizeof(ENERGY_METERING_DATA));

    data.TotalGeneration = sensors[OBK_GENERATION_TOTAL].lastReading;
    data.TotalConsumption = sensors[OBK_CONSUMPTION_TOTAL].lastReading;
    data.TodayConsumpion = sensors[OBK_CONSUMPTION_TODAY].lastReading;
    data.YesterdayConsumption = sensors[OBK_CONSUMPTION_YESTERDAY].lastReading;
    data.actual_mday = actual_mday;
    data.ConsumptionHistory[0] = sensors[OBK_CONSUMPTION_2_DAYS_AGO].lastReading;
    data.ConsumptionHistory[1] = sensors[OBK_CONSUMPTION_3_DAYS_AGO].lastReading;
    data.ConsumptionResetTime = ConsumptionResetTime;
    ConsumptionSaveCounter++;
    data.save_counter = ConsumptionSaveCounter;

    HAL_SetEnergyMeterStatus(&data);
}

commandResult_t BL09XX_ResetEnergyCounter(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    float value;
    int i;

    if(args==0||*args==0) 
    {
        sensors[OBK_GENERATION_TOTAL].lastReading = 0.0;
	sensors[OBK_CONSUMPTION_TOTAL].lastReading = 0.0;
        energyCounterStamp = xTaskGetTickCount();
        if (energyCounterStatsEnable == true)
        {
            if (energyCounterMinutes != NULL)
            {
                for(i = 0; i < energyCounterSampleCount; i++)
                {
                    energyCounterMinutes[i] = 0.0;
                }
            }
            energyCounterMinutesStamp = xTaskGetTickCount();
            energyCounterMinutesIndex = 0;
        }
        for(i = OBK_CONSUMPTION__DAILY_FIRST; i <= OBK_CONSUMPTION__DAILY_LAST; i++)
        {
            sensors[i].lastReading = 0.0;
        }
    } else {
        value = atof(args);
        sensors[OBK_CONSUMPTION_TOTAL].lastReading = value;
        energyCounterStamp = xTaskGetTickCount();
    }
    ConsumptionResetTime = (time_t)NTP_GetCurrentTime();
#if WINDOWS
#elif PLATFORM_BL602
#elif PLATFORM_W600 || PLATFORM_W800
#elif PLATFORM_XR809
#elif PLATFORM_BK7231N || PLATFORM_BK7231T
    if (ota_progress()==-1)
#endif
    { 
        BL09XX_SaveEmeteringStatistics();
        lastConsumptionSaveStamp = xTaskGetTickCount();
    }
    return CMD_RES_OK;
}

commandResult_t BL09XX_SetupEnergyStatistic(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    // SetupEnergyStats enable sample_time sample_count
    int enable;
    int sample_time;
    int sample_count;
    int json_enable;

    Tokenizer_TokenizeString(args,0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 3)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

    enable = Tokenizer_GetArgInteger(0);
    sample_time = Tokenizer_GetArgInteger(1);
    sample_count = Tokenizer_GetArgInteger(2);
    if (Tokenizer_GetArgsCount() >= 4)
        json_enable = Tokenizer_GetArgInteger(3);
    else
        json_enable = 0;

    /* Security limits for sample interval */
    if (sample_time <10)
        sample_time = 10;
    if (sample_time >900)
        sample_time = 900;

    /* Security limits for sample count */
    if (sample_count < 10)
        sample_count = 10;
    if (sample_count > 180)
        sample_count = 180;   

    /* process changes */
    if (enable != 0)
    {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "Consumption History enabled");
        /* Enable function */
        energyCounterStatsEnable = true;
        if (energyCounterSampleCount != sample_count)
        {
            /* upgrade sample count, free memory */
            if (energyCounterMinutes != NULL)
                os_free(energyCounterMinutes);
            energyCounterMinutes = NULL;
            energyCounterSampleCount = sample_count;
        }
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "Sample Count:    %d", energyCounterSampleCount);
        if (energyCounterSampleInterval != sample_time)
        {
            /* change sample time */            
            energyCounterSampleInterval = sample_time;
            if (energyCounterMinutes != NULL)
                memset(energyCounterMinutes, 0, energyCounterSampleCount*sizeof(float));
        }
        
        if (energyCounterMinutes == NULL)
        {
            /* allocate new memeory */
            energyCounterMinutes = (float*)os_malloc(sample_count*sizeof(float));
            if (energyCounterMinutes != NULL)
            {
                memset(energyCounterMinutes, 0, energyCounterSampleCount*sizeof(float));
            }
        }
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "Sample Interval: %d", energyCounterSampleInterval);

        energyCounterMinutesStamp = xTaskGetTickCount();
        energyCounterMinutesIndex = 0;
    } else {
        /* Disable Consimption Nistory */
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "Consumption History disabled");
        energyCounterStatsEnable = false;
        if (energyCounterMinutes != NULL)
        {
            os_free(energyCounterMinutes);
            energyCounterMinutes = NULL;
        }
        energyCounterSampleCount = sample_count;
        energyCounterSampleInterval = sample_time;
    }

    energyCounterStatsJSONEnable = (json_enable != 0) ? true : false; 

    return CMD_RES_OK;
}

commandResult_t BL09XX_VCPPublishIntervals(const void *context, const char *cmd, const char *args, int cmdFlags)
{
	Tokenizer_TokenizeString(args, 0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	changeDoNotSendMinFrames = Tokenizer_GetArgInteger(0);
	changeSendAlwaysFrames = Tokenizer_GetArgInteger(1);

	return CMD_RES_OK;
}
commandResult_t BL09XX_VCPPrecision(const void *context, const char *cmd, const char *args, int cmdFlags)
{
	int i;
	Tokenizer_TokenizeString(args, 0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	for (i = 0; i < Tokenizer_GetArgsCount(); i++) {
		int val = Tokenizer_GetArgInteger(i);
			switch(i) {
			case 0: // voltage
				sensors[OBK_VOLTAGE].rounding_decimals = val;
				break;
			case 1: // current
				sensors[OBK_CURRENT].rounding_decimals = val;
				break;
			case 2: // power
				sensors[OBK_POWER].rounding_decimals = val;
				sensors[OBK_POWER_APPARENT].rounding_decimals = val;
				sensors[OBK_POWER_REACTIVE].rounding_decimals = val;
				break;
			case 3: // energy
				for (int j = OBK_CONSUMPTION__DAILY_FIRST; j <= OBK_CONSUMPTION__DAILY_LAST; j++) {
					sensors[j].rounding_decimals = val;
				};

			};
	}

	return CMD_RES_OK;
}
commandResult_t BL09XX_VCPPublishThreshold(const void *context, const char *cmd, const char *args, int cmdFlags)
{
	Tokenizer_TokenizeString(args, 0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 3)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	sensors[OBK_VOLTAGE].changeSendThreshold = Tokenizer_GetArgFloat(0);
	sensors[OBK_CURRENT].changeSendThreshold = Tokenizer_GetArgFloat(1);
	sensors[OBK_POWER].changeSendThreshold = Tokenizer_GetArgFloat(2);
	sensors[OBK_POWER_APPARENT].changeSendThreshold = Tokenizer_GetArgFloat(2);
	sensors[OBK_POWER_REACTIVE].changeSendThreshold = Tokenizer_GetArgFloat(2);
	//sensors[OBK_POWER_FACTOR].changeSendThreshold = Tokenizer_GetArgFloat(TODO);

	if (Tokenizer_GetArgsCount() >= 4) {
		for (int i = OBK_CONSUMPTION_LAST_HOUR; i <= OBK_CONSUMPTION__DAILY_LAST; i++) {
			sensors[i].changeSendThreshold = Tokenizer_GetArgFloat(3);
		}
	}
	return CMD_RES_OK;
}

commandResult_t BL09XX_SetupConsumptionThreshold(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    float threshold;
    Tokenizer_TokenizeString(args,0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
    
    threshold = atof(Tokenizer_GetArg(0)); 

    // Icreased maximun value to 1000. This will allow less flash wear on devices measuring large ammounts of power, such as solar
    if (threshold<1.0f)
        threshold = 1.0f;
    if (threshold>1000.0f)
        threshold = 1000.0f;
    changeSavedThresholdEnergy = threshold;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "ConsumptionThreshold: %1.1f", changeSavedThresholdEnergy);

    return CMD_RES_OK;
}

bool Channel_AreAllRelaysOpen() {
	int i, role, ch;

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		role = g_cfg.pins.roles[i];
		ch = g_cfg.pins.channels[i];
		if (role == IOR_Relay) {
			// this channel is high = relay is set
			if (CHANNEL_Get(ch)) {
				return false;
			}
		}
		if (role == IOR_Relay_n) {
			// this channel is low = relay_n is set
			if (CHANNEL_Get(ch)==false) {
				return false;
			}
		}
		if (role == IOR_BridgeForward) {
			// this channel is high = relay is set
			if (CHANNEL_Get(ch)) {
				return false;
			}
		}
	}
	return true;
}

// Lets hide this as I'm using the flag for testing purposes
float BL_ChangeEnergyUnitIfNeeded(float Wh) {
	if (CFG_HasFlag(OBK_FLAG_MQTT_ENERGY_IN_KWH)) {
		return Wh * 0.001f;
	}
	return Wh;
}

void BL_ProcessUpdate(float voltage, float current, float power,
                      float frequency, float energyWh) {
    int i;
    int xPassedTicks;
    //float time counter
    float energy_counter_data = 0;
	
    cJSON* root;
    cJSON* stats;
    char *msg;
    portTickType interval;
    time_t ntpTime;
    struct tm *ltm;
    char datetime[64];
	float diff;

		if (CFG_HasFlag(OBK_FLAG_POWER_ALLOW_NEGATIVE))
		{			
			//sync with the clock
			check_time = NTP_GetMinute();
			check_hour = NTP_GetHour();

			// This variable runs once every hour
			if (!(check_hour == old_hour))
			{
				hour_reset = 1;
				old_hour = check_hour;
				// This resets the time the bypass relay was on throughout the day, before sunset.
				if (check_hour < 5) {time_on = 0;}
			}
			else if (!(check_time == old_time))
			{
				// This runs once a minute
				min_reset = 1;
				//overpower_reset = 1;
				old_time = check_time;
				lastsync++;
			}

			//Make an animation to indicate bypass is on
			if (dump_load_relay == 4)
			{
				// In case we want to do something here. Not currently implemented.
			}
			
			// This turns the bypass load off if we are using a lot of power
			if (((sensors[OBK_POWER].lastReading) > max_power_bypass_off) && (!(dump_load_relay == 4)))
			{
				// Make sure we don't run it twice on the same minute
				if (!(check_time == check_time_power))
				{
					// Hold the loop
					lastsync = 0;
					//hour_reset = 0;
					dump_load_relay = 4;
					check_time_power = check_time;
					check_hour_power = check_hour;
				}
				
			}

	// Add to the table ---------------------------------------------------------------------------------
			
		export_matrix[check_hour] = old_export_energy + (int)real_export;
		consumption_matrix [check_hour] = old_real_consumption + (int)real_consumption;		

	// End of Add to the table --------------------------------------------------------------------------
			
			// If netmetering is enabled, we reset every hour.
			if (hour_reset == 1)
			{
				// reset the timing variable. if we are producing enough, if wont cycle the diversion load.
				lastsync = 0;

				//We want the last hour values to be recovered. So we use some logic.
				if (check_hour >0) {net_matrix[check_hour-1] += net_energy;}
				else {net_matrix[23] += net_energy;}
				// Clear old data from our current time table.
				net_matrix[check_hour] = 0;
				consumption_matrix [check_hour] = 0;
				export_matrix[check_hour] = 0;
				
				
			// Clear the variables
			old_export_energy = 0;
			old_real_consumption = 0;
			net_energy = 0;
			real_export = 0;
			real_consumption = 0;
			min_reset = 0;
			hour_reset = 0;
			energyCounterMinutesIndex = 0;
			// Save to Flash
			savetoflash = 1;
			// Save the time
			time_hour_reset = check_hour;
			time_min_reset = check_time;	
			}
	
			// ------------------------------------------------------------------------------------------------------------------
			// Calculate the Effective energy consumed / produced during the period by summing both counters and deduct their values at the start of the period
			net_energy = (real_consumption - real_export);			// calculate difference since start
			// ------------------------------------------------------------------------------------------------------------------
			
			// Bypass load code. Runs if there is excess energy and at a programmable time, in case there was no sun
			// Make sure to reset the old time at every hour, otherwise the loop will not run, because old minutes are ahead in time!

			// Status Check
			// Here we define a Bypass. For example if a very heavy load is connected, it's likelly our bypass load is not desired.
			// In this case, we turn the load off and wait for the next cycle for a new update.

			//The relay is updated ever x numer of minutes as defined on 'dump_load_hysteresis'
			if (lastsync >= dump_load_hysteresis)
			{
				// save the last time the loop was run
				lastsync = 0;
		
				// Are we exporting enough? If so, turn the relay on
				if (((int)net_energy>(int)dump_load_on))
					{
					dump_load_relay = 1;
					time_on += dump_load_hysteresis;	// Increase the timer.					
					}
				else if ((check_hour >= bypass_on_time) && (check_hour < bypass_off_time) && (time_on < min_daily_time_on))
					{
					dump_load_relay = 2;
					}
				else if (((int)net_energy<(int)dump_load_off))
					{
					// If none of the exemptions applies, we turn the diversion load off.
					dump_load_relay = 3;
					}
			}
				
			//-------------------------------------------------------------------------------------------------------------------------------------------------
			} // end of negative flag loop

	// I had reports that BL0942 sometimes gives 
	// a large, negative peak of current/power
	if (!CFG_HasFlag(OBK_FLAG_POWER_ALLOW_NEGATIVE)) 
    {
		if (power < 0.0f)
			power = 0.0f;
		if (voltage < 0.0f)
			voltage = 0.0f;
		if (current < 0.0f)
			current = 0.0f;
	}
	if (CFG_HasFlag(OBK_FLAG_POWER_FORCE_ZERO_IF_RELAYS_OPEN))
	{
		if (Channel_AreAllRelaysOpen()) {
			power = 0;
			current = 0;
		}
	}

   	sensors[OBK_VOLTAGE].lastReading = voltage;
   	sensors[OBK_CURRENT].lastReading = current;
	sensors[OBK_POWER].lastReading = power;
	sensors[OBK_POWER_APPARENT].lastReading = sensors[OBK_VOLTAGE].lastReading * sensors[OBK_CURRENT].lastReading;
    	sensors[OBK_POWER_REACTIVE].lastReading = (sensors[OBK_POWER_APPARENT].lastReading <= fabsf(sensors[OBK_POWER].lastReading)
										? 0
										: sqrtf(powf(sensors[OBK_POWER_APPARENT].lastReading, 2) -
												powf(sensors[OBK_POWER].lastReading, 2)));  
	sensors[OBK_POWER_FACTOR].lastReading =
        (sensors[OBK_POWER_APPARENT].lastReading == 0 ? 1 : sensors[OBK_POWER].lastReading / sensors[OBK_POWER_APPARENT].lastReading);

	lastReadingFrequency = frequency;

    float energy = 0;
    float energy_today_temp = 0;
    if (isnan(energyWh)) {
        xPassedTicks = (int)(xTaskGetTickCount() - energyCounterStamp);
        // FIXME: Wrong calculation if tick count overflows
        if (xPassedTicks <= 0)
            xPassedTicks = 1;
        energy = xPassedTicks * power / (3600000.0f / portTICK_PERIOD_MS);
    } 
    else
	// Check if the last power reading is positive or negative. If positive Increment consumption.
    	{
	if ((int)power>0)
	{
		sensors[OBK_CONSUMPTION_TOTAL].lastReading += energyWh;
		real_consumption += energyWh;
		energy_counter_data += energyWh;
		energy_today_temp = energyWh;
	}
	else
	{
		//If not positive, we check if the negative power flag is enabled. If so, we load the generation counter.
		if (CFG_HasFlag(OBK_FLAG_POWER_ALLOW_NEGATIVE))
		{
			// If the power is negative - Load the generation counter, but only if we allow negative measurements :-)
			sensors[OBK_GENERATION_TOTAL].lastReading += energyWh;	
			real_export += energyWh;
			energy_counter_data -= energyWh;
		}
	}
	}
    // -------------------------------------------------------------------------------------------
    // Apply values. Add Extra variable for generation 
    // We use temp variables so the timer can go up or down. This would cause issues with Home assistant.
    // We also take advantage of this to save at regular intervals.
    	//real_consumption = sensors[OBK_CONSUMPTION_TOTAL].lastReading;
	//real_export = sensors[OBK_GENERATION_TOTAL].lastReading;
	
    energyCounterStamp = xTaskGetTickCount();
    HAL_FlashVars_SaveTotalConsumption(sensors[OBK_CONSUMPTION_TOTAL].lastReading);
	sensors[OBK_CONSUMPTION_TODAY].lastReading  += energy_today_temp;

    if (NTP_IsTimeSynced()) {
        ntpTime = (time_t)NTP_GetCurrentTime();
        ltm = gmtime(&ntpTime);
        if (ConsumptionResetTime == 0)
            ConsumptionResetTime = (time_t)ntpTime;

        if (actual_mday == -1)
        {
            actual_mday = ltm->tm_mday;
        }
        if (actual_mday != ltm->tm_mday)
        {
            for (i = OBK_CONSUMPTION__DAILY_LAST; i >= OBK_CONSUMPTION__DAILY_FIRST; i--) {
                sensors[i].lastReading = sensors[i - 1].lastReading;
			}
            sensors[OBK_CONSUMPTION_TODAY].lastReading = 0.0;
            actual_mday = ltm->tm_mday;

#if WINDOWS
#elif PLATFORM_BL602
#elif PLATFORM_W600 || PLATFORM_W800
#elif PLATFORM_XR809
#elif PLATFORM_BK7231N || PLATFORM_BK7231T
            if (ota_progress()==-1)
#endif
            {
                BL09XX_SaveEmeteringStatistics();
                lastConsumptionSaveStamp = xTaskGetTickCount();
            }

        }
    }

    if (energyCounterStatsEnable == true)
    {
        interval = energyCounterSampleInterval;
        interval *= (1000 / portTICK_PERIOD_MS); 
        if ((xTaskGetTickCount() - energyCounterMinutesStamp) >= interval)
        {
			if (energyCounterMinutes != NULL) {
				sensors[OBK_CONSUMPTION_LAST_HOUR].lastReading = 0;
				for(int i = 0; i < energyCounterSampleCount; i++) {
					sensors[OBK_CONSUMPTION_LAST_HOUR].lastReading  += energyCounterMinutes[i];
				}
			}
            if ((energyCounterStatsJSONEnable == true) && (MQTT_IsReady() == true))
            {
                root = cJSON_CreateObject();
                cJSON_AddNumberToObject(root, "uptime", g_secondsElapsed);
                cJSON_AddNumberToObject(root, "consumption_total", BL_ChangeEnergyUnitIfNeeded(DRV_GetReading(OBK_CONSUMPTION_TOTAL)));
                cJSON_AddNumberToObject(root, "consumption_last_hour", BL_ChangeEnergyUnitIfNeeded(DRV_GetReading(OBK_CONSUMPTION_LAST_HOUR)));
                cJSON_AddNumberToObject(root, "consumption_stat_index", energyCounterMinutesIndex);
                cJSON_AddNumberToObject(root, "consumption_sample_count", energyCounterSampleCount);
                cJSON_AddNumberToObject(root, "consumption_sampling_period", energyCounterSampleInterval);
                if(NTP_IsTimeSynced() == true)
                {
                    cJSON_AddNumberToObject(root, "consumption_today", BL_ChangeEnergyUnitIfNeeded(DRV_GetReading(OBK_CONSUMPTION_TODAY)));
                    cJSON_AddNumberToObject(root, "consumption_yesterday", BL_ChangeEnergyUnitIfNeeded(DRV_GetReading(OBK_CONSUMPTION_YESTERDAY)));
                    ltm = gmtime(&ConsumptionResetTime);
                    if (NTP_GetTimesZoneOfsSeconds()>0)
                    {
                       snprintf(datetime,sizeof(datetime), "%04i-%02i-%02iT%02i:%02i+%02i:%02i",
                               ltm->tm_year+1900, ltm->tm_mon+1, ltm->tm_mday, ltm->tm_hour, ltm->tm_min,
                               NTP_GetTimesZoneOfsSeconds()/3600, (NTP_GetTimesZoneOfsSeconds()/60) % 60);
                    } else {
                       snprintf(datetime, sizeof(datetime), "%04i-%02i-%02iT%02i:%02i-%02i:%02i",
                               ltm->tm_year+1900, ltm->tm_mon+1, ltm->tm_mday, ltm->tm_hour, ltm->tm_min,
                               abs(NTP_GetTimesZoneOfsSeconds()/3600), (abs(NTP_GetTimesZoneOfsSeconds())/60) % 60);
                    }
                    cJSON_AddStringToObject(root, "consumption_clear_date", datetime);
                }

                if (energyCounterMinutes != NULL)
                {
                    stats = cJSON_CreateArray();
					// WARNING - it causes HA problems?
					// See: https://github.com/openshwprojects/OpenBK7231T_App/issues/870
					// Basically HA has 256 chars state limit?
					// Wait, no, it's over 256 even without samples?
                    for(i = 0; i < energyCounterSampleCount; i++)
                    {
                        cJSON_AddItemToArray(stats, cJSON_CreateNumber(energyCounterMinutes[i]));
                    }
                    cJSON_AddItemToObject(root, "consumption_samples", stats);
                }

                if(NTP_IsTimeSynced() == true)
                {
                    stats = cJSON_CreateArray();
                    for(i = OBK_CONSUMPTION__DAILY_FIRST; i <= OBK_CONSUMPTION__DAILY_LAST; i++)
                    {
                        cJSON_AddItemToArray(stats, cJSON_CreateNumber(DRV_GetReading(i)));
                    }
                    cJSON_AddItemToObject(root, "consumption_daily", stats);
                }

                msg = cJSON_PrintUnformatted(root);
                cJSON_Delete(root);

               // addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "JSON Printed: %d bytes", strlen(msg));

                MQTT_PublishMain_StringString("consumption_stats", msg, 0);
                stat_updatesSent++;
                os_free(msg);
            }

            if (energyCounterMinutes != NULL)
            {
                for (i=energyCounterSampleCount-1;i>0;i--)
                {
                // Modified to save negative energy 
		energyCounterMinutes[i] = energyCounterMinutes[i-1];   
		/* if (energyCounterMinutes[i-1]>0.0)
                    {
                        energyCounterMinutes[i] = energyCounterMinutes[i-1];
                    } else {
                        energyCounterMinutes[i] = 0.0;
                    }*/
                }
                energyCounterMinutes[0] = 0.0;
            }
            energyCounterMinutesStamp = xTaskGetTickCount();
            energyCounterMinutesIndex++;

        }

        if (energyCounterMinutes != NULL)
            //energyCounterMinutes[0] += energy;
	    
	// Changes below should allow it to measure both import and export
	energyCounterMinutes[0] += energy_counter_data;
	//energy_counter_data = 0;
	//-----------------------------------------------------------------------------------------------------------------------------
	/*if ((int)power>0)
	{
		sensors[OBK_CONSUMPTION_TOTAL].lastReading += energyWh;
		energyCounterMinutes[0] += (int)energyWh;
	}
	else
	{
		//If not positive, we check if the negative power flag is enabled. If so, we load the generation counter.
		if (CFG_HasFlag(OBK_FLAG_POWER_ALLOW_NEGATIVE))
		{
			// If the power is negative - Load the generation counter, but only if we allow negative measurements :-)
			sensors[OBK_GENERATION_TOTAL].lastReading += energyWh;	
			energyCounterMinutes[0] -= (int)energyWh;	
		}
	}*/
	//-----------------------------------------------------------------------------------------------------------------------------
    }

    for(i = OBK__FIRST; i <= OBK__LAST; i++)
    {
        // send update only if there was a big change or if certain time has passed
        // Do not send message with every measurement. 
		diff = sensors[i].lastSentValue - sensors[i].lastReading;
		// check for change
        if ( ((fabsf(diff) > sensors[i].changeSendThreshold) &&
               (sensors[i].noChangeFrame >= changeDoNotSendMinFrames)) ||
             (sensors[i].noChangeFrame >= changeSendAlwaysFrames) )
        {
            sensors[i].noChangeFrame = 0;

			enum EventCode eventChangeCode;
			switch (i) {
			case OBK_VOLTAGE:				eventChangeCode = CMD_EVENT_CHANGE_VOLTAGE;			break;
			case OBK_CURRENT:				eventChangeCode = CMD_EVENT_CHANGE_CURRENT;			break;
			case OBK_POWER:					eventChangeCode = CMD_EVENT_CHANGE_POWER; 			break;
			case OBK_CONSUMPTION_TOTAL:			eventChangeCode = CMD_EVENT_CHANGE_CONSUMPTION_TOTAL; 		break;
			case OBK_GENERATION_TOTAL:			eventChangeCode = CMD_EVENT_CHANGE_GENERATION_TOTAL; 		break;
			case OBK_CONSUMPTION_LAST_HOUR:			eventChangeCode = CMD_EVENT_CHANGE_CONSUMPTION_LAST_HOUR; 	break;
			default:					eventChangeCode = CMD_EVENT_NONE; 				break;
			}
			switch (eventChangeCode) {
			case CMD_EVENT_NONE:
				break;
			case CMD_EVENT_CHANGE_CURRENT: ;
                int prev_mA = sensors[i].lastSentValue * 1000;
                int now_mA = sensors[i].lastReading * 1000;
                EventHandlers_ProcessVariableChange_Integer(eventChangeCode, prev_mA,now_mA);
				break;
			default:
				EventHandlers_ProcessVariableChange_Integer(eventChangeCode, sensors[i].lastSentValue, sensors[i].lastReading);
				break;
			}

            if (MQTT_IsReady() == true)
            {
				sensors[i].lastSentValue = sensors[i].lastReading;
				if (i == OBK_CONSUMPTION_CLEAR_DATE) {
					sensors[i].lastReading = ConsumptionResetTime; //Only to make the 'nochangeframe' mechanism work here
					ltm = gmtime(&ConsumptionResetTime);
					/* 2019-09-07T15:50-04:00 */
					if (NTP_GetTimesZoneOfsSeconds()>0)
					{
						snprintf(datetime, sizeof(datetime), "%04i-%02i-%02iT%02i:%02i+%02i:%02i",
								ltm->tm_year+1900, ltm->tm_mon+1, ltm->tm_mday, ltm->tm_hour, ltm->tm_min,
								NTP_GetTimesZoneOfsSeconds()/3600, (NTP_GetTimesZoneOfsSeconds()/60) % 60);
					} else {
						snprintf(datetime, sizeof(datetime), "%04i-%02i-%02iT%02i:%02i-%02i:%02i",
								ltm->tm_year+1900, ltm->tm_mon+1, ltm->tm_mday, ltm->tm_hour, ltm->tm_min,
								abs(NTP_GetTimesZoneOfsSeconds()/3600), (abs(NTP_GetTimesZoneOfsSeconds())/60) % 60);
					}
					MQTT_PublishMain_StringString(sensors[i].names.name_mqtt, datetime, 0);
				} else { //all other sensors
					float val = sensors[i].lastReading;
					if (sensors[i].names.units == UNIT_WH) val = BL_ChangeEnergyUnitIfNeeded(val);
					MQTT_PublishMain_StringFloat(sensors[i].names.name_mqtt, val, sensors[i].rounding_decimals, 0);
				}
				stat_updatesSent++;
			}
        } else {
            // no change frame
            sensors[i].noChangeFrame++;
            stat_updatesSkipped++;
        }
    }        

// Save the total counters periodically  
// If we are measuring bi-directional we save every net metering period instead. This is about 100x a day.
if (((((sensors[OBK_CONSUMPTION_TOTAL].lastReading - lastSavedEnergyCounterValue) >= changeSavedThresholdEnergy) ||
        ((xTaskGetTickCount() - lastConsumptionSaveStamp) >= (6 * 3600 * 1000 / portTICK_PERIOD_MS)) || 
	((sensors[OBK_GENERATION_TOTAL].lastReading - lastSavedGenerationCounterValue) >= changeSavedThresholdEnergy)) && (!(CFG_HasFlag(OBK_FLAG_POWER_ALLOW_NEGATIVE))))||(savetoflash == 1))
    {

savetoflash = 0;
#if WINDOWS
#elif PLATFORM_BL602
#elif PLATFORM_W600 || PLATFORM_W800
#elif PLATFORM_XR809
#elif PLATFORM_BK7231N || PLATFORM_BK7231T
        if (ota_progress() == -1)
#endif
        {
            // Save Consumption
	    lastSavedEnergyCounterValue = sensors[OBK_CONSUMPTION_TOTAL].lastReading;
	    // Save Generation
	    lastSavedGenerationCounterValue = sensors[OBK_GENERATION_TOTAL].lastReading;
            BL09XX_SaveEmeteringStatistics();
            lastConsumptionSaveStamp = xTaskGetTickCount();
        }
    }
}

void BL_Shared_Init(void)
{
    int i;
    ENERGY_METERING_DATA data;

    for(i = OBK__FIRST; i <= OBK__LAST; i++)
    {
        sensors[i].noChangeFrame = 0;
        sensors[i].lastReading = 0;
    }
    energyCounterStamp = xTaskGetTickCount(); 

    if (energyCounterStatsEnable == true)
    {
        if (energyCounterMinutes == NULL)
        {
            energyCounterMinutes = (float*)os_malloc(energyCounterSampleCount*sizeof(float));
        }
        if (energyCounterMinutes != NULL)
        {
            for(i = 0; i < energyCounterSampleCount; i++)
            {
                energyCounterMinutes[i] = 0.0;
            }   
        }
        energyCounterMinutesStamp = xTaskGetTickCount();
        energyCounterMinutesIndex = 0;
    }

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "Read ENERGYMETER status values. sizeof(ENERGY_METERING_DATA)=%d\n", sizeof(ENERGY_METERING_DATA));

    HAL_GetEnergyMeterStatus(&data);
    sensors[OBK_CONSUMPTION_TOTAL].lastReading = data.TotalConsumption;
    sensors[OBK_GENERATION_TOTAL].lastReading = data.TotalGeneration;
    sensors[OBK_CONSUMPTION_TODAY].lastReading = data.TodayConsumpion;
    sensors[OBK_CONSUMPTION_YESTERDAY].lastReading = data.YesterdayConsumption;
    actual_mday = data.actual_mday;    
    lastSavedEnergyCounterValue = data.TotalConsumption;
    lastSavedGenerationCounterValue = data.TotalGeneration;
    sensors[OBK_CONSUMPTION_2_DAYS_AGO].lastReading = data.ConsumptionHistory[0];
    sensors[OBK_CONSUMPTION_3_DAYS_AGO].lastReading = data.ConsumptionHistory[1];
    ConsumptionResetTime = data.ConsumptionResetTime;
    ConsumptionSaveCounter = data.save_counter;
    lastConsumptionSaveStamp = xTaskGetTickCount();

    //int HAL_SetEnergyMeterStatus(ENERGY_METERING_DATA *data);

	//cmddetail:{"name":"EnergyCntReset","args":"[OptionalNewValue]",
	//cmddetail:"descr":"Resets the total Energy Counter, the one that is usually kept after device reboots. After this commands, the counter will start again from 0 (or from the value you specified).",
	//cmddetail:"fn":"BL09XX_ResetEnergyCounter","file":"driver/drv_bl_shared.c","requires":"",
	//cmddetail:"examples":""}
  	CMD_RegisterCommand("EnergyCntReset", BL09XX_ResetEnergyCounter, NULL);
	//cmddetail:{"name":"SetupEnergyStats","args":"[Enable1or0][SampleTime][SampleCount][JSonEnable]",
	//cmddetail:"descr":"Setup Energy Statistic Parameters: [enable 0 or 1] [sample_time[10..90]] [sample_count[10..180]] [JsonEnable 0 or 1]. JSONEnable is optional.",
	//cmddetail:"fn":"BL09XX_SetupEnergyStatistic","file":"driver/drv_bl_shared.c","requires":"",
	//cmddetail:"examples":""}
  	CMD_RegisterCommand("SetupEnergyStats", BL09XX_SetupEnergyStatistic, NULL);
	//cmddetail:{"name":"ConsumptionThreshold","args":"[FloatValue]",
	//cmddetail:"descr":"Setup value for automatic save of consumption data [1..100]",
	//cmddetail:"fn":"BL09XX_SetupConsumptionThreshold","file":"driver/drv_bl_shared.c","requires":"",
	//cmddetail:"examples":""}
    	CMD_RegisterCommand("ConsumptionThreshold", BL09XX_SetupConsumptionThreshold, NULL);
	//cmddetail:{"name":"VCPPublishThreshold","args":"[VoltageDeltaVolts][CurrentDeltaAmpers][PowerDeltaWats][EnergyDeltaWh]",
	//cmddetail:"descr":"Sets the minimal change between previous reported value over MQTT and next reported value over MQTT. Very useful for BL0942, BL0937, etc. So, if you set, VCPPublishThreshold 0.5 0.001 0.5, it will only report voltage again if the delta from previous reported value is largen than 0.5V. Remember, that the device will also ALWAYS force-report values every N seconds (default 60)",
	//cmddetail:"fn":"BL09XX_VCPPublishThreshold","file":"driver/drv_bl_shared.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("VCPPublishThreshold", BL09XX_VCPPublishThreshold, NULL);
	//cmddetail:{"name":"VCPPrecision","args":"[VoltageDigits][CurrentDigitsAmpers][PowerDigitsWats][EnergyDigitsWh]",
	//cmddetail:"descr":"Sets the number of digits after decimal point for power metering publishes. Default is BL09XX_VCPPrecision 1 3 2 3. This works for OBK-style publishes.",
	//cmddetail:"fn":"BL09XX_VCPPrecision","file":"driver/drv_bl_shared.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("VCPPrecision", BL09XX_VCPPrecision, NULL);
	//cmddetail:{"name":"VCPPublishIntervals","args":"[MinDelayBetweenPublishes][ForcedPublishInterval]",
	//cmddetail:"descr":"First argument is minimal allowed interval in second between Voltage/Current/Power/Energy publishes (even if there is a large change), second value is an interval in which V/C/P/E is always published, even if there is no change",
	//cmddetail:"fn":"BL09XX_VCPPublishIntervals","file":"driver/drv_bl_shared.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("VCPPublishIntervals", BL09XX_VCPPublishIntervals, NULL);
}

// OBK_POWER etc
float DRV_GetReading(energySensor_t type) 
{
	return sensors[type].lastReading;
}

energySensorNames_t* DRV_GetEnergySensorNames(energySensor_t type)
{
	return &sensors[type].names;
}
