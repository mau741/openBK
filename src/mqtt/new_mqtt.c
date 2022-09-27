
#include "new_mqtt.h"
#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../hal/hal_wifi.h"
#include "../driver/drv_public.h"
#include "../driver/drv_ntp.h"

#ifndef LWIP_MQTT_EXAMPLE_IPADDR_INIT
#if LWIP_IPV4
#define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(IPADDR_LOOPBACK))
#else
#define LWIP_MQTT_EXAMPLE_IPADDR_INIT
#endif
#endif
extern smartblub_config_data_t smartblub_config_data;

int wal_stricmp(const char *a, const char *b) {
  int ca, cb;
  do {
     ca = (unsigned char) *a++;
     cb = (unsigned char) *b++;
     ca = tolower(toupper(ca));
     cb = tolower(toupper(cb));
   } while ((ca == cb) && (ca != '\0'));
   return ca - cb;
}
int wal_strnicmp(const char *a, const char *b, int count) {
  int ca, cb;
  do {
     ca = (unsigned char) *a++;
     cb = (unsigned char) *b++;
     ca = tolower(toupper(ca));
     cb = tolower(toupper(cb));
     count--;
   } while ((ca == cb) && (ca != '\0') && (count > 0));
   return ca - cb;
}

// from mqtt.c
extern void mqtt_disconnect(mqtt_client_t *client);

static int g_my_reconnect_mqtt_after_time = -1;
ip_addr_t mqtt_ip LWIP_MQTT_EXAMPLE_IPADDR_INIT;
mqtt_client_t* mqtt_client;
static int g_timeSinceLastMQTTPublish = 0;
static int mqtt_initialised = 0;

typedef struct mqtt_callback_tag {
    char *topic;
    char *subscriptionTopic;
    int ID;
    mqtt_callback_fn callback;
} mqtt_callback_t;

#define MAX_MQTT_CALLBACKS 32
static mqtt_callback_t *callbacks[MAX_MQTT_CALLBACKS];
static int numCallbacks = 0;
// note: only one incomming can be processed at a time.
static mqtt_request_t g_mqtt_request;

int loopsWithDisconnected = 0;
int mqtt_reconnect = 0;
// set for the device to broadcast self state on start
int g_bPublishAllStatesNow = 0;

#define PUBLISHITEM_ALL_INDEX_FIRST   -14

//These 3 values are pretty much static
#define PUBLISHITEM_SELF_STATIC_RESERVED_2		-14
#define PUBLISHITEM_SELF_STATIC_RESERVED_1		-13
#define PUBLISHITEM_SELF_HOSTNAME				-12  //Device name
#define PUBLISHITEM_SELF_BUILD					-11  //Build
#define PUBLISHITEM_SELF_MAC					-10   //Device mac

#define PUBLISHITEM_DYNAMIC_INDEX_FIRST			-9

//These values are dynamic
#define PUBLISHITEM_SELF_DYNAMIC_LIGHTSTATE		-9
#define PUBLISHITEM_SELF_DYNAMIC_LIGHTMODE		-8
#define PUBLISHITEM_SELF_DYNAMIC_DIMMER			-7
#define PUBLISHITEM_SELF_DATETIME				-6  //Current unix datetime
#define PUBLISHITEM_SELF_SOCKETS				-5  //Active sockets
#define PUBLISHITEM_SELF_RSSI					-4  //Link strength
#define PUBLISHITEM_SELF_UPTIME					-3  //Uptime
#define PUBLISHITEM_SELF_FREEHEAP				-2  //Free heap
#define PUBLISHITEM_SELF_IP						-1  //ip address

int g_publishItemIndex = PUBLISHITEM_ALL_INDEX_FIRST;
static bool g_firstFullBroadcast = true;  //Flag indicating that we need to do a full broadcast

int g_memoryErrorsThisSession = 0;
static SemaphoreHandle_t g_mutex = 0;

static bool MQTT_Mutex_Take(int del) {
	int taken;

	if(g_mutex == 0)
	{
		g_mutex = xSemaphoreCreateMutex( );
	}
    taken = xSemaphoreTake( g_mutex, del );
    if (taken == pdTRUE) {
		return true;
	}
	return false;
}
static void MQTT_Mutex_Free() {
	xSemaphoreGive( g_mutex );
}

void MQTT_PublishWholeDeviceState() {
	g_bPublishAllStatesNow = 1;

  //Publish all status items once. Publish only dynamic items after that.
  g_publishItemIndex = g_firstFullBroadcast == true ? PUBLISHITEM_ALL_INDEX_FIRST:PUBLISHITEM_DYNAMIC_INDEX_FIRST;
}

void MQTT_PublishOnlyDeviceChannelsIfPossible() {
	if(g_bPublishAllStatesNow == 1)
		return;
	g_bPublishAllStatesNow = 1;

  //Start with channels
  g_publishItemIndex = 0;
}
static struct mqtt_connect_client_info_t mqtt_client_info =
{
  "test",
  // do not fil those settings, they are overriden when read from memory
  "user", /* user */
  "pass", /* pass */
  120,  /* keep alive */
  NULL, /* will_topic */
  NULL, /* will_msg */
  0,    /* will_qos */
  0     /* will_retain */
#if LWIP_ALTCP && LWIP_ALTCP_TLS
  , NULL
#endif
};

// channel set callback
int channelSet(mqtt_request_t* request);
static void MQTT_do_connect(mqtt_client_t *client);
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);


// this can REPLACE callbacks, since we MAY wish to change the root topic....
// in which case we would re-resigster all callbacks?
int MQTT_RegisterCallback( const char *basetopic, const char *subscriptiontopic, int ID, mqtt_callback_fn callback){
  int index;
  int i;
  int subscribechange = 0;
	if (!basetopic || !subscriptiontopic || !callback){
		return -1;
	}
  addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"MQTT_RegisterCallback called for bT %s subT %s", basetopic, subscriptiontopic);

  // find existing to replace
  for (index = 0; index < numCallbacks; index++){
    if (callbacks[index]){
      if (callbacks[index]->ID == ID){
        break;
      }
    }
  }

  // find empty if any (empty by MQTT_RemoveCallback)
  if (index == numCallbacks){
    for (index = 0; index < numCallbacks; index++){
      if (!callbacks[index]){
        break;
      }
    }
  }

	if (index >= MAX_MQTT_CALLBACKS){
		return -4;
	}
  if (!callbacks[index]){
	  callbacks[index] = (mqtt_callback_t*)os_malloc(sizeof(mqtt_callback_t));
	  if(callbacks[index]!=0) {
		memset(callbacks[index],0,sizeof(mqtt_callback_t));
	  }
  }
	if (!callbacks[index]){
		return -2;
	}
  if (!callbacks[index]->topic || strcmp(callbacks[index]->topic, basetopic)){
    if (callbacks[index]->topic){
      os_free(callbacks[index]->topic);
    }
    callbacks[index]->topic = (char *)os_malloc(strlen(basetopic)+1);
    if (!callbacks[index]->topic){
      os_free(callbacks[index]);
      return -3;
    }
    strcpy(callbacks[index]->topic, basetopic);
  }

  if (!callbacks[index]->subscriptionTopic || strcmp(callbacks[index]->subscriptionTopic, subscriptiontopic)){
    if (callbacks[index]->subscriptionTopic){
      os_free(callbacks[index]->subscriptionTopic);
    }
    callbacks[index]->subscriptionTopic = (char *)os_malloc(strlen(subscriptiontopic)+1);
    callbacks[index]->subscriptionTopic[0] = '\0';
    if (!callbacks[index]->subscriptionTopic){
      os_free(callbacks[index]->topic);
      os_free(callbacks[index]);
      return -3;
    }

    // find out if this subscription is new.
    for (i = 0; i < numCallbacks; i++){
      if (callbacks[i]){
        if (callbacks[i]->subscriptionTopic &&
            !strcmp(callbacks[i]->subscriptionTopic, subscriptiontopic)){
          break;
        }
      }
    }
    strcpy(callbacks[index]->subscriptionTopic, subscriptiontopic);
    // if this subscription is new, must reconnect
    if (i == numCallbacks){
      subscribechange++;
    }
  }

	callbacks[index]->callback = callback;
  if (index == numCallbacks){
	  numCallbacks++;
  }

  if (subscribechange){
    mqtt_reconnect = 8;
  }
	// success
	return 0;
}

int MQTT_RemoveCallback(int ID){
  int index;

  for (index = 0; index < numCallbacks; index++){
    if (callbacks[index]){
      if (callbacks[index]->ID == ID){
        if (callbacks[index]->topic) {
          os_free(callbacks[index]->topic);
          callbacks[index]->topic = NULL;
        }
        if (callbacks[index]->subscriptionTopic) {
          os_free(callbacks[index]->subscriptionTopic);
          callbacks[index]->subscriptionTopic = NULL;
        }
    		os_free(callbacks[index]);
        callbacks[index] = NULL;
        mqtt_reconnect = 8;
        return 1;
      }
    }
  }
  return 0;
}



// this accepts cmnd/<basename>/<xxx> to receive data to set channels
int tasCmnd(mqtt_request_t* request){
  // we only need a few bytes to receive a decimal number 0-100
  char copy[64];
  int len = request->receivedLen;
  const char *p = request->topic;

  // assume a string input here, copy and terminate
  if(len > sizeof(copy)-1) {
    len = sizeof(copy)-1;
  }
  strncpy(copy, (char *)request->received, len);
  // strncpy does not terminate??!!!!
  copy[len] = '\0';

  // TODO: better
  // skip to after second forward slash
  while(*p != '/') { if(*p == 0) return 0; p++; }
  p++;
  while(*p != '/') { if(*p == 0) return 0; p++; }
  p++;

  // use command executor....
  CMD_ExecuteCommandArgs(p, copy, COMMAND_FLAG_SOURCE_MQTT);

  // return 1 to stop processing callbacks here.
  // return 0 to allow later callbacks to process this topic.
  return 1;
}

//void MQTT_GetStats(int *outUsed, int *outMax, int *outFreeMem) {
//	mqtt_get_request_stats(mqtt_client,outUsed, outMax,outFreeMem);
//}

// copied here because for some reason renames in sdk?
static void MQTT_disconnect(mqtt_client_t *client)
{
  if (!client)
	  return;
  addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"CAME TO MQTT DISCONNECT\n");
  // this is what it was renamed to.  why?
  mqtt_disconnect(client);
}

/* Called when publish is complete either with sucess or failure */
static void mqtt_pub_request_cb(void *arg, err_t result)
{
  if(result != ERR_OK) {
  addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"Publish result: %d\n", result);
  }
}

// This is used to publish channel values in "obk0696FB33/1/get" format with numerical value,
// This is also used to publish custom information with string name,
// for example, "obk0696FB33/voltage/get" is used to publish voltage from the sensor
static OBK_Publish_Result MQTT_PublishMain(mqtt_client_t *client, const char *sChannel, const char *sVal, int flags, bool appendGet)
{
	char pub_topic[32];
//  const char *pub_payload= "{\"temperature\": \"45.5\"}";
  err_t err;
  //int myValue;
  u8_t qos = 2; /* 0 1 or 2, see MQTT specification */
  u8_t retain = 0; /* No don't retain such crappy payload... */
	const char *baseName;



  if(client==0)
	  return OBK_PUBLISH_WAS_DISCONNECTED;


	if(flags & OBK_PUBLISH_FLAG_MUTEX_SILENT) {
		if(MQTT_Mutex_Take(100)==0) {
			return OBK_PUBLISH_MUTEX_FAIL;
		}
	} else {
		if(MQTT_Mutex_Take(500)==0) {
			addLogAdv(LOG_ERROR,LOG_FEATURE_MQTT,"MQTT_PublishMain: mutex failed for %s=%s\r\n", sChannel, sVal);
			return OBK_PUBLISH_MUTEX_FAIL;
		}
	}
	if(flags & OBK_PUBLISH_FLAG_RETAIN) {
		retain = 1;
	}
	// global tool
	if(CFG_HasFlag(OBK_FLAG_MQTT_ALWAYSSETRETAIN)) {
		retain = 1;
	}


  if(mqtt_client_is_connected(client)==0) {
		 g_my_reconnect_mqtt_after_time = 5;
		MQTT_Mutex_Free();
		return OBK_PUBLISH_WAS_DISCONNECTED;
  }

  g_timeSinceLastMQTTPublish = 0;

  addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"Publishing %s = %s \n",sChannel,sVal);
  uint8_t mac[6];
  char ble_name[20];
      		wifi_get_mac_address((char *)mac, 1);
      		snprintf(ble_name, sizeof(ble_name), "%02x%02x", mac[4], mac[5]);
	baseName = CFG_GetShortDeviceName();
	sprintf(pub_topic,"%s/res/%s",baseName,ble_name);

	//sprintf(pub_topic,"%s/%s%s",baseName,sChannel, (appendGet == true ? "/get" : ""));
  err = mqtt_publish(client, pub_topic, sVal, strlen(sVal), qos, retain, mqtt_pub_request_cb, 0);
  if(err != ERR_OK) {
	 if(err == ERR_CONN) {
		addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"Publish err: ERR_CONN aka %d\n", err);
	 } else if(err == ERR_MEM) {
		addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"Publish err: ERR_MEM aka %d\n", err);
		g_memoryErrorsThisSession ++;
	 } else {
		addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"Publish err: %d\n", err);
	 }
  }
	MQTT_Mutex_Free();
	return OBK_PUBLISH_OK;
}
void MQTT_OBK_Printf( char *s) {
		addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,s);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
  int i;
  // unused - left here as example
  //const struct mqtt_connect_client_info_t* client_info = (const struct mqtt_connect_client_info_t*)arg;

  // if we stored a topic in g_mqtt_request, then we found a matching callback, so use it.
  if (g_mqtt_request.topic[0]) {
    // note: data is NOT terminated (it may be binary...).
    g_mqtt_request.received = data;
    g_mqtt_request.receivedLen = len;

  addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"MQTT in topic %s", g_mqtt_request.topic);

    for (i = 0; i < numCallbacks; i++){
      char *cbtopic = callbacks[i]->topic;
      if (!strncmp(g_mqtt_request.topic, cbtopic, strlen(cbtopic))){
        // note - callback must return 1 to say it ate the mqtt, else further processing can be performed.
        // i.e. multiple people can get each topic if required.
        if (callbacks[i]->callback(&g_mqtt_request)){
          return;
        }
      }
    }
  }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	//const char *p;
  int i;
  // unused - left here as example
  //const struct mqtt_connect_client_info_t* client_info = (const struct mqtt_connect_client_info_t*)arg;

	// look for a callback with this URL and method, or HTTP_ANY
  g_mqtt_request.topic[0] = '\0';
	for (i = 0; i < numCallbacks; i++){
		char *cbtopic = callbacks[i]->topic;
		if (strncmp(topic, cbtopic, strlen(cbtopic))){
      strncpy(g_mqtt_request.topic, topic, sizeof(g_mqtt_request.topic) - 1);
      g_mqtt_request.topic[sizeof(g_mqtt_request.topic) - 1] = 0;
      break;
		}
	}

  addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"MQTT client in mqtt_incoming_publish_cb topic %s\n",topic);
}
// this accepts obkXXXXXX/<chan>/set to receive data to set channels
int channelSet(mqtt_request_t* request){
  // we only need a few bytes to receive a decimal number 0-100
  char copy[12];
  int len = request->receivedLen;
  char *p = request->topic;
  int channel = 0;
  char *array[80];
	const char pub_payload[150];
  int j=0;
  static char mystring[500];
  memset(mystring,'\0',sizeof(mystring));
  char blub_data[100];
  memcpy(mystring,request->received, len);
 addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"mystring %s", mystring);
 addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"request->received %s", request->received);
 addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"request->topic %s", request->topic);
 addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"len %d", len);



 if (strncmp(mystring, "{\"CONF_REQ\"}", 12) == 0) {

	 snprintf(blub_data, sizeof(blub_data), "{\"RED\":%d,\"BLUE\":%d,\"GREEN\":%d,\"WHITE\":%d}",smartblub_config_data.r_brightness,smartblub_config_data.b_brightness,smartblub_config_data.g_brightness, smartblub_config_data.w_brightness);
		 snprintf(pub_payload, sizeof(pub_payload),"{\"CONF\":%s}",blub_data);
		       addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"calling pub: \n");

		 	 MQTT_PublishMain(mqtt_client,0,pub_payload, 0, true);
  }

 else if (*mystring == '{') {

	 twProcessConfig(mystring);
	 snprintf(blub_data, sizeof(blub_data), "{\"RED\":%d,\"BLUE\":%d,\"GREEN\":%d,\"WHITE\":%d}",smartblub_config_data.r_brightness,smartblub_config_data.b_brightness,smartblub_config_data.g_brightness, smartblub_config_data.w_brightness);
	 snprintf(pub_payload, sizeof(pub_payload),"{\"CONF\":%s}",blub_data);
	       addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"calling pub: \n");
	       if(smartblub_config_data.red_led==true)
	       {

	       	addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," red\n");
	       	smartblub_config_data.red_led=false;
	       	 CHANNEL_Set(smartblub_config_data.r_channel,smartblub_config_data.r_brightness,smartblub_config_data.r_pin);

	       }
	       if(smartblub_config_data.green_led==true)
	       {
	       	smartblub_config_data.green_led=false;
	       	 CHANNEL_Set(smartblub_config_data.g_channel,smartblub_config_data.g_brightness,smartblub_config_data.g_pin);

	       }
	       if(smartblub_config_data.blue_led==true)
	       {
	       	smartblub_config_data.blue_led=false;
	       	 CHANNEL_Set(smartblub_config_data.b_channel,smartblub_config_data.b_brightness,smartblub_config_data.b_pin);

	       }
	       if(smartblub_config_data.white_led==true)
	       {
	       	smartblub_config_data.white_led=false;
	       	 CHANNEL_Set(smartblub_config_data.w_channel,smartblub_config_data.w_brightness,smartblub_config_data.w_pin);
	       }
	       if(smartblub_config_data.led_offall==true)
	       {
	       	smartblub_config_data.led_offall=false;
	       	 CHANNEL_Set(smartblub_config_data.r_channel,smartblub_config_data.r_brightness,smartblub_config_data.r_pin);
	       		 CHANNEL_Set(smartblub_config_data.b_channel,smartblub_config_data.b_brightness,smartblub_config_data.b_pin);
	       		 CHANNEL_Set(smartblub_config_data.g_channel,smartblub_config_data.g_brightness,smartblub_config_data.g_pin);
	       		 CHANNEL_Set(smartblub_config_data.w_channel,smartblub_config_data.w_brightness,smartblub_config_data.w_pin);

	       }
	       g_cfg_pendingChanges = 3;
	       	 CFG_Save_IfThereArePendingChanges() ;
	 	 MQTT_PublishMain(mqtt_client,0,pub_payload, 0, true);

  }



addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.red_led=%d", smartblub_config_data.red_led);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.green_led=%d", smartblub_config_data.green_led);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.blue_led=%d", smartblub_config_data.blue_led);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.white_led=%d", smartblub_config_data.white_led);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.led_offall=%d", smartblub_config_data.led_offall);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.r_brightness=%d", smartblub_config_data.r_brightness);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.b_brightness=%d", smartblub_config_data.b_brightness);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.g_brightness=%d", smartblub_config_data.g_brightness);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.w_brightness=%d", smartblub_config_data.w_brightness);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.r_pin=%d", smartblub_config_data.r_pin);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.b_pin=%d", smartblub_config_data.b_pin);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.g_pin=%d", smartblub_config_data.g_pin);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.w_pin=%d", smartblub_config_data.w_pin);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.r_channel=%d", smartblub_config_data.r_channel);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.b_channel=%d", smartblub_config_data.b_channel);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.g_channel=%d", smartblub_config_data.g_channel);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT," smartblub_config_data.w_channel=%d", smartblub_config_data.w_channel);




return 1;
}

static void
mqtt_request_cb(void *arg, err_t err)
{
  const struct mqtt_connect_client_info_t* client_info = (const struct mqtt_connect_client_info_t*)arg;

  addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"MQTT client \"%s\" request cb: err %d\n", client_info->client_id, (int)err);
}
static void mqtt_sub_request_cb(void *arg, err_t result)
{
  /* Just print the result code here for simplicity,
     normal behaviour would be to take some action if subscribe fails like
     notifying user, retry subscribe or disconnect from server */
    addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"Subscribe result: %i\n", result);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
  int i;
	char tmp[64];
	const char *baseName;
	 addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"MQTT_STATUS=%d",status);
	char *mqtt_clientID;
  err_t err = ERR_OK;
	char my_mac[20];
	const char pub_payload[150];
	char blub_data[100];
  const struct mqtt_connect_client_info_t* client_info = (const struct mqtt_connect_client_info_t*)arg;
  LWIP_UNUSED_ARG(client);
  addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"MQTT_STATUS=%d",status);
  uint8_t mac[6];
  char ble_name[20];
  const char *my_status= "\"ONLINE\"";
	wifi_get_mac_address((char *)mac, 1);
	snprintf(ble_name, sizeof(ble_name), "%02x%02x", mac[4], mac[5]);
	 char *config_reply;
	 int fwv;
	 mqtt_clientID = CFG_device_id();
	  addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"mqtt_clientID in online =%s",mqtt_clientID);
	snprintf(ble_name, sizeof(ble_name), "%02x%02x", mac[4], mac[5]);

	snprintf(my_mac, sizeof(my_mac), "\"%02x:%02x:%02x:%02x:%02x:%02x\"", mac[5], mac[4], mac[3], mac[2], mac[1],mac[0]);

	snprintf(blub_data, sizeof(blub_data), "{\"RED\":%d,\"BLUE\":%d,\"GREEN\":%d,\"WHITE\":%d}",smartblub_config_data.r_brightness,smartblub_config_data.b_brightness,smartblub_config_data.g_brightness, smartblub_config_data.w_brightness);
	 addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"my_mac=%s",my_mac);
	 addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"blub_data=%s",blub_data);

//   addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"MQTT client < removed name > connection cb: status %d\n",  (int)status);
 //  addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"MQTT client \"%s\" connection cb: status %d\n", client_info->client_id, (int)status);

  if (status == MQTT_CONNECT_ACCEPTED) {
    addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"mqtt_connection_cb: Successfully connected\n");

    mqtt_set_inpub_callback(mqtt_client,
          mqtt_incoming_publish_cb,
          mqtt_incoming_data_cb,
          LWIP_CONST_CAST(void*, &mqtt_client_info));

    // subscribe to all callback subscription topics
    // this makes a BIG assumption that we can subscribe multiple times to the same one?
    // TODO - check that subscribing multiple times to the same topic is not BAD
    for (i = 0; i < numCallbacks; i++){
      if (callbacks[i]){
        if (callbacks[i]->subscriptionTopic && callbacks[i]->subscriptionTopic[0]){
          err = mqtt_sub_unsub(client,
            callbacks[i]->subscriptionTopic, 1,
            mqtt_request_cb, LWIP_CONST_CAST(void*, client_info),
            1);
          if(err != ERR_OK) {
            addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"mqtt_subscribe to %s return: %d\n", callbacks[i]->subscriptionTopic, err);
          } else {
            addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"mqtt_subscribed to %s\n", callbacks[i]->subscriptionTopic);
          }
        }
      }
    }

  	baseName = CFG_GetShortDeviceName();

  	 sprintf(tmp,"%s/res/%s",baseName,ble_name);
  	    addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"tmp=%s\n",tmp);
  	    addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"res");
  	 //  pub_payload =  "{\"FWV\":11}";
  	    fwv=11;
  	   // sprintf(pub_payload,"{\"FWV\":%d}",fwv);


  	   // snprintf(pub_payload, sizeof(pub_payload),"{\"STATUS\":%s,\"FWV\":%d,\"MAC\":%s,\"CONF\":%s}",my_status,fwv,my_mac,blub_data);

  	  snprintf(pub_payload, sizeof(pub_payload),"{\"STATUS\":%s,\"S_NO\":\"%s\",\"FWV\":%d,\"MAC\":%s,\"CONF\":%s}",my_status,mqtt_clientID,fwv,my_mac,blub_data);
  	    addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"pub_payload=%s",pub_payload);
  	  MQTT_PublishMain(mqtt_client,0,pub_payload, 0, true);
  	   // err = mqtt_publish(client,tmp,pub_payload,strlen(pub_payload), 2,true,mqtt_pub_request_cb,0);
    if(err != ERR_OK) {
      addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"Publish err: %d\n", err);
      if(err == ERR_CONN) {
      // g_my_reconnect_mqtt_after_time = 5;
      }
    }

	// publish all values on state
	MQTT_PublishWholeDeviceState();

    //mqtt_sub_unsub(client,
    //        "topic_qos1", 1,
    //        mqtt_request_cb, LWIP_CONST_CAST(void*, client_info),
    //        1);
    //mqtt_sub_unsub(client,
    //        "topic_qos0", 0,
    //        mqtt_request_cb, LWIP_CONST_CAST(void*, client_info),
    //        1);
  } else {
    addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"mqtt_connection_cb: Disconnected, reason: %d\n", status);
  }
}

static void MQTT_do_connect(mqtt_client_t *client)
{
  const char *mqtt_userName, *mqtt_host, *mqtt_pass, *mqtt_clientID,*wifi_ssid, *wifi_pass, *mqtt_topic;
  int mqtt_port;

	//const char  *mqtt_host,*mqtt_port;
  int res;
  const char status[50]= "\"OFFLINE\"";
//
  struct hostent* hostEntry;
  char will_topic[32];
  int fwv;
  fwv=11;
	uint8_t mac[6];
	const char pub_payload[300];
	char blub_data[100];
char my_mac[20];
char ble_name[20];
		wifi_get_mac_address((char *)mac, 1);
		snprintf(ble_name, sizeof(ble_name), "%02x%02x", mac[4], mac[5]);
		snprintf(my_mac, sizeof(my_mac), "\"%02x:%02x:%02x:%02x:%02x:%02x\"", mac[5],mac[4], mac[3], mac[2], mac[1],mac[0]);
		snprintf(blub_data, sizeof(blub_data), "{\"RED\":%d,\"BLUE\":%d,\"GREEN\":%d,\"WHITE\":%d}",smartblub_config_data.r_brightness,smartblub_config_data.b_brightness,smartblub_config_data.g_brightness, smartblub_config_data.w_brightness);
		 addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"my_mac=%s",my_mac);
		addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"blub_data=%s",blub_data);
  sprintf(will_topic,"%s/res/%s",CFG_GetShortDeviceName(),ble_name);
//
  mqtt_userName = CFG_GetMQTTUserName();
  mqtt_pass = CFG_GetMQTTPass();
  //mqtt_clientID = CFG_GetMQTTBrokerName();
  mqtt_clientID = CFG_device_id();
  mqtt_host = CFG_GetMQTTHost();
	mqtt_port = CFG_GetMQTTPort();
	mqtt_topic=CFG_mqtttopic();
	wifi_ssid = CFG_GetWiFiSSID();
wifi_pass = CFG_GetWiFiPass();
char *cbtopicsub;


sprintf(cbtopicsub,"%s/config/%s/%s",CFG_GetShortDeviceName(),mqtt_topic,ble_name);

snprintf(pub_payload, sizeof(pub_payload),"{\"STATUS\":%s,\"S_NO\":\"%s\",\"FWV\":%d,\"MAC\":%s,\"CONF\":%s}",status,mqtt_clientID,fwv,my_mac,blub_data);
	    addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"pub_payload=%s",pub_payload);
  addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"mqtt_userName %s\r\nmqtt_pass %s\r\nmqtt_clientID %s\r\nmqtt_host %s:%d\r\n",
    mqtt_userName,
    mqtt_pass,
    mqtt_clientID,
    mqtt_host,
    mqtt_port
  );


  if (!mqtt_host[0]){
    addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"mqtt_host empty, not starting mqtt\r\n");
    return;
  }
  addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "status=%s",status);
  // set pointer, there are no buffers to strcpy
  mqtt_client_info.client_id = mqtt_clientID;
  mqtt_client_info.client_pass = mqtt_pass;
  mqtt_client_info.client_user = mqtt_userName;
  mqtt_client_info.keep_alive=75;

  mqtt_client_info.will_topic = will_topic;
  mqtt_client_info.will_msg = pub_payload;
  mqtt_client_info.will_retain = false,
  mqtt_client_info.will_qos = 2,


addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"client_id %s\r\n",mqtt_client_info.client_id);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"client_pass %s\r\n",mqtt_client_info.client_pass);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"client_user %s\r\n",mqtt_client_info.client_user);
addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"mqtt_client_info.keep_alive %d\r\n",mqtt_client_info.keep_alive);

  hostEntry = gethostbyname(mqtt_host);
  if (NULL != hostEntry){
    if (hostEntry->h_addr_list && hostEntry->h_addr_list[0]){
      int len = hostEntry->h_length;
      if (len > 4){
        addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"mqtt_host resolves to addr len > 4\r\n");
        len = 4;
      }
      memcpy(&mqtt_ip, hostEntry->h_addr_list[0], len);
    } else {
      addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"mqtt_host resolves no addresses?\r\n");
      return;
    }

    // host name/ip
    //ipaddr_aton(mqtt_host,&mqtt_ip);

    /* Initiate client and connect to server, if this fails immediately an error code is returned
      otherwise mqtt_connection_cb will be called with connection result after attempting
      to establish a connection with the server.
      For now MQTT version 3.1.1 is always used */

   res = mqtt_client_connect(mqtt_client,
            &mqtt_ip, mqtt_port,
            mqtt_connection_cb, LWIP_CONST_CAST(void*, &mqtt_client_info),
            &mqtt_client_info);
   if(res != ERR_OK) {
      addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"Connect error in mqtt_client_connect - code: %d\n", res);

    }

  } else {
   addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"mqtt_host %s not found by gethostbyname\r\n", mqtt_host);
  }
}
OBK_Publish_Result MQTT_PublishMain_StringInt(const char *sChannel, int iv)
{
	char valueStr[16];

	sprintf(valueStr,"%i",iv);

	return MQTT_PublishMain(mqtt_client,sChannel,valueStr, 0, true);

}
OBK_Publish_Result MQTT_PublishMain_StringFloat(const char *sChannel, float f)
{
	char valueStr[16];

	sprintf(valueStr,"%f",f);

	return MQTT_PublishMain(mqtt_client,sChannel,valueStr, 0, true);

}
OBK_Publish_Result MQTT_PublishMain_StringString(const char *sChannel, const char *valueStr, int flags)
{

	return MQTT_PublishMain(mqtt_client,sChannel,valueStr, flags, true);

}
OBK_Publish_Result MQTT_ChannelChangeCallback(int channel, int iVal)
{
	char channelNameStr[8];
	char valueStr[16];

   addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "Channel has changed! Publishing change %i with %i \n",channel,iVal);

	sprintf(channelNameStr,"%i",channel);
	sprintf(valueStr,"%i",iVal);

	return MQTT_PublishMain(mqtt_client,channelNameStr,valueStr, 0, true);
}
OBK_Publish_Result MQTT_ChannelPublish(int channel, int flags)
{
	char channelNameStr[8];
	char valueStr[16];
	int iValue;

	iValue = CHANNEL_Get(channel);

	addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "Forced channel publish! Publishing val %i with %i \n",channel,iValue);

	sprintf(channelNameStr,"%i",channel);
	sprintf(valueStr,"%i",iValue);

	return MQTT_PublishMain(mqtt_client,channelNameStr,valueStr, flags, true);
}
OBK_Publish_Result MQTT_PublishCommand(const void *context, const char *cmd, const char *args, int cmdFlags) {
	const char *topic, *value;
	OBK_Publish_Result ret;

	Tokenizer_TokenizeString(args);

	if(Tokenizer_GetArgsCount() < 2){
		addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"Publish command requires two arguments (topic and value)");
		return 0;
	}
	topic = Tokenizer_GetArg(0);
	value = Tokenizer_GetArg(1);

	ret = MQTT_PublishMain_StringString(topic,value, 0);

	return ret;
}
// initialise things MQTT
// called from user_main
void MQTT_init(){
	char cbtopicbase[64];
	char cbtopicsub[64];
  const char *baseName;
  uint8_t mac[6];
    	  char ble_name[20];
    	  char *mqtt_topic;

    	  mqtt_topic=CFG_mqtttopic();

    		wifi_get_mac_address((char *)mac, 1);
    		snprintf(ble_name, sizeof(ble_name), "%02x%02x", mac[4], mac[5]);
	baseName = CFG_GetShortDeviceName();
	sprintf(cbtopicbase,"%s/",baseName);

	addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"cbtopicbase %s \r\n", cbtopicbase);
  sprintf(cbtopicsub,"%s/config/%s/%s",baseName,mqtt_topic,ble_name);
  addLogAdv(LOG_INFO,LOG_FEATURE_MQTT,"cbtopicsub %s \r\n", cbtopicsub);
//  // register the main set channel callback
//	sprintf(cbtopicbase,"%s/",baseName);
//  sprintf(cbtopicsub,"%s/+/set",baseName);
  // note: this may REPLACE an existing entry with the same ID.  ID 1 !!!
  MQTT_RegisterCallback( cbtopicbase, cbtopicsub, 1, channelSet);

  // register the TAS cmnd callback
	sprintf(cbtopicbase,"cmnd/%s/",baseName);
  sprintf(cbtopicsub,"cmnd/%s/+",baseName);
  // note: this may REPLACE an existing entry with the same ID.  ID 2 !!!
  MQTT_RegisterCallback( cbtopicbase, cbtopicsub, 2, tasCmnd);

  mqtt_initialised = 1;

//	CMD_RegisterCommand("publish","",MQTT_PublishCommand, "Sqqq", NULL);

}

OBK_Publish_Result MQTT_DoItemPublishString(const char *sChannel, const char *valueStr) {
  return MQTT_PublishMain(mqtt_client, sChannel, valueStr, OBK_PUBLISH_FLAG_MUTEX_SILENT, false);
}
OBK_Publish_Result MQTT_DoItemPublish(int idx) {
  char dataStr[3*6+1];  //This is sufficient to hold mac value

	switch(idx) {
    case PUBLISHITEM_SELF_STATIC_RESERVED_2:
    case PUBLISHITEM_SELF_STATIC_RESERVED_1:
      return OBK_PUBLISH_WAS_NOT_REQUIRED;
	case PUBLISHITEM_SELF_DYNAMIC_LIGHTSTATE:
		if(LED_IsRunningDriver()) {
			return LED_SendEnableAllState();
		}
		return OBK_PUBLISH_WAS_NOT_REQUIRED; // didnt publish
    case PUBLISHITEM_SELF_DYNAMIC_LIGHTMODE:
		if(LED_IsRunningDriver()) {
			return LED_SendCurrentLightMode();
		}
		return OBK_PUBLISH_WAS_NOT_REQUIRED; // didnt publish
    case PUBLISHITEM_SELF_DYNAMIC_DIMMER:
		if(LED_IsRunningDriver()) {
			return LED_SendDimmerChange();
		}
		return OBK_PUBLISH_WAS_NOT_REQUIRED; // didnt publish
    case PUBLISHITEM_SELF_HOSTNAME:
      return MQTT_DoItemPublishString("host", CFG_GetShortDeviceName());

    case PUBLISHITEM_SELF_BUILD:
      return MQTT_DoItemPublishString("build", g_build_str);
      
    case PUBLISHITEM_SELF_MAC:
      return MQTT_DoItemPublishString("mac", HAL_GetMACStr(dataStr));
    
    case PUBLISHITEM_SELF_DATETIME:
      //Drivers are only built on BK7231 chips
      #ifndef OBK_DISABLE_ALL_DRIVERS
        if (DRV_IsRunning("NTP")) {
          sprintf(dataStr,"%d",NTP_GetCurrentTime());
          return MQTT_DoItemPublishString("datetime", dataStr);
        }
        else{
          return OBK_PUBLISH_WAS_NOT_REQUIRED;
        }
      #else
        return OBK_PUBLISH_WAS_NOT_REQUIRED;
      #endif

    case PUBLISHITEM_SELF_SOCKETS:
      sprintf(dataStr,"%d",LWIP_GetActiveSockets());
      return MQTT_DoItemPublishString("sockets", dataStr);

    case PUBLISHITEM_SELF_RSSI:
      sprintf(dataStr,"%d",HAL_GetWifiStrength());
      return MQTT_DoItemPublishString("rssi", dataStr);

    case PUBLISHITEM_SELF_UPTIME:
      sprintf(dataStr,"%d",Time_getUpTimeSeconds());
      return MQTT_DoItemPublishString("uptime", dataStr);

    case PUBLISHITEM_SELF_FREEHEAP:
      sprintf(dataStr,"%d",xPortGetFreeHeapSize());
      return MQTT_DoItemPublishString("freeheap", dataStr);

    case PUBLISHITEM_SELF_IP:
      g_firstFullBroadcast = false; //We published the last status item, disable full broadcast
      return MQTT_DoItemPublishString("ip", HAL_GetMyIPString());

    default:
      break;
  }
  
	if(CHANNEL_IsInUse(idx)) {
		// MQTT_ChannelPublish(g_publishItemIndex, OBK_PUBLISH_FLAG_MUTEX_SILENT);
	}
	return OBK_PUBLISH_WAS_NOT_REQUIRED; // didnt publish
}
static int g_secondsBeforeNextFullBroadcast = 30;

// called from user timer.
int MQTT_RunEverySecondUpdate() {
	addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "came to run mqtt every second update\n");
	if (!mqtt_initialised)
		return 0;

	// take mutex for connect and disconnect operations
	if(MQTT_Mutex_Take(100)==0) {
		addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "Mutex connect and disconnect operations\n");
		return 0;
	}

	// reconnect if went into MQTT library ERR_MEM forever loop
	if(g_memoryErrorsThisSession >= 5) {
		addLogAdv(LOG_INFO,LOG_FEATURE_MQTT, "MQTT will reconnect soon to fix ERR_MEM errors\n");
		g_memoryErrorsThisSession = 0;
		mqtt_reconnect = 5;
	}
	// if asked to reconnect (e.g. change of topic(s))
	if (mqtt_reconnect > 0){
		mqtt_reconnect --;
		addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "in mqtt reconnect=%d\n",mqtt_reconnect);
		if (mqtt_reconnect == 0){
			// then if connected, disconnect, and then it will reconnect automatically in 2s
			if (mqtt_client && mqtt_client_is_connected(mqtt_client)) {

				addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "mqtt disconnected\n");
				MQTT_disconnect(mqtt_client);
				loopsWithDisconnected = 8;
			}
		}
		addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "Timer discovers disconnected mqtt inside%i\n",loopsWithDisconnected);
		addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "in mqtt reconnect2=%d\n",mqtt_reconnect);
	}

	if(mqtt_client == 0 || mqtt_client_is_connected(mqtt_client) == 0) {
		addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "Timer discovers disconnected mqtt %i\n",loopsWithDisconnected);
		loopsWithDisconnected++;
		if(loopsWithDisconnected > 10)
		{
			if(mqtt_client == 0)
			{
				mqtt_client = mqtt_client_new();
			}
			addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "came here to connect again \n");
			MQTT_do_connect(mqtt_client);
			loopsWithDisconnected = 0;
		}
		MQTT_Mutex_Free();
		return 0;
	}
	else {
		MQTT_Mutex_Free();
		// below mutex is not required any more
		addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "Mutex is not required anymore\n");
		addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "g_timeSinceLastMQTTPublish=%d\n",g_timeSinceLastMQTTPublish);
		// it is connected
		g_timeSinceLastMQTTPublish++;
		addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "g_bPublishAllStatesNow=%d\n",g_bPublishAllStatesNow);
		addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "g_publishItemIndex=%d\n",g_publishItemIndex);
						addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "CHANNEL_MAX=%d\n",CHANNEL_MAX);



		// do we want to broadcast full state?
		// Do it slowly in order not to overload the buffers
		// The item indexes start at negative values for special items
		// and then covers Channel indexes up to CHANNEL_MAX
		if(g_bPublishAllStatesNow) {
			// Doing step by a step a full publish state
			if(g_timeSinceLastMQTTPublish > 2) {
				OBK_Publish_Result publishRes;
				int g_sent_thisFrame = 0;

				while(g_publishItemIndex < CHANNEL_MAX) {
					publishRes = MQTT_DoItemPublish(g_publishItemIndex);
					if(publishRes != OBK_PUBLISH_WAS_NOT_REQUIRED){
						addLogAdv(LOG_INFO,LOG_FEATURE_MQTT, "[g_bPublishAllStatesNow] item %i result %i\n",g_publishItemIndex,publishRes);
					}
					// There are several things that can happen now
					// OBK_PUBLISH_OK - it was required and was published
					if(publishRes == OBK_PUBLISH_OK) {
						g_sent_thisFrame++;
						if(g_sent_thisFrame>=1){
							g_publishItemIndex++;
							break;
						}
					}
					// OBK_PUBLISH_MUTEX_FAIL - MQTT is busy
					if(publishRes == OBK_PUBLISH_MUTEX_FAIL
						|| publishRes == OBK_PUBLISH_WAS_DISCONNECTED) {
						// retry the same later
						addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "Mutex is busy\n");
						break;
					}
					// OBK_PUBLISH_WAS_NOT_REQUIRED
					// The item is not used for this device
					g_publishItemIndex++;
				}
				if(g_publishItemIndex >= CHANNEL_MAX) {
					// done
					addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "CAME TO ZERO\n");
					g_bPublishAllStatesNow = 0;
				}
			}
		}
		else {
		// not doing anything
		addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "DNT DO ANYTHING\n");
			if(CFG_HasFlag(OBK_FLAG_MQTT_BROADCASTSELFSTATEPERMINUTE)) {
				addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "g_secondsBeforeNextFullBroadcast=%d\n",g_secondsBeforeNextFullBroadcast);
			// this is called every second
				g_secondsBeforeNextFullBroadcast--;
				if(g_secondsBeforeNextFullBroadcast <= 0) {
					g_secondsBeforeNextFullBroadcast = 60;
					addLogAdv(LOG_INFO,LOG_FEATURE_MAIN, "MQTT_PublishWholeDeviceState\n");
					MQTT_PublishWholeDeviceState();
				}
		}
		}

	}
	return 1;
}

