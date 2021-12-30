#ifndef my_blynk_h
#define my_blynk_h
#include "my_config.h"
#include <freertos/semphr.h>

//#define INTELLISENSE_DUMB
#ifdef INTELLISENSE_DUMB
#include <freertos/FreeRTOSConfig.h>
// Dumbass intellisense (from freertos/smphr.h)
#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	#define xSemaphoreCreateMutex() xQueueCreateMutex( queueQUEUE_TYPE_MUTEX )
#endif 
#endif /* ifdef INTELLISENSE_DUMB */

//#define BLYNK_TEMPLATE_ID           "TMPLAlOVHdPg"
#define BLYNK_DEVICE_NAME           "Test Device"
//#define BLYNK_AUTH_TOKEN            "1ErEYq9y5iXaEHdohl6dmr2ViOdMq4r9"
#define BLYNK_AUTH_TOKEN            "e5d7ccd58b3f4b179e7df6bd866715a5"

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial


#include <my_blynk_esp32.h>

extern BlynkWifi Blynk;
extern SemaphoreHandle_t blynk_mutex;
extern void blynk_init();

#include <BlynkWidgets.h>


#endif