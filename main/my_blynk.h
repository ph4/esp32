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

#ifndef BLYNK_DEVICE_NAME
	#error BLYNK_DEVICE_NAME is not set
#endif
#ifndef BLYNK_AUTH_TOKEN
	#error BLYNK_AUTH_TOKEN is not set
#endif

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial


#include <my_blynk_esp32.h>

extern BlynkWifi Blynk;
extern SemaphoreHandle_t blynk_mutex;
extern void blynk_init();

#include <BlynkWidgets.h>


#endif