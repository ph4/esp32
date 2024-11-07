#include "my_blynk_esp32.h"

//#include <BlynkApiArduino.h>

SemaphoreHandle_t blynk_mutex;

static WiFiClient _blynkWifiClient;
static BlynkArduinoClient _blynkTransport(_blynkWifiClient);

BlynkWifi Blynk(_blynkTransport);
