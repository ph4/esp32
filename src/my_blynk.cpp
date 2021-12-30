#include "my_blynk.h"

SemaphoreHandle_t blynk_mutex;
static WiFiClient _blynkWifiClient;
static BlynkArduinoClient _blynkTransport(_blynkWifiClient);

BlynkWifi blynk(_blynkTransport);