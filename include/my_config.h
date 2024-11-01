#ifndef my_config_h
#define my_config_h

#include <driver/gpio.h>
#include <driver/adc.h>
#include <math.h>
#include <freertos/FreeRTOS.h>

//#define DHT_DEBUG
#include <DHT.h>
#include "secrets.h"

// Voltage stats
#define VBAT_ADC1_CHANNEL ADC1_GPIO35_CHANNEL
#define VCC_ADC1_CHANNEL ADC1_GPIO34_CHANNEL
#define VBAT_INTERVAL 30
#define VCC_SAMPLES 32

#define LEDC_RESOLUTION 13U
const uint32_t LEDC_MAX = pow(2U, LEDC_RESOLUTION) - 1;

// Pins
#define BLINK_GPIO (gpio_num_t) CONFIG_BLINK_GPIO
#define BELL_GPIO GPIO_NUM_13

// Dht
#define DHT_GPIO GPIO_NUM_15
#define DHT_TYPE DHT22


const int MAX_SEMAPHORE_WAIT_MS = 10000;
const TickType_t MAX_SEMAPHORE_WAIT =
    MAX_SEMAPHORE_WAIT_MS / portTICK_PERIOD_MS;

#endif