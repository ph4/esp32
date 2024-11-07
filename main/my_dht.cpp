#include "my_dht.h"

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <esp_log.h>

#include "my_config.h"
#include "my_blynk.h"

void dht_init() {}

void dht_task(void *pvParameter) {
  const char *TAG = "DHT sensor";
  DHT_Unified dht(DHT_GPIO, DHT_TYPE);
  dht.begin();
  ESP_LOGI(TAG, "DHTxx Unified Sensor Example");
  // Print temperature sensor details.
  sensor_t sensor_tmp;
  dht.temperature().getSensor(&sensor_tmp);
  ESP_LOGI(TAG, "------------------------------------");
  ESP_LOGI(TAG, "Temperature Sensor");
  ESP_LOGI(TAG, "Sensor Type: %s", sensor_tmp.name);
  ESP_LOGI(TAG, "Driver Ver:  %d", sensor_tmp.version);
  ESP_LOGI(TAG, "Unique ID:   %d", sensor_tmp.sensor_id);
  ESP_LOGI(TAG, "Max Value:   %.2f°C", sensor_tmp.max_value);
  ESP_LOGI(TAG, "Min Value:   %.2f°C", sensor_tmp.min_value);
  ESP_LOGI(TAG, "Resolution:  %.2f°C", sensor_tmp.resolution);
  ESP_LOGI(TAG, "------------------------------------");

  sensor_t sensor_rh;
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor_rh);
  ESP_LOGI(TAG, "Humidity Sensor");
  ESP_LOGI(TAG, "Sensor Type: %s", sensor_rh.name);
  ESP_LOGI(TAG, "Driver Ver:  %d", sensor_rh.version);
  ESP_LOGI(TAG, "Unique ID:   %d", sensor_rh.sensor_id);
  ESP_LOGI(TAG, "Max Value:   %.2f%%", sensor_rh.max_value);
  ESP_LOGI(TAG, "Min Value:   %.2f%%", sensor_rh.min_value);
  ESP_LOGI(TAG, "Resolution:  %.2f%%", sensor_rh.resolution);
  ESP_LOGI(TAG, "------------------------------------");

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t interval = sensor_tmp.min_delay / 1000 / portTICK_PERIOD_MS;
  ESP_LOGI(TAG, "Sensor min delay: %d ms", sensor_tmp.min_delay / 1000);

  float factor;
  if (pvParameter != NULL) {
    factor = max(1.0f, *(float *)pvParameter);
  } else {
    factor = 3;
  }
  vTaskDelayUntil(&lastWake, interval * factor);

  while (1) {
    sensors_event_t e1, e2;
    dht.temperature().getEvent(&e1);
    dht.humidity().getEvent(&e2);

    ESP_LOGD(TAG, "Start dht {");

    xSemaphoreTake(blynk_mutex, portMAX_DELAY);
    if (!isnanf(e1.temperature))
      Blynk.virtualWrite(V5, e1.temperature);
    else
      ESP_LOGD(TAG, "failed to read temperature");
    // Blynk.virtualWrite(V5, sensor_tmp.min_value);

    if (!isnanf(e2.relative_humidity))
      Blynk.virtualWrite(V6, e2.relative_humidity);
    else
      ESP_LOGD(TAG, "failed to read humidity");
    // Blynk.virtualWrite(V6, sensor_rh.min_value);
    xSemaphoreGive(blynk_mutex);

    ESP_LOGD(TAG, "}End dht");

    vTaskDelayUntil(&lastWake, interval);
  }
}
