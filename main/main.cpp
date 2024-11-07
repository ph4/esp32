#include <stdio.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_sntp.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <nvs.h>

#include <Arduino.h>
//#include <analogWrite.h>
#include <WiFi.h>

#include <lwip/inet.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 4
#endif

#include "my_config.h"
#include "my_blynk.h"
#include "my_dht.h"
#include "bot.h"
#include "freertos_polyfil.h"
#include "logging.h"

#define BLYNK_INPUT_FLOAT(pin, name) float name; BLYNK_INPUT(pin) { name = getValue.asFloat(); }
#define BLYNK_INPUT_FLOAT_CLAMPED(pin, name, min, max) float name; BLYNK_INPUT(pin) { name = BlynkMathClamp(getValue.asFloat(), min, max); }

#define BLYNK_INPUT_INT(pin, name) int name; BLYNK_INPUT(pin) { name = getValue.asInt(); }
#define BLYNK_INPUT_INT_CLAMPED(pin, name, min, max) int name; BLYNK_INPUT(pin) { name = BlynkMathClamp(getValue.asInt(), min, max); }

//#include <BlynkWidgets.h>

volatile bool running = true;

void blynk_init() {
  
}

WidgetTerminal terminal(V20);

#define TASK_BUFFER_SIZE 40
#define TASK_BUFFER_COUNT 25

char task_buffer[TASK_BUFFER_COUNT * TASK_BUFFER_SIZE];

void str_replace_char(char *str, char a, char b) {
  while (*str) {
    if (*str == a) *str = b;
    str++;
  }
}

volatile bool send_task_stats_r = false;
void send_task_list(void *) {
  vTaskList(task_buffer);
  str_replace_char(task_buffer, '\t', ' ');
  if (pdTRUE == xSemaphoreTake(blynk_mutex, MAX_SEMAPHORE_WAIT)) {
    terminal.println("Task list --------------------");
    terminal.println("Name           S P hw_stack N CPU");
    terminal.write(task_buffer);
    terminal.println("------------------------------");
    terminal.flush();
    xSemaphoreGive(blynk_mutex);
  } else LOG_MUTEX_FAIL("task_list", blynk_mutex);

  send_task_stats_r = false;
  vTaskDelete(NULL);
}

void send_runtime_stats(void *) {
  vTaskGetRunTimeStats(task_buffer);
  str_replace_char(task_buffer, '\t', ' ');
  if (pdTRUE == xSemaphoreTake(blynk_mutex, MAX_SEMAPHORE_WAIT)) {
    terminal.println("Runtime stats ----------------");
    terminal.println("Name         runtime (%)");
    terminal.write(task_buffer);
    terminal.println("------------------------------");
    terminal.flush();
    xSemaphoreGive(blynk_mutex);
  } else LOG_MUTEX_FAIL("send_rt_stats", blynk_mutex);
  
  send_task_stats_r = false;
  vTaskDelete(NULL);
}

// get taskList
BLYNK_INPUT(V21) {
  if (getValue.asInt()) {
    if (!send_task_stats_r) {
      send_task_stats_r = true;
      xTaskCreate(&send_task_list, "send_task_list", 4096, NULL, 3, NULL);
    }
  }
}

// get runtime stats
BLYNK_INPUT(V22) {
  if (getValue.asInt()) {
    if (!send_task_stats_r) {
      send_task_stats_r = true;
      xTaskCreate(&send_runtime_stats, "send_rt_stats", 4096, NULL, 3, NULL);
    }
  }
}

// BlynkTimer timer;
BLYNK_CONNECTED() { Blynk.syncAll(); }


BLYNK_INPUT_INT_CLAMPED(V1, bell_length_ms, 0, 5000)
BLYNK_INPUT_FLOAT_CLAMPED(V2, bell_power, 0.0f, 1.0f)

BLYNK_INPUT_FLOAT_CLAMPED(V10, bell_exp_base, 0.0f, 20000.0f)

BLYNK_INPUT_FLOAT_CLAMPED(V11, bell_min_power, 0.0f, 1.0f)

BLYNK_INPUT_INT_CLAMPED(V12, bell_kickstart_ms, 0, 50)
BLYNK_INPUT_FLOAT_CLAMPED(V13, bell_kickstart_power, 0.0f, 1.0f)
  
//bell_min_power = 0.025f;

float finvlerp(float xx, float yy, float value)
{
    return (value - xx)/(yy - xx);
}

float flerp(float x0, float x_max, float f) {
  return f * (x_max - x0) + x0;
}

uint32_t bell_power_to_duty(float power) {
  LOGD("power = %f", power);
  configASSERT(bell_min_power >= 0);
  uint32_t duty = flerp((float)LEDC_MAX * bell_min_power, LEDC_MAX, power);
  LOGD("duty = %u", duty);
  if (duty > LEDC_MAX) duty = LEDC_MAX;
  LOGD("duty fixed = %u", duty);
  configASSERT(duty <= LEDC_MAX);
  return duty;
}

typedef struct {
 uint32_t ks_duty;
 TickType_t ks_ticks;
 uint32_t duty;
 TickType_t ticks;
} bell_config_t;

bell_config_t bell_config;

void bell_task(void * parent) {
  uint32_t nv = 0U;
  BaseType_t interrupted = false;

  while (running) {
    if (nv == 0U) {
      xTaskNotifyWait(0U, ULONG_MAX, &nv, portMAX_DELAY);
    }
    bell_config_t config = bell_config;
    
    
    gpio_set_level(BLINK_GPIO, HIGH);
    if (config.ks_ticks > 0) {
      // analogWrite(BELL_GPIO, config.ks_duty, LEDC_MAX);
      vTaskDelay(config.ks_ticks);
    }

    if (config.ticks > 0) {
      // analogWrite(BELL_GPIO, config.duty, LEDC_MAX);

      interrupted = xTaskNotifyWait(0U, ULONG_MAX, &nv, config.ticks);
    }
    // analogWrite(BELL_GPIO, 0U);

    gpio_set_level(BLINK_GPIO, LOW);

    if (!interrupted) {
      nv = 0U;
    } else {
      interrupted = false;
      ESP_LOGI("bell", "bell_interrupted");
    }
  }

  vTaskDelete(NULL);
}


BLYNK_INPUT(V0) {
  static TaskHandle_t bell_task_handle = NULL;
  if (bell_task_handle == NULL) {
    LOGI("creating bell_task");
    xTaskCreate(&bell_task, "bell_task", 4096, xTaskGetCurrentTaskHandle(), 10,
                &bell_task_handle);
  }
  if (getValue.asInt()) {
    float val;
    if (bell_exp_base > 1)
      val = (powf(bell_exp_base, bell_power) - 1.0f) / (bell_exp_base - 1.0f);
    else {
      val = bell_power;
    }

    bell_config.duty = bell_power_to_duty(bell_power);
    bell_config.ticks = pdMS_TO_TICKS(max(bell_length_ms - bell_kickstart_ms, 0));

    bell_config.ks_ticks = pdMS_TO_TICKS(bell_kickstart_ms);
    bell_config.ks_duty = bell_power_to_duty(bell_kickstart_power);

    LOGI("notifying bell_task: len=%d, duty=%u, ks_len=%d, ks_duty=%u", bell_length_ms, bell_config.duty, bell_kickstart_ms, bell_config.ks_duty);
    xTaskNotify(bell_task_handle, 1U, eSetValueWithOverwrite);
  }
}


int vcc_mv = 0;

// extern "C" int rom_phy_get_vdd33();
void send_voltages(void *pvParameter) {
  //const char *TAG = "voltages";

  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(VBAT_ADC1_CHANNEL, ADC_ATTEN_11db);
  adc1_config_channel_atten(VCC_ADC1_CHANNEL, ADC_ATTEN_11db);

  // Calculate ADC characteristics i.e. gain and offset factors
  esp_adc_cal_characteristics_t characteristics;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100,
                           &characteristics);

  int vbat_cnt = 0;
  int vbat_acc = 0;
  int min[2] = {INT_MAX, INT_MAX};
  int max[2] = {INT_MIN, INT_MIN};

  TickType_t lastWake = xTaskGetTickCount();

  while (running) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
    int vcc = 0;
    for (int i = 0; i < VCC_SAMPLES; i++) {
      vcc += adc1_get_raw(VCC_ADC1_CHANNEL);
    }
    vcc = esp_adc_cal_raw_to_voltage(vcc / VCC_SAMPLES, &characteristics) * 2;
    vcc_mv = vcc;

    if (xSemaphoreTake(blynk_mutex, MAX_SEMAPHORE_WAIT) == pdTRUE) {
      Blynk.virtualWrite(V33, vcc);
      xSemaphoreGive(blynk_mutex);
    } else
      LOG_MUTEX_FAIL("VCC", blynk_mutex);

    int vbat = adc1_get_raw(VBAT_ADC1_CHANNEL);
    vbat = esp_adc_cal_raw_to_voltage(vbat, &characteristics);
    vbat_acc += vbat;

    // min[1] < min[0] < USEFUL_READINGS < max[0] < max[1]
    if (vbat <= min[1]) {
      min[1] = min[0];
      min[0] = vbat;
    }
    if (vbat >= max[1]) {
      max[1] = max[0];
      max[0] = vbat;
    }
    vbat_cnt++;

    if (vbat_cnt >= VBAT_INTERVAL) {
      // Remove 2 lowest and 2 highest readings
      vbat_acc -= (min[0] + min[1]);
      vbat_acc -= (max[0] + max[1]);
      vbat = (vbat_acc / (vbat_cnt - 4)) * 2;

      if (xSemaphoreTake(blynk_mutex, MAX_SEMAPHORE_WAIT) == pdTRUE) {
        Blynk.virtualWrite(V34, vbat);
        xSemaphoreGive(blynk_mutex);
      } else
        LOG_MUTEX_FAIL("VBAT", blynk_mutex)

      // Reset counters
      vbat_cnt = 0;
      vbat_acc = 0;
      max[0] = max[1] = INT_MIN;
      min[0] = min[1] = INT_MAX;
    }

  }

  vTaskDelete(NULL);
}

void send_telemetry(void *pvParameter) {
  TickType_t lastWake = xTaskGetTickCount();
  while (running) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5000));
    int time = millis() / 1000;
    if (xSemaphoreTake(blynk_mutex, MAX_SEMAPHORE_WAIT) == pdTRUE) {
      Blynk.virtualWrite(V30, time);
      xSemaphoreGive(blynk_mutex);
    } else
      LOG_MUTEX_FAIL("telemetry", blynk_mutex)
  }

  vTaskDelete(NULL);
}


void blynk_task(void *pvParam) {
  LOGI("Blynk init");
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS, BLYNK_DOMAIN, 8080);

  TickType_t lastWake = xTaskGetTickCount();
  while (running) {
    vTaskDelayUntil(&lastWake, 333 / portTICK_PERIOD_MS);
    LOGD("Blynk Run start {");

    xSemaphoreTake(blynk_mutex, portMAX_DELAY);

    LOGD("  Blynk Run >>");
    Blynk.run();
    LOGD("  Blynk Run <<");

    xSemaphoreGive(blynk_mutex);

    LOGD("} Blynk Run end");
    //
    // timer.run();
  }

  vTaskDelete(NULL);
}

void after_connect() {
  static bool first_run = true;
  if (!first_run) return;
  first_run = false;

  static const char *TAG = "after_wifi";
  ESP_LOGI(TAG, "Starting SNTP");
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();

  // time_t rawtime = time(NULL);
  // struct tm * timeinfo;
  // timeinfo = localtime (&rawtime);
  // ESP_LOGI(TAG, "Sntp started, local time: %s", asctime(timeinfo));

#define K *1024
  // clang-format off
  ESP_LOGI(TAG,                "Starting telemetry_task");
  xTaskCreatePinnedToCore(&send_telemetry, "telemetry_task",  6 K, NULL, 6, NULL, 0);

  ESP_LOGI(TAG,                "Starting voltages_task");
  xTaskCreatePinnedToCore(&send_voltages,  "voltages_task",   6 K, NULL, 6, NULL, 0);

  ESP_LOGI(TAG,                "Starting dht_task");
  xTaskCreate(&dht_task,       "dht_task",        6 K, NULL, 3, NULL);

  ESP_LOGI(TAG,                "Starting bot_task");
  xTaskCreate(&bot_task,       "bot_task",        10 K, NULL, 5, NULL);

  ESP_LOGI(TAG,                "Starting blynk_task");
  xTaskCreate(&blynk_task,     "blynk_task",      6 K, NULL, 6, NULL);

// clang-format on
#undef K
}

extern void wifi_sta_event_cb(WiFiEvent_t id, WiFiEventInfo_t info);

#define WIFI_MAX_RETRY 5
volatile int wifi_retry = 0;
//volatile bool wifi_connected = false;

SemaphoreHandle_t wifi_connected_sem;
SemaphoreHandle_t wifi_disconnected_sem;

void wifi_task(void *) {
    LOGI("Starting WiFi on: %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
    IPAddress ip(192, 168, 1, 37);
    IPAddress gw(192, 168, 1, 1);
    IPAddress mask(255, 255, 255, 0);
    IPAddress dns1(85,21,192,5);
    IPAddress dns2(213,234,192,7);
    //IPAddress dns1(8,8,8,8);
    //IPAddress dns2(8,8,4,4);
    //WiFi.config(ip, gw, mask, dns1, dns2);
    WiFi.onEvent(&wifi_sta_event_cb, ARDUINO_EVENT_WIFI_STA_CONNECTED);
    WiFi.onEvent(&wifi_sta_event_cb, ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.onEvent(&wifi_sta_event_cb, ARDUINO_EVENT_WIFI_STA_LOST_IP);
    WiFi.onEvent(&wifi_sta_event_cb, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    //while (WiFi.status() != WL_CONNECTED) {
    //  vTaskDelay(pdMS_TO_TICKS(100));  // TODO: add timeout and restart
    //}

    while(wifi_retry <= WIFI_MAX_RETRY) {
      // Waiting for wifi disconnect event
      xSemaphoreTake(wifi_disconnected_sem, portMAX_DELAY);
      wifi_retry += 1;
      LOGW("Trying to reconnect to wifi: %s (%d/%d)", WIFI_SSID, wifi_retry,
           WIFI_MAX_RETRY);
      WiFi.begin();
    }

    LOGE("Coud not connect to in %d tries, restarting", WIFI_MAX_RETRY);
    esp_restart();
}

void wifi_sta_event_cb(WiFiEvent_t id, WiFiEventInfo_t info) {
  const char * TAG = "wifi_event";
  switch (id) {
    case SYSTEM_EVENT_STA_CONNECTED:
      ESP_LOGI(TAG, "WiFi Connected: %s", WiFi.SSID().c_str());
      wifi_retry = 0;
      xSemaphoreGive(wifi_connected_sem);
      break;
    case SYSTEM_EVENT_STA_GOT_IP: {

      esp_netif_ip_info_t ip = info.got_ip.ip_info;

      char ipaddr[INET_ADDRSTRLEN];
      inet_ntoa_r(ip.ip, ipaddr, INET_ADDRSTRLEN);

      uint32_t mask_addr = ip.netmask.addr;
      int mask = 0;
      while (mask_addr) {
        mask_addr >>= 1;
        mask++;
      }

      ESP_LOGI(TAG, "WiFi Got IP: %s/%d via %s", ipaddr, mask, inet_ntoa(ip.gw));
      after_connect();
      break;
    }
    // case SYSTEM_EVENT_STA_LOST_IP:
    //   ESP_LOGE(TAG, "WiFi Lost IP");
    //   //falltrough
    //   [[fallthrough]]
    case SYSTEM_EVENT_STA_DISCONNECTED:
      ESP_LOGW(TAG, "WiFi Disconnected");
      //xSemaphoreTake(wifi_connected_sem, portMAX_DELAY);
      xSemaphoreGive(wifi_disconnected_sem);
      break;
    default:
      break;
  }
}

void setup() {
  //static const char *TAG = "setup";
  Serial.begin(115200);

  /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
  gpio_pad_select_gpio(BLINK_GPIO);
  /* Set the GPIO as a push/pull output */
  ESP_ERROR_CHECK(gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(gpio_set_level(BLINK_GPIO, 0));

  // gpio_pad_select_gpio(BELL_GPIO);
  // gpio_set_direction(BELL_GPIO, GPIO_MODE_OUTPUT);
  // gpio_set_level(BELL_GPIO, 0);

  // You can also specify server:
  // Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  // Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  // Setup a function to be called every second
  // timer.setInterval(1000L, myTimerEvent);
  // configTime(0, 0, "pool.ntp.org");
  //
  if (blynk_mutex == NULL)
    blynk_mutex = xSemaphoreCreateMutex();
  wifi_connected_sem = xSemaphoreCreateBinary();
  wifi_disconnected_sem = xSemaphoreCreateBinary();
  LOGI("Starting wifi_task");
  xTaskCreate(&wifi_task,     "wifi_task",      4096, NULL, 8, NULL);
}

TickType_t loop_lastWake = xTaskGetTickCount();

void loop(void * param) {
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  //vTaskDelayUntil(&loop_lastWake, pdMS_TO_TICKS(10000));
  vTaskSuspend( NULL );
}

#if !CONFIG_AUTOSTART_ARDUINO
extern "C" void app_main() {
  // Initialize NVS wifi needs it
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );
  setup();
  xTaskCreate(&loop, "loop", configMINIMAL_STACK_SIZE, NULL, 5,
              NULL);
}
#endif
