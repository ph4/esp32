#ifndef _logging_h
#define _logging_h

#define LOGE(format, ...) ESP_LOGE(__func__, format, ##__VA_ARGS__)
#define LOGW(format, ...) ESP_LOGW(__func__, format, ##__VA_ARGS__)
#define LOGI(format, ...) ESP_LOGI(__func__, format, ##__VA_ARGS__)
#define LOGD(format, ...) ESP_LOGD(__func__, format, ##__VA_ARGS__)
#define LOGV(format, ...) ESP_LOGV(__func__, format, ##__VA_ARGS__)


#define LOG_MUTEX_FAIL(TAG, MUTEX)                                \
  do {                                                            \
    TaskHandle_t holder = xSemaphoreGetMutexHolder(MUTEX);        \
    TaskStatus_t status;                                          \
    vTaskGetInfo(holder, &status, pdFALSE, eRunning);             \
    ESP_LOGW(TAG, "Failed to take mutex in %d ms, owner: \"%s\"", \
             MAX_SEMAPHORE_WAIT_MS, status.pcTaskName);           \
  } while (0);

#define WARN_MUTEX_LONG(TAG, MUTEX)                            \
  do {                                                         \
    if (xSemaphoreTake(MUTEX, MAX_SEMAPHORE_WAIT) == pdFAIL) { \
      TaskHandle_t holder = xSemaphoreGetMutexHolder(MUTEX);   \
      TaskStatus_t status;                                     \
      vTaskGetInfo(holder, &status, pdFALSE, eRunning);        \
      ESP_LOGW(TAG, "Mutex take > %d ms, owner: \"%s\"",       \
               MAX_SEMAPHORE_WAIT_MS, status.pcTaskName);      \
      xSemaphoreTake(MUTEX, portMAX_DELAY);                    \
    }                                                          \
  } while (0);


#endif
