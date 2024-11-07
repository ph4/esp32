#include "bot.h"

#include <esp_log.h>
#include <freertos_polyfil.h>

#include "my_blynk.h"
#include "my_config.h"
#include "logging.h"

unsigned long bot_lasttime;  // last time messages' scan has been done
bool Start = false;

WiFiClientSecure bot_secured_client;
UniversalTelegramBot bot(BOT_TOKEN, bot_secured_client);
bool (*bot_callback)(UniversalTelegramBot& bot, telegramMessage& msg);

void bot_init() {
  // bot_secured_client = WiFiClientSecure();
  bot_secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  // bot = UniversalTelegramBot;
  bot.longPoll = 20;
}

void bot_task(void * pvParams) {
  bot_init();
  TickType_t nextWake = xTaskGetTickCount();
  const TickType_t interval = pdMS_TO_TICKS(bot.longPoll * 1000 / 3);
  while (1) {
    LOGD("Polling server");
    // Should throttle bot if connection is broken
    vTaskDelayUntil(&nextWake, interval);
    //LOGI("Heap pre getUpdates: %u", xPortGetFreeHeapSize());
    int bot_num_updates = bot.getUpdates(bot.last_message_received + 1);
    //LOGI("Heap post getUpdates: %u", xPortGetFreeHeapSize());
    while (bot_num_updates) {
      //LOGI("Heap pre handle_message: %u", xPortGetFreeHeapSize());
      bot_handle_message(bot_num_updates);
      //LOGI("Heap post handle_message: %u", xPortGetFreeHeapSize());
      bot_num_updates = bot.getUpdates(bot.last_message_received + 1);
      //LOGI("Heap post handle_message + getUpdates: %u", xPortGetFreeHeapSize());
    }
  }
  vTaskDelete(NULL);
}

void bot_handle_message(int numMessages) {
  static const char * TAG = "bot_handle";
  ESP_LOGI(TAG, "numMessages = '%d'", numMessages);
  for (int i = 0; i < numMessages; i++) {
    auto msg = &bot.messages[i];
    ESP_LOGI(TAG, "msg->type = '%s'", msg->type.c_str());
    if (bot_callback != NULL && bot_callback(bot, bot.messages[i])) {
      continue;
    }

    String& chat_id = bot.messages[i].chat_id;
    String& text = bot.messages[i].text;

    String& from_name = bot.messages[i].from_name;

    if (from_name == "") from_name = "Guest";

    if (text == "/send_test_action") {
      bot.sendChatAction(chat_id, "typing");
      delay(4000);
      bot.sendMessage(chat_id, "Did you see the action message?");

      // You can't use own message, just choose from one of bellow

      // typing for text messages
      // upload_photo for photos
      // record_video or upload_video for videos
      // record_audio or upload_audio for audio files
      // upload_document for general files
      // find_location for location data

      // more info here - https://core.telegram.org/bots/api#sendchataction
    }

    if (text == "/admin") {
      if (chat_id == OWNER_ID) {
        bot.sendMessage(chat_id, "You are an admin");
      } else {
        bot.sendMessage(chat_id, "You are NOT an admin");
      }
    }

    if (text == "/start") {
      String welcome = "Welcome to Universal Arduino Telegram Bot library, " +
                       from_name + ".\n";
      welcome += "This is Chat Action Bot example.\n\n";
      welcome += "/send_test_action : to send test chat action message\n";
      bot.sendMessage(chat_id, welcome);
    }
  }
}
