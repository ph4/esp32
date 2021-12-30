#ifndef my_bot_h
#define my_bot_h

#include <UniversalTelegramBot.h>
#include <WiFiClientSecure.h>


extern WiFiClientSecure bot_secured_client;

extern UniversalTelegramBot bot;

extern bool (*bot_callback) (UniversalTelegramBot &bot, telegramMessage &msg);

extern void bot_task(void *pvParams);
//extern void bot_init();
extern void bot_handle_message(int numMessages);
#endif