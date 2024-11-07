#ifndef my_blynk_esp32_h
#define my_blynk_esp32_h

#include <Blynk/BlynkProtocol.h>
#include <Adapters/BlynkArduinoClient.h>
#include <Blynk/BlynkApi.h>
#include <WiFi.h>

#ifndef ESP32
#error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#endif

#define BLYNK_SEND_ATOMIC


class BlynkWifi
    : public BlynkProtocol<BlynkArduinoClient>
{
    typedef BlynkProtocol<BlynkArduinoClient> Base;
public:
    BlynkWifi(BlynkArduinoClient& transp)
        : Base(transp)
    {}

    void connectWiFi(const char* ssid, const char* pass)
    {
        if(WiFi.isConnected() && WiFi.SSID() == ssid) {
            BLYNK_LOG2(BLYNK_F("Already connected to "), ssid);
        }
        else {
            BLYNK_LOG2(BLYNK_F("Connecting to "), ssid);
            WiFi.mode(WIFI_STA);

            if (pass && strlen(pass)) {
                WiFi.begin(ssid, pass);
            } else {
                WiFi.begin(ssid);
            }
            while (WiFi.status() != WL_CONNECTED) {
                BlynkDelay(200);
            }
            BLYNK_LOG1(BLYNK_F("Connected to WiFi"));
        }

        IPAddress myip = WiFi.localIP();
        (void)myip; // Eliminate warnings about unused myip
        BLYNK_LOG_IP("IP: ", myip);
    }

    void config(const char* auth,
                const char* domain = BLYNK_DEFAULT_DOMAIN,
                uint16_t    port   = BLYNK_DEFAULT_PORT)
    {
        Base::begin(auth);
        this->conn.begin(domain, port);
    }

    void config(const char* auth,
                IPAddress   ip,
                uint16_t    port = BLYNK_DEFAULT_PORT)
    {
        Base::begin(auth);
        this->conn.begin(ip, port);
    }

    void begin(const char* auth,
               const char* ssid,
               const char* pass,
               const char* domain = BLYNK_DEFAULT_DOMAIN,
               uint16_t    port   = BLYNK_DEFAULT_PORT)
    {
        connectWiFi(ssid, pass);
        config(auth, domain, port);
        while(this->connect() != true) {}
    }

    void begin(const char* auth,
               const char* ssid,
               const char* pass,
               IPAddress   ip,
               uint16_t    port   = BLYNK_DEFAULT_PORT)
    {
        connectWiFi(ssid, pass);
        config(auth, ip, port);
        while(this->connect() != true) {}
    }

};

#endif