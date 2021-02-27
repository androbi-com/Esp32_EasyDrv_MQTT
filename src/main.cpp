/*

Copyright 2021 C. Denk(androbi)

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of 
   conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list 
   of conditions and the following disclaimer in the documentation and/or other materials 
   provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used 
   to endorse or promote products derived from this software without specific prior 
   written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <time.h>
#include "I2CHelper.h"
#define CLIENT_ADDR 0x48
#define CMD_GET_POS_1   0x00   
#define CMD_GET_POS_2   0x01   
#define CMD_GET_OMEGA_1 0x02 
#define CMD_GET_OMEGA_2 0x03
#define CMD_GET_ENC_ALL 0x04
#define CMD_GET_TIME    0x05
#define UNIX_OFFSET   946684800
#define MICROS 1000000
#define DEEP_SECS 10

#define CMD_DBG_OUTPUT 0x30 // returns 1 byte

// enable/disable command
#define CMD_PUT_ENABLED_STATE 0x10
#define DISABLE_ALL 0x00
#define ENABLE_ALL 0xFF

#define CMD_PUT_DRIVER_STATE 0x11                // set driver state
#define DRIVER_ENABLED_MASK 0x08 | (0x08 << 4);  // 1000 1000
#define DRIVER_SLOWFAST_MASK 0x04 | (0x04 << 4); // 0100 0100

#define CMD_PUT_DUTY_CYCLE_1 0x20  // set duty cycle motor 1
#define CMD_PUT_DUTY_CYCLE_2 0x21  // set duty cycle motor 2
#define CMD_PUT_DUTY_CYCLE_12 0x22 // set both duty cycles

#define CMD_PUT_PDI_PARS 0x40  // set pdi parameters Kp,Ki,Kd (3 float vals)
#define PUT_PDI_PARS_LENGTH 12 // message length
#define CMD_PUT_SET_MODE 0x41  // set setpoints and mode set_1, set_2, modePDI (2 uint16, 1 uint8)
#define PUT_SET_MODE_LENGTH 5  // message length
#define CMD_PUT_RESET_POS 0x42  // set setpoints and mode set_1, set_2, modePDI (2 uint16, 1 uint8)
#define PUT_RESET_POS_LENGTH 4  // message length

#define LEN_POS 4
#define LED_RED

static int32_t pos_offset[2]={0,0};
static int16_t old_pos[2]={-1,-1};

static unsigned long unix_offset;
static uint16_t one_sec;
static float one_sec_f;
#ifndef WIFI_SSID
    #error define WIFI_SSID, WIFI_PASSWORD and MQTT_BROKER in your environment
#else
    const char *ssid = WIFI_SSID;
    const char *password = WIFI_PASSWORD;
    const char *mqtt_server = MQTT_BROKER;
#endif
WiFiClient espClient;
PubSubClient client(espClient);
I2CHelper easyDrv(CLIENT_ADDR);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

typedef enum DriveStateEnum
{
    COAST = 0, // 00
    REVERSE,   // 01
    FORWARD,   // 10
    BREAK      // 11
} DriveMode;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void deepsleep() {
    esp_sleep_enable_timer_wakeup(DEEP_SECS * MICROS);
    esp_deep_sleep_start();
}

void led_off()
{
    digitalWrite(16, HIGH);
    digitalWrite(17, HIGH);
    digitalWrite(18, HIGH);
}
void led_on(bool r = false, bool g = false, bool b = false)
{
    digitalWrite(16, !r);
    digitalWrite(17, !g);
    digitalWrite(18, !b);
}

void send_3float(I2CHelper *easyDrv, uint8_t reg, float val1, float val2, float val3)
{
    easyDrv->beginSend(reg);
    easyDrv->writeFloat(val1);
    easyDrv->writeFloat(val2);
    easyDrv->writeFloat(val3);
    easyDrv->endSend();
}

void send_2int16(I2CHelper *easyDrv, uint8_t reg, int16_t val1, int16_t val2)
{
    easyDrv->beginSend(reg);
    easyDrv->writeInt16(val1);
    easyDrv->writeInt16(val2);
    easyDrv->endSend();
}

void send_2uint8(I2CHelper *easyDrv, uint8_t reg, uint8_t val1, uint8_t val2)
{
    easyDrv->beginSend(reg);
    easyDrv->writeUint8(val1);
    easyDrv->writeUint8(val2);
    easyDrv->endSend();
}

void send_2int16_1uint8(I2CHelper *easyDrv, uint8_t reg, int16_t val1, int16_t val2, uint8_t val3)
{
    easyDrv->beginSend(reg);
    easyDrv->writeInt16(val1);
    easyDrv->writeInt16(val2);
    easyDrv->writeUint8(val3);
    easyDrv->endSend();
}

void i2c_get_enc_all(I2CHelper *easyDrv,  uint32_t *i32_arr, int16_t *i16_arr)
{
    easyDrv->writeAddr(CMD_GET_ENC_ALL);
    Wire.requestFrom(easyDrv->client, (uint8_t)16); //2*4 + 4*2
    easyDrv->readToUint32Array(i32_arr,2);
    easyDrv->readToInt16Array(i16_arr,4);
    return;
}
void i2c_get_timer_info(I2CHelper *easyDrv, uint32_t *current, uint16_t *one_sec)
{
    easyDrv->writeAddr(CMD_GET_TIME);
    Wire.requestFrom(easyDrv->client, (uint8_t) 6); //4 + 2
    *current = easyDrv->readUint32();
    *one_sec = easyDrv->readUint16();
    return;
}

// connect to local wifi network
void wifi_connect()
{
#ifdef WIFI_SSID
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(ssid, password);
#endif
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());
}

// callback for mqtt
void callback(char *topic, byte *message, unsigned int length)
{
    led_off();
    led_on(true); // red led: receive message from mqtt
    Serial.print("<- ");
    Serial.print(topic);
    Serial.print(": ");
    if (length > 80) {
        return;
    }
    char cstr[length+1];
    for (int i=0;i<length;i++)
    {
        cstr[i] = (char)message[i];
    }
    cstr[length] = 0;
    Serial.println(cstr);
    if (strcmp(topic,"easydrv/deepsleep") == 0)
    {
        if (strcmp(cstr,"now") == 0)
        {
            deepsleep();
        }
    }
    if (strcmp(topic,"easydrv/power") == 0)
    {
        if (strcmp(cstr,"on") == 0)
        {
            easyDrv.sendByte(CMD_PUT_ENABLED_STATE, ENABLE_ALL);
        }
        else if (strcmp(cstr,"off") == 0)
        {
            easyDrv.sendByte(CMD_PUT_ENABLED_STATE, DISABLE_ALL);
        }
    }
    else if (strcmp(topic,"easydrv/pidpars") == 0)
    {
        char* pch = strtok (cstr," ,");
        float pars[3];
        uint8_t n=0;
        while (pch != NULL && (n < 3))
        {
            pars[n++]=atof(pch);
            pch = strtok (NULL, " ,");
        }
        send_3float(&easyDrv,CMD_PUT_PDI_PARS,pars[0],pars[1],pars[2]);
    }
    else if (strcmp(topic,"easydrv/setmode") == 0)
    {
        char* pch = strtok (cstr," ,");
        int16_t pars[3];
        uint8_t n=0;
        while (pch != NULL && (n < 3))
        {
            pars[n++]=atoi(pch);
            pch = strtok (NULL, " ,");
        }
        send_2int16_1uint8(&easyDrv,CMD_PUT_SET_MODE,pars[0],pars[1],(uint8_t) pars[2]);
    }
    else if (strcmp(topic,"easydrv/resetpos") == 0)
    {
        char* pch = strtok (cstr," ,");
        int16_t pars[2];
        uint8_t n=0;
        while (pch != NULL && (n < 2))
        {
            pars[n++]=atoi(pch);
            pch = strtok (NULL, " ,");
        }
        old_pos[0]=pars[0];
        old_pos[1]=pars[1];
        pos_offset[0]=0;
        pos_offset[1]=0;
        send_2int16(&easyDrv,CMD_PUT_RESET_POS,pars[0],pars[1]);
    }
    else if (strcmp(topic,"easydrv/state") == 0)
    {
        uint8_t state = atoi(cstr);
        easyDrv.sendByte(CMD_PUT_DRIVER_STATE, state);
    }
    else if (strcmp(topic,"easydrv/duty12") == 0)
    {

        char* pch = strtok (cstr," ,");
        uint8_t pars[2];
        uint8_t n=0;
        while (pch != NULL && (n < 2))
        {
            pars[n++]=atoi(pch);
            pch = strtok (NULL, " ,");
        }
        send_2uint8(&easyDrv,CMD_PUT_DUTY_CYCLE_12,pars[0],pars[1]);
    }
    led_off();
}

// establish mqtt client connection
void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP32Client"))
        {
            Serial.println("connected");

            // Subscribe to topics
            client.subscribe("easydrv/power");
            client.subscribe("easydrv/state");
            client.subscribe("easydrv/duty12");
            client.subscribe("easydrv/pidpars");
            client.subscribe("easydrv/setmode");
            client.subscribe("easydrv/resetpos");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

// arduino setup
void setup()
{
    char strbuffer[80];

    // init I2C and uart
    Wire.begin();
    Serial.begin(115200);

    // connect wifi
    wifi_connect();
 
    // ntp client
    timeClient.begin();

    // mqtt client
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    // color led on EzSBC board
    pinMode(16, OUTPUT);
    pinMode(17, OUTPUT);
    pinMode(18, OUTPUT);

    // scan I2C printing found devices
    led_off();
    Serial.println("Scanning I2C...");
    scan_i2c(true);

    // wait for our device
    bool found = check_i2c_addr(CLIENT_ADDR);
    if (!found) {
        Serial.println("Waiting for client device to come up ..");
    }
    while(!found) {
        found = check_i2c_addr(CLIENT_ADDR);
        if (!found) {
            led_on(true); // red
            delay(500);
            led_off();
            delay(2500);
        }
    }
    Serial.println("Client device ready");

    // connect to mqtt broker
    reconnect();

    // get time from ntp server
    Serial.println("Get time from NTP server.. ");
    while(!timeClient.update()) {
        timeClient.forceUpdate();
    }
    Serial.print("Current time (UTC): ");
    Serial.println(timeClient.getFormattedTime());
    unsigned long currentEpoch = timeClient.getEpochTime();

    // get current mcu time so we can synch with unix time stamp
    uint32_t current;
    i2c_get_timer_info(&easyDrv, &current, &one_sec);
    one_sec_f = one_sec;
    uint32_t mcu_secs=current/one_sec;  // is probably "0" just after reboot of mcu (current < one_sec)
    unix_offset = currentEpoch - mcu_secs;
    ltoa(unix_offset,strbuffer,10);
    client.publish("easydrv/unixoffset", strbuffer);
    Serial.print("-> easydrv/unixoffset: ");
    Serial.println(strbuffer);
}

void loop()
{
    // arrays to receive data
    int16_t i16_arr[4];
    uint32_t i32_arr[2];
    char strbuffer[80];

    // reconnect if no connection
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    // Get [t1, t2, pos1, pos2, omega1, omega2] from easydrv.
    i2c_get_enc_all(&easyDrv, i32_arr, i16_arr);
    led_on(false, false, true); // set led blue (receiving from i2c)

    if ((i16_arr[0] != old_pos[0]) || (i16_arr[1] != old_pos[1]))  {
        // Positions are signed 16bit values, these can easily overflow -> track sign change 32767 <-> -32768
        for (uint8_t ipos=0; ipos<2; ipos++) {
            int32_t delta = (int32_t)i16_arr[ipos] - (int32_t)old_pos[ipos];
            // overflow produces a big positive or negative jump in the difference
            if (abs(delta) > 30000) {
                if (sgn(delta) < 0) {    
                    pos_offset[ipos] += 65535; // actual position is: pos_offset[ipos]+i16_arr[ipos]
                } else {
                    pos_offset[ipos] -= 65535;
                }
            }
            old_pos[ipos] = i16_arr[ipos];
        }
        // convert to float to get sub second resolution in graph
        float t1 = i32_arr[0]/one_sec_f;
        float t2 = i32_arr[1]/one_sec_f;
        sprintf(strbuffer, "%f %f %d %d %d %d", t1, t2, pos_offset[0]+i16_arr[0], pos_offset[1]+i16_arr[1], i16_arr[2], i16_arr[3]);
        //Serial.println(strbuffer);
        led_on(false, true, false); // switch led to green (sending to mqtt broker)
        client.publish("easydrv/encall", strbuffer);
        Serial.print("-> easydrv/encall: ");
        Serial.println(strbuffer);
    }
    delay(50);
    led_off();
    delay(50);
}