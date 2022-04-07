#include <Arduino.h>
/*
Wifi e-ink
----------------------
https://remoteqth.com

 ___               _        ___ _____ _  _
| _ \___ _ __  ___| |_ ___ / _ \_   _| || |  __ ___ _ __
|   / -_) '  \/ _ \  _/ -_) (_) || | | __ |_/ _/ _ \ '  \
|_|_\___|_|_|_\___/\__\___|\__\_\|_| |_||_(_)__\___/_|_|_|


This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Remote USB access "screen /dev/ttyUSB0 115200"
HARDWARE ESP32-GATEWAY

Changelog:
2019-11-06 - e-Ink U8g2lib libs

---
  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)
  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.
  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//-------------------------------------------------------------------------------------------------------
//  CONFIGURE
//-------------------------------------------------------------------------------------------------------

const String WX_TOPIC = "OK1HRA-6";           // same as 'location' on meteo wx station
IPAddress mqtt_server_ip(54, 38, 157, 134);   // same MQTT boker as use meteo wx station
int MQTT_PORT = 1883;                         // MQTT broker port
char* SsidPass[3][2] = {                      // three Wifi SSID and password - select by buttons after start up
  {"SSID1","password1"},
  {"SSID2","password2"},
  {"SSID3","password3"},
};

//-------------------------------------------------------------------------------------------------------

#define REV 20210320
#define OTAWEB                      // enable upload firmware via web
#define MQTT                      // enable MQTT
// #define SELECTAPP
// #define SELECWIFI
// #define ENABLEBUTT

#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
  #include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
  #include <Wire.h>
#endif
#include <esp_adc_cal.h>

/*
  U8g2lib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.
    https://www.waveshare.com/wiki/2.7inch_e-Paper_HAT
*/

// Please UNCOMMENT one of the contructor lines below
// U8g2 Contructor List (Frame Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
// U8G2_SSD1607_200X200_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 18, /* data=*/ 23, /* cs=*/ 5, /* dc=*/ 17, /* reset=*/ 16);	// eInk/ePaper Display, original LUT from embedded artists

// OK1HRA-e-ink
// U8G2_SSD1607_200X200_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 21, /* data=*/ 22, /* cs=*/ 5, /* dc=*/ 19, /* reset=*/ 18);	// eInk/ePaper Display, original LUT from embedded artists
U8G2_SSD1607_200X200_F_4W_SW_SPI u8g2(U8G2_R2, /* clock=*/ 21, /* data=*/ 22, /* cs=*/ 5, /* dc=*/ 19, /* reset=*/ 18);	// eInk/ePaper Display, original LUT from embedded artists
// U8G2_SSD1607_GD_200X200_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 18, /* data=*/ 23, /* cs=*/ 5, /* dc=*/ 17, /* reset=*/ 16);	// Good Display

// https://www.waveshare.com/wiki/E-Paper_ESP32_Driver_Board
// https://www.waveshare.com/wiki/2.9inch_e-Paper_Module
// U8G2_SSD1607_WS_200X200_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 14, /* cs=*/ 15, /* dc=*/ 27, /* reset=*/ 26);	// Waveshare E-Paper ESP32 Driver Board

// https://www.waveshare.com/wiki/1.54inch_e-Paper_Module
// End of constructor list
bool eInkNeedRefresh = false;
long eInkRefreshBanTimer;

int Az=0;
char buf[4];
#define RAD_TO_DEG 57.295779513082320876798154814105

//WX
float WindDir;
float WindSpeedAvg;
float WindSpeedMaxPeriod;
float Pressure;
float HumidityRel;
float TemperatureC;

// ntp
#include "time.h"
const char* ntpServer = "pool.ntp.org";
// const char* ntpServer = "tik.cesnet.cz";
// const char* ntpServer = "time.google.com";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;

// PINOUT
const int PwrPin = 15;
const int MeasureOnPin = 14;
long MeasureOnTimeout[2] = {0,1000};
long PowerOffTimeout[2] = {0,10000};

const int LedPin = 2;
int LedPinStatus = 0;
int LedPinPWM = 0;
bool LedPinDir = true;
unsigned long LedPinTimer = 0;
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

const int BattPin = 36;
const int StatPin = 35;
const int VccPin = 39;
float BattVoltage;
float StatVoltage;
float VccVoltage;
// Command to see the REF_VOLTAGE: espefuse.py --port /dev/ttyUSB0 adc_info
// or dc2_vref_to_gpio(25)
#define REF_VOLTAGE 1100
esp_adc_cal_characteristics_t *adc_chars = new esp_adc_cal_characteristics_t;

const int LswPin = 32; //12;
const int CswPin = 34; //0;
const int RswPin = 13;

int DebuggingOutput = 1;  // 0-off | 1-Serial

#define WIFI
#include <WiFi.h>
// #include <ETH.h>
int SsidPassSize = (sizeof(SsidPass)/sizeof(char *))/2; //array size
int SelectSsidPass = -1;
#define wifi_max_try 10             // Number of try

unsigned int RunApp = 255;
#if defined(OTAWEB)
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <AsyncElegantOTA.h>
  AsyncWebServer OTAserver(80);
#endif

#if defined(MQTT)
  #include <PubSubClient.h>
  // #include "PubSubClient.h" // lokalni verze s upravou #define MQTT_MAX_PACKET_SIZE 128
  // WiFiClient esp32Client;
  // PubSubClient mqttClient(esp32Client);
  WiFiClient espClient;
  PubSubClient mqttClient(espClient);
  // PubSubClient mqttClient(ethClient);
  // PubSubClient mqttClient(server, 1883, callback, ethClient);
  long lastMqttReconnectAttempt = 0;
  // bool HeartBeatCfm = true;
  // unsigned long HeartBeatCounter;
  // unsigned long HeartBeatTimer;
  // unsigned long LatencyTimer;
  // bool MQTT_ENABLE     = 1;          // enable public to MQTT broker
  // IPAddress mqtt_server_ip(0, 0, 0, 0);
  // byte BrokerIpArray[2][4]{
  //   // {192,168,1,200},   // MQTT broker remoteqth.com
  //   {54,38,157,134},   // MQTT broker remoteqth.com
  // };
  // IPAddress server(10, 24, 213, 92);    // MQTT broker
  // int MQTT_PORT;       // MQTT broker PORT
  // int MQTT_PORT_Array[2] = {
  //   1883,
  //   1883
  // };       // MQTT broker PORT
  boolean MQTT_LOGIN      = 0;          // enable MQTT broker login
  // char MQTT_USER= 'login';    // MQTT broker user login
  // char MQTT_PASS= 'passwd';   // MQTT broker password
  const int MqttBuferSize = 1000; // 1000
  char mqttTX[MqttBuferSize];
  char mqttPath[MqttBuferSize];
  // char mqttTX[100];
  // char mqttPath[100];
  long MqttStatusTimer[2]{1500,1000};
  // long HeartBeatTimer[2]={0,1000};
#endif

//-------------------------------------------------------------------------------------------------------
void setup(void) {
  #if defined(ENABLEBUTT)
    pinMode(PwrPin, OUTPUT);
    digitalWrite(PwrPin, HIGH);
    pinMode(MeasureOnPin, OUTPUT);
    digitalWrite(MeasureOnPin, HIGH);
    pinMode(LedPin, OUTPUT);
    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(LedPin, ledChannel);
    // ledcWrite(ledChannel, 0);

    analogReadResolution(11);
    //// esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_11, REF_VOLTAGE, adc_chars);

    pinMode(BattPin, INPUT);
    pinMode(StatPin, INPUT);
    pinMode(VccPin, INPUT);

    pinMode(LswPin, INPUT);
    pinMode(CswPin, INPUT);
    pinMode(RswPin, INPUT);
  #endif

  u8g2.begin();
  Serial.begin(115200);
  Serial.print("RemoteQTH WiFi e-ink rev ");
  Serial.println(REV);


  #if defined(WIFI)
    WiFi.disconnect(true);

    #if defined(SELECWIFI)
      SelectSsidPass = SelectMenu("Select WIFI", 45, SsidPass[0][0], SsidPass[1][0], SsidPass[2][0]);
    #else
      SelectSsidPass=0;
    #endif

    WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_STA_CONNECTED);
    WiFi.onEvent(WiFiGotIP, SYSTEM_EVENT_STA_GOT_IP);
    WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
    /* Remove WiFi event
    Serial.print("WiFi Event ID: ");
    Serial.println(eventID);
    WiFi.removeEvent(eventID);*/

    Serial.print("WIFI SSID ");
    Serial.print(SsidPass[SelectSsidPass][0]);
    Serial.print(" | ");
    Serial.println(SsidPass[SelectSsidPass][1]);
    WiFi.begin(SsidPass[SelectSsidPass][0], SsidPass[SelectSsidPass][1]);
    Serial.print("Connecting ");
    int count_try = 0;
    while(WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      count_try++;    // Increase try counter
      if ( count_try >= wifi_max_try ) {
        Serial.println("\n");
        Serial.println("Impossible to establish WiFi connexion");

        print_wifi_error();
        // Serial.println("Sleep a little and retry later, bye");

        // https://diyprojects.io/esp32-how-to-connect-local-wifi-network-arduino-code/
        // Set the wakeup
        // esp_sleep_enable_timer_wakeup(duration_deep_sleep * 1000000);
        // And ask processor to sleep now
        // esp_deep_sleep_start();
      }
    }

    Serial.println("");
    Serial.println("WIFI connected");
    Serial.print("WIFI IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("WIFI dBm: ");
    Serial.println(WiFi.RSSI());

    #if !defined(SELECWIFI)
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_logisoso18_tf);
      String(SsidPass[SelectSsidPass][0]).toCharArray(buf, 25);
      u8g2.drawStr(0,100, buf);
      String(WiFi.RSSI()).toCharArray(buf, 10);
      strcat(buf, " dBm");
      u8g2.drawStr(50,140, buf);
      u8g2.sendBuffer();
    #endif

    #if defined(MQTT)
      if (MQTT_LOGIN == true){
        // if (mqttClient.connect("esp32gwClient", MQTT_USER, MQTT_PASS)){
        //   AfterMQTTconnect();
        // }
      }else{
        mqttClient.setServer(mqtt_server_ip, MQTT_PORT);
        Serial.println("EthEvent-MQTTclient");
        mqttClient.setCallback(MqttRx);
        Serial.println("EthEvent-MQTTcallback");
        lastMqttReconnectAttempt = 0;

        char charbuf[50];
         // memcpy( charbuf, ETH.macAddress(), 6);
         WiFi.macAddress().toCharArray(charbuf, 18);
         // charbuf[6] = 0;
        if (mqttClient.connect(charbuf)){
          Serial.print("EthEvent-maccharbuf ");
          Serial.println(charbuf);
          // Prn(3, 1, String(charbuf));
          mqttReconnect();
        }
      }
    #endif

  #endif
  // delay(2000);
  #if defined(SELECTAPP)
    RunApp = SelectMenu("Run App", 45, "WX", "Rotator", "Window");
    eInkNeedRefresh=true;
  #else
    RunApp = 0;
  #endif


  #if defined(OTAWEB)
    OTAserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "PSE QSY to /update");
    });
    AsyncElegantOTA.begin(&OTAserver);    // Start ElegantOTA
    OTAserver.begin();
  #endif

  //init and get the time
   configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}
// https://github.com/olikraus/u8g2/wiki/fntgrplogisoso
// https://github.com/olikraus/u8g2/wiki/u8g2reference

//-------------------------------------------------------------------------------------------------------
void loop(void) {
  LED();
  Watchdog();
  Mqtt();
  eInk(RunApp);

  // u8g2.setDrawColor(1);
  // u8g2.setBitmapMode(0);
  // u8g2.drawXBM( 0, 0, rq_logo_width, rq_logo_height, rq_logo_bits);
  // delay(1000);
  // Az--;
  // eInk(1);
  // Serial.print(".");
  // delay(5000);
  //   // Az=Az+10;
  //   // Az--;
  // eInk(2);
  // Serial.println(".");
  // delay(5000);
  //   // Az=Az+10;
  //   // Az--;

  // if(Az>360){
  //   Az=0;
  // }
  #if defined(OTAWEB)
   AsyncElegantOTA.loop();
  #endif
}

//-------------------------------------------------------------------------------------------------------
int SelectMenu(char* MENU, int Mxx, char* M1, char* M2, char* M3){

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso22_tr);
  u8g2.drawStr(Mxx, 40, M1);
  u8g2.drawStr(Mxx, 70, M2);
  u8g2.drawStr(Mxx, 100, M3);

  u8g2.drawBox(0,120,200,80);
  u8g2.setFontMode(1);  /* activate transparent font mode */
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_logisoso18_tf);
  u8g2.drawStr(Mxx, 150, MENU);
  const char* IP = String(WiFi.localIP()).c_str();
  u8g2.drawStr(10, 180, IP );

  u8g2.sendBuffer();
  u8g2.setFontMode(0);
  u8g2.setDrawColor(1);
  LedPinStatus = 4;  // 0=off 1=on 2=blinkFast 3=blikSlow 4=fadeSlow

  while(1){
    LED();
    if(digitalRead(LswPin)==0){
      LedPinStatus = 0;  // 0=off 1=on 2=blinkFast 3=blikSlow 4=fadeSlow
      return 0;
    }
    if(digitalRead(CswPin)==0){
      LedPinStatus = 0;  // 0=off 1=on 2=blinkFast 3=blikSlow 4=fadeSlow
      return 1;
    }
    if(digitalRead(RswPin)==0){
      LedPinStatus = 0;  // 0=off 1=on 2=blinkFast 3=blikSlow 4=fadeSlow
      return 2;
    }
  }
  return -1;
}

//-------------------------------------------------------------------------------------------------------
void Watchdog(){
  #if defined(ENABLEBUTT)
    // ADC
    if ((millis() - MeasureOnTimeout[0]) > MeasureOnTimeout[1]){
      /*
        The maximum voltage is limited by VDD_A
        - 0dB attenuaton (ADC_ATTEN_DB_0) gives full-scale voltage 1.1V
        - 2.5dB attenuation (ADC_ATTEN_DB_2_5) gives full-scale voltage 1.5V
        - 6dB attenuation (ADC_ATTEN_DB_6) gives full-scale voltage 2.2V
        - 11dB attenuation (ADC_ATTEN_DB_11) gives full-scale voltage 3.9V
      */
      // Default is 11db, full scale

      digitalWrite(MeasureOnPin, LOW);
      BattVoltage = float(analogRead_cal(BattPin, ADC_ATTEN_DB_11)) / 1000.0f * (13.3f/10.0f);
      digitalWrite(MeasureOnPin, HIGH);
      // Debugging("Batt "+String(BattVoltage)+" V (3,6-4,2V)");

      digitalWrite(MeasureOnPin, LOW);
      StatVoltage = float(analogRead_cal(StatPin, ADC_ATTEN_DB_11)) / 1000.0f * (13.3f/10.0f);
      digitalWrite(MeasureOnPin, HIGH);
      // Debugging("Stat "+String(StatVoltage)+" V (Charge complete H)");

      VccVoltage = float(analogRead_cal(VccPin, ADC_ATTEN_DB_11)) / 1000.0f * (20.0f/10.0f);
      // Debugging("Vcc "+String(VccVoltage)+" V ");

      // Debugging("L switch "+String(digitalRead(LswPin)));
      // Debugging("C switch "+String(digitalRead(CswPin)));
      // Debugging("R switch "+String(digitalRead(RswPin)));

      MeasureOnTimeout[0]=millis();
    }
  #endif

  // POWER OFF
  // if ((millis() - PowerOffTimeout[0]) > PowerOffTimeout[1]){
  //   PowerOffTimeout[0]=millis();
  //   Debugging("Power OFF...");
  //   digitalWrite(PwrPin, LOW);
  // }
  static long TouchBanTimer;

  if ((millis() - TouchBanTimer) > 1000){
    switch (RunApp) {
        // WX
        case 0:
          #if defined(ENABLEBUTT)
            if(digitalRead(LswPin)==0){
              MqttPubString("get", "4eink", false);
              TouchBanTimer=millis();
            }
          #endif
        break;

        case 1:
        break;

        case 2:
        break;
    }
  }

}

//-------------------------------------------------------------------------------------------------------
void LED(){
  switch (LedPinStatus) {
    case (0):
      if(LedPinPWM!=0){
        ledcWrite(ledChannel, 0);
        LedPinPWM=0;
      }
      break;

    case (1):
      if(LedPinPWM!=255){
        ledcWrite(ledChannel, 255);
        LedPinPWM=255;
      }
      break;

    case (2): // blink fast
      if(millis()-LedPinTimer>50){
        if(LedPinPWM==255){
          LedPinPWM=0;
        }else{
          LedPinPWM=255;
        }
        ledcWrite(ledChannel, LedPinPWM);
        LedPinTimer=millis();
      }
      break;

    case (3): // blink slow
      if(millis()-LedPinTimer>150){
        if(LedPinPWM==255){
          LedPinPWM=0;
        }else{
          LedPinPWM=255;
        }
        ledcWrite(ledChannel, LedPinPWM);
        LedPinTimer=millis();
      }
      break;

    case (4): // fade slow
      if(millis()-LedPinTimer>1){
        if(LedPinDir==true){
          LedPinPWM++;
        }else{
          LedPinPWM--;
        }
        if(LedPinPWM<=0 || LedPinPWM>=254){
          LedPinDir=!LedPinDir;
        }
        ledcWrite(ledChannel, LedPinPWM);
        LedPinTimer=millis();
      }
      break;
  }
}


//-------------------------------------------------------------------------------------------------------
int analogRead_cal(uint8_t channel, adc_atten_t attenuation) {
  adc1_channel_t channelNum;

  /*
     Set number of cycles per sample
     Default is 8 and seems to do well
     Range is 1 - 255
   * */
  // analogSetCycles(uint8_t cycles);

  /*
     Set number of samples in the range.
     Default is 1
     Range is 1 - 255
     This setting splits the range into
     "samples" pieces, which could look
     like the sensitivity has been multiplied
     that many times
   * */
  // analogSetSamples(uint8_t samples);

  switch (channel) {
    case (36):
      channelNum = ADC1_CHANNEL_0;
      break;

    case (39):
      channelNum = ADC1_CHANNEL_3;
      break;

    case (34):
      channelNum = ADC1_CHANNEL_6;
      break;

    case (35):
      channelNum = ADC1_CHANNEL_7;
      break;

    case (32):
      channelNum = ADC1_CHANNEL_4;
      break;

    case (33):
      channelNum = ADC1_CHANNEL_5;
      break;
  }

  adc1_config_channel_atten(channelNum, attenuation);
  return esp_adc_cal_raw_to_voltage(analogRead(channel), adc_chars);
}

//-------------------------------------------------------------------------------------------------------
void eInk(int View){
  if(eInkNeedRefresh==true && (millis()-eInkRefreshBanTimer)>1500 ){
    int XX;
    u8g2.clearBuffer();
    switch (View) {

        // WX
        case 0:
          if(TemperatureC>=0 && TemperatureC<=9){
            XX=60;
          }else if( (TemperatureC>=-9 && TemperatureC<=-1)||(TemperatureC>=10 && TemperatureC<=99) ){
            XX=34;
          }else{
            XX=0;
          }
          // https://github.com/olikraus/u8g2/wiki/fntgrplogisoso
          // Temp
          u8g2.setFont(u8g2_font_logisoso92_tn);
          dtostrf(TemperatureC, 1, 0, buf);  //1 is mininum width, 0 is precision
          u8g2.drawStr(XX,105, buf);
          u8g2.setFont(u8g2_font_logisoso58_tf);
          u8g2.drawGlyph(180-XX, 80, 176);
          // UTC
          // u8g2.setFont(u8g2_font_logisoso18_tf);
          u8g2.setFont(u8g2_font_logisoso16_tr);
          UtcTime(2).toCharArray(buf, 6);
          u8g2.drawStr(150,115, buf);
          // Wind Dir
          u8g2.drawBox(0,120,200,80);
          u8g2.setFontMode(1);  /* activate transparent font mode */
          u8g2.setDrawColor(0);
          Arrow(WindDir,40,160,30);
          // Wind speed
          u8g2.setFont(u8g2_font_logisoso18_tf);
          dtostrf(WindSpeedMaxPeriod, 1, 0, buf);  //1 is mininum width, 1 is precision
          strcat(buf, " m/s");
          u8g2.drawStr(90,150, buf);
          // Pressure & humidity
          u8g2.drawBox(85,162,100,2);
          u8g2.drawDisc(85+(int)HumidityRel, 163, 4, U8G2_DRAW_ALL);
          dtostrf(Pressure, 1, 0, buf);  //1 is mininum width, 0 is precision
          strcat(buf, " hpa");
          u8g2.drawStr(90,190, buf);
          // itoa (WiFi.RSSI(), buf, 10);
          // u8g2.drawStr(85,190, buf);
          // u8g2.drawStr(125,190, "dBm");

          u8g2.setFontMode(0);
          u8g2.setDrawColor(1);
          // u8g2.drawStr(45,180, "rotate");
        break;

        case 1:
          u8g2.setFont(u8g2_font_logisoso30_tf);
          itoa (Az, buf, 10);
          u8g2.drawStr(70,195,buf);
          u8g2.drawGlyph(130,195,176);
          DirectionalRosette(Az, 100, 75, 70);
        break;

        case 2:
          DirectionalRosette(Az, 100, 100, 95);
          // u8g2.setFontMode(1);  /* activate transparent font mode */
          // u8g2.setDrawColor(2); // activate color2 (XOR)
          u8g2.setFont(u8g2_font_logisoso30_tf);
          itoa (Az, buf, 10);
          u8g2.drawStr(70,175,buf);
          u8g2.drawGlyph(130,180,176);
        break;

        case 3:
          u8g2.drawRFrame(0,0,95,45,7);
            u8g2.drawRFrame(105,0,95,45,7);
          u8g2.drawRFrame(0,55,95,45,7);
            u8g2.drawRFrame(105,55,95,45,7);
          u8g2.drawRFrame(0,105,95,45,7);
            u8g2.drawRFrame(105,105,95,45,7);
          u8g2.drawRFrame(0,155,95,45,7);
            u8g2.drawRFrame(105,155,95,45,7);
          // DirectionalRosette(Az, 100, 100, 95);
          // u8g2.setFontMode(1);  /* activate transparent font mode */
          // u8g2.setDrawColor(2); // activate color2 (XOR)
          // u8g2.setFont(u8g2_font_logisoso30_tf);
          // itoa (Az, buf, 10);
          // u8g2.drawStr(70,180,buf);
          // u8g2.drawGlyph(130,180,176);
        break;
    }
    u8g2.sendBuffer();
    eInkNeedRefresh=false;
  }
}
//-------------------------------------------------------------------------------------------------------

  float Xcoordinate(int dir, int Center, int r){
    float x = Center + sin(dir/RAD_TO_DEG) * r;
    return x;
  }
//-------------------------------------------------------------------------------------------------------

  float Ycoordinate(int dir, int Center, int r){
    float y = Center - cos(dir/RAD_TO_DEG) * r;
    return y;
  }

//-------------------------------------------------------------------------------------------------------

  void Arrow(int deg, int X, int Y, int r){
    int deg2 = deg+130;
    int deg3 = deg+230;
    u8g2.drawTriangle(Xcoordinate(deg,X,r), Ycoordinate(deg,Y,r), Xcoordinate(deg2,X,r/2), Ycoordinate(deg2,Y,r/2), Xcoordinate(deg+180,X,0), Ycoordinate(deg+180,Y,0));
    u8g2.drawTriangle(Xcoordinate(deg,X,r), Ycoordinate(deg,Y,r), Xcoordinate(deg3,X,r/2), Ycoordinate(deg3,Y,r/2), Xcoordinate(deg+180,X,0), Ycoordinate(deg+180,Y,0));
    u8g2.drawTriangle(Xcoordinate(deg+180,X,r), Ycoordinate(deg+180,Y,r), Xcoordinate(deg3,X,r/10), Ycoordinate(deg3,Y,r/10), Xcoordinate(deg+180,X,0), Ycoordinate(deg+180,Y,0));
    u8g2.drawTriangle(Xcoordinate(deg+180,X,r), Ycoordinate(deg+180,Y,r), Xcoordinate(deg2,X,r/10), Ycoordinate(deg2,Y,r/10), Xcoordinate(deg+180,X,0), Ycoordinate(deg+180,Y,0));
  }

//-------------------------------------------------------------------------------------------------------

  void DirectionalRosette(int deg, int X, int Y, int R){
    int dot1;
    int dot2;
    if(R>70){
      dot1=2;
      dot2=5;
    }else{
      dot1=1;
      dot2=3;
    }
    u8g2.setFont(u8g2_font_logisoso16_tr);
    u8g2.drawStr(Xcoordinate(0,X-5,R-10), Ycoordinate(0,Y,R-10), "N");
    u8g2.drawStr(Xcoordinate(90,X+5,R-10), Ycoordinate(90,Y+8,R-10), "E");
    u8g2.drawStr(Xcoordinate(180,X-6,R-10), Ycoordinate(180,Y+13,R-10), "S");
    u8g2.drawStr(Xcoordinate(270,X-16,R-10), Ycoordinate(270,Y+8,R-10), "W");
    for (int j=0; j<36; j++) {
      if(j % 9 == 0){
        // u8g2.drawDisc(Xcoordinate(j*10,X,R), Ycoordinate(j*10,Y,R), dot2, U8G2_DRAW_ALL);
      }else{
        u8g2.drawDisc(Xcoordinate(j*10,X,R), Ycoordinate(j*10,Y,R), dot1, U8G2_DRAW_ALL);
      }
    }
    Arrow(deg,X,Y,R*0.9);
}

//-------------------------------------------------------------------------------------------------------

void Debugging(String StringForDebug){
  StringForDebug.reserve(50);
  if(DebuggingOutput!=0){
    // Serial0 KEY
    if(DebuggingOutput==1){
      Serial.print(StringForDebug);
      Serial.println();
    }
    // Serial2 CAT
    if(DebuggingOutput==2){
      // Serial2.print(StringForDebug);
      // Serial2.println();
    }
    // UDP IP
    // if(DebuggingOutput==3 && EnableEthernet==true && EthLinkStatus==1){
      // InterruptON(0,0,0,0); // keyb, enc, gps, Interlock
      // DebuggingIP = ~Ethernet.subnetMask() | Ethernet.gatewayIP();
      // UdpCommand.beginPacket(DebuggingIP, DebuggingPort);
      // // UdpCommand.beginMulticast(UdpCommand.BroadcastIP(), BroadcastPort, ETH.localIP()).
      //   UdpCommand.print(F("NET_ID"));
      //   UdpCommand.print(F("debug:"));
      //   UdpCommand.print(StringForDebug);
      //   UdpCommand.print(F(";"));
      // UdpCommand.endPacket();
      // InterruptON(1,1,1,1); // keyb, enc, gps, Interlock
    // }
    // MQTT
    if(DebuggingOutput==4){
      // MqttPubString("debug", StringForDebug, false);
    }
  }
}

//-------------------------------------------------------------------------------------------------------
void print_wifi_error(){
  switch(WiFi.status())
  {
    case WL_IDLE_STATUS : Serial.println("WL_IDLE_STATUS"); break;
    case WL_NO_SSID_AVAIL : Serial.println("WL_NO_SSID_AVAIL"); break;
    case WL_CONNECT_FAILED : Serial.println("WL_CONNECT_FAILED"); break;
    case WL_DISCONNECTED : Serial.println("WL_DISCONNECTED"); break;
    default : Serial.printf("No know WiFi error"); break;
  }
}
//-------------------------------------------------------------------------------------------------------
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Event|Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.print("Event|WiFi connected, IP address: ");
  Serial.println(WiFi.localIP());
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.print("Event|Disconnected from WiFi access point > Reason: ");
  Serial.println(info.disconnected.reason);
  WiFi.begin(SsidPass[SelectSsidPass][0], SsidPass[SelectSsidPass][1]);
  Serial.print("Event|Reconnecting ");
  LedPinStatus=1;
  LED();
  int count_try = 0;
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    count_try++;    // Increase try counter
    if ( count_try >= wifi_max_try ) {
      Serial.println("\n");
      Serial.println("Event|Impossible to establish WiFi connexion");

      print_wifi_error();
      // Serial.println("Sleep a little and retry later, bye");

      // https://diyprojects.io/esp32-how-to-connect-local-wifi-network-arduino-code/
      // Set the wakeup
      // esp_sleep_enable_timer_wakeup(duration_deep_sleep * 1000000);
      // And ask processor to sleep now
      // esp_deep_sleep_start();
    }
  }
  Serial.println("");
  Serial.print("Event|WIFI connected, IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Event|WIFI dBm: ");
  Serial.println(WiFi.RSSI());

  #if defined(MQTT)
    if (MQTT_LOGIN == true){
      // if (mqttClient.connect("esp32gwClient", MQTT_USER, MQTT_PASS)){
      //   AfterMQTTconnect();
      // }
    }else{
      mqttClient.setServer(mqtt_server_ip, MQTT_PORT);
      Serial.println("EthEvent-MQTTclient");
      mqttClient.setCallback(MqttRx);
      Serial.println("EthEvent-MQTTcallback");
      lastMqttReconnectAttempt = 0;

      char charbuf[50];
       // memcpy( charbuf, ETH.macAddress(), 6);
       WiFi.macAddress().toCharArray(charbuf, 18);
       // charbuf[6] = 0;
      if (mqttClient.connect(charbuf)){
        Serial.print("EthEvent-maccharbuf ");
        Serial.println(charbuf);
        // Prn(3, 1, String(charbuf));
        mqttReconnect();
      }
    }
  #endif

  LedPinStatus=0;
  LED();
}

//-------------------------------------------------------------------------------------------------------
void Mqtt(){
  #if defined(MQTT)
    if (millis()-MqttStatusTimer[0]>MqttStatusTimer[1]){
      if(!mqttClient.connected()){
        long now = millis();
        if (now - lastMqttReconnectAttempt > 5000) {
          lastMqttReconnectAttempt = now;
          Serial.print("Attempt to MQTT reconnect | ");
          Serial.println(millis()/1000);
          if (mqttReconnect()) {
            lastMqttReconnectAttempt = 0;
          }
        }
      }else{
        // Client connected
        mqttClient.loop();
      }
      MqttStatusTimer[0]=millis();
    }
  #endif
}

//-------------------------------------------------------------------------------------------------------

#if defined(MQTT)
bool mqttReconnect() {
    // charbuf[6] = 0;
    char charbuf[50];
    // memcpy( charbuf, ETH.macAddress(), 6);
    WiFi.macAddress().toCharArray(charbuf, 18);
    if (mqttClient.connect(charbuf)) {
      Serial.println("mqttReconnect-connected");

      // resubscribe
      String topic = String(WX_TOPIC) + "/WX/WindDir-azimuth";
      topic.reserve(50);
      const char *cstr0 = topic.c_str();
      if(mqttClient.subscribe(cstr0)==true){
        Serial.print("mqttReconnect-subscribe ");
        Serial.println(String(cstr0));
      }

      topic = String(WX_TOPIC) + "/WX/Temperature-Celsius";
      topic.reserve(50);
      const char *cstr1 = topic.c_str();
      if(mqttClient.subscribe(cstr1)==true){
        Serial.print("mqttReconnect-subscribe ");
        Serial.println(String(cstr1));
      }

      topic = String(WX_TOPIC) + "/WX/WindSpeedMaxPeriod-mps";
      topic.reserve(50);
      const char *cstr2 = topic.c_str();
      if(mqttClient.subscribe(cstr2)==true){
        Serial.print("mqttReconnect-subscribe ");
        Serial.println(String(cstr2));
      }

      topic = String(WX_TOPIC) + "/WX/Pressure-hPa";
      topic.reserve(50);
      const char *cstr3 = topic.c_str();
      if(mqttClient.subscribe(cstr3)==true){
        Serial.print("mqttReconnect-subscribe ");
        Serial.println(String(cstr3));
      }

      topic = String(WX_TOPIC) + "/WX/HumidityRel-Percent";
      topic.reserve(50);
      const char *cstr4 = topic.c_str();
      if(mqttClient.subscribe(cstr4)==true){
        Serial.print("mqttReconnect-subscribe ");
        Serial.println(String(cstr4));
      }

    }
    MqttPubString("get", "4eink", false);
    return mqttClient.connected();
}
#endif

//------------------------------------------------------------------------------------
void MqttRx(char *topic, byte *payload, unsigned int length) {
  #if defined(MQTT)
    String CheckTopicBase;
    CheckTopicBase.reserve(100);
    byte* p = (byte*)malloc(length);
    memcpy(p,payload,length);
    // static bool HeardBeatStatus;
    Serial.println("RX mqtt...");

    // WX
      CheckTopicBase = String(WX_TOPIC) + "/WX/WindDir-azimuth";
      if ( CheckTopicBase.equals( String(topic) )){
        int NR = 0;
        unsigned long exp = 1;
        for (int i = length-1; i >=0 ; i--) {
          // Numbers only
          if(p[i]>=48 && p[i]<=58){
            NR = NR + (p[i]-48)*exp;
            exp = exp*10;
          }
        }
        WindDir=NR;
        eInkNeedRefresh=true;
        eInkRefreshBanTimer=millis();
      }

      CheckTopicBase = String(WX_TOPIC) + "/WX/Temperature-Celsius";
      if ( CheckTopicBase.equals( String(topic) )){
        String buf = "";
        for (int i = 0; i <=length-1 ; i++) {
          buf = buf+String((char)p[i]);
        }
        TemperatureC = buf.toFloat();
        Serial.print("Temperature-Celsius ");
        Serial.println(TemperatureC);
        eInkNeedRefresh=true;
        eInkRefreshBanTimer=millis();
      }

      CheckTopicBase = String(WX_TOPIC) + "/WX/WindSpeedMaxPeriod-mps";
      if ( CheckTopicBase.equals( String(topic) )){
        String buf = "";
        for (int i = 0; i <=length-1 ; i++) {
          buf = buf+String((char)p[i]);
        }
        WindSpeedMaxPeriod = buf.toFloat();
        Serial.print("WindSpeedMaxPeriod-mps ");
        Serial.println(WindSpeedMaxPeriod);
        eInkNeedRefresh=true;
        eInkRefreshBanTimer=millis();
      }

      CheckTopicBase = String(WX_TOPIC) + "/WX/Pressure-hPa";
      if ( CheckTopicBase.equals( String(topic) )){
        String buf = "";
        for (int i = 0; i <=length-1 ; i++) {
          buf = buf+String((char)p[i]);
        }
        Pressure = buf.toFloat();
        Serial.print("Pressure-hPa ");
        Serial.println(Pressure);
        eInkNeedRefresh=true;
        eInkRefreshBanTimer=millis();
      }

      CheckTopicBase = String(WX_TOPIC) + "/WX/HumidityRel-Percent";
      if ( CheckTopicBase.equals( String(topic) )){
        String buf = "";
        for (int i = 0; i <=length-1 ; i++) {
          buf = buf+String((char)p[i]);
        }
        HumidityRel = buf.toFloat();
        Serial.print("HumidityRel-Percent ");
        Serial.println(HumidityRel);
        eInkNeedRefresh=true;
        eInkRefreshBanTimer=millis();
      }

  #endif
} // MqttRx END

//-----------------------------------------------------------------------------------
void MqttPubString(String TOPIC, String DATA, bool RETAIN){
  char charbuf[50];
   // memcpy( charbuf, mac, 6);
   WiFi.macAddress().toCharArray(charbuf, 18);
  // if(EnableEthernet==1 && MQTT_ENABLE==1 && EthLinkStatus==1 && mqttClient.connected()==true){
  if(mqttClient.connected()==true){
    if (mqttClient.connect(charbuf)) {
      String topic = String(WX_TOPIC) + "/WX/"+TOPIC;
      topic.toCharArray( mqttPath, 50 );
      DATA.toCharArray( mqttTX, 50 );
      mqttClient.publish(mqttPath, mqttTX, RETAIN);
      Serial.print("TX mqtt > ");
      Serial.print(mqttPath);
      Serial.print(" ");
      Serial.println(mqttTX);
    }
  }
}

//-------------------------------------------------------------------------------------------------------
String UtcTime(int format){
  tm timeinfo;
  char buf[50]; //50 chars should be enough
  // if(WiFi.status() == WL_CONNECTED) {
  //   strcpy(buf, "n/a");
  // }else{
    if(!getLocalTime(&timeinfo)){
      strcpy(buf, "n/a");
    }else{
      if(format==1){
        strftime(buf, sizeof(buf), "%Y-%b-%d %H:%M:%S", &timeinfo);
      }else if(format==2){
        strftime(buf, sizeof(buf), "%H:%M", &timeinfo);
      }else if(format==3){
        strftime(buf, sizeof(buf), "%Y", &timeinfo);
      }
    }
  // }
  // Serial.println(buf);
  return String(buf);
}
