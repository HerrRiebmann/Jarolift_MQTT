/*  Controlling Jarolift TDEF 433MHZ radio shutters via ESP8266 and CC1101 Transceiver Module in asynchronous mode.
    Copyright (C) 2017-2018 Steffen Hille et al.

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
*/


// Changelog: see CHANGES.md

/*
  Kanal  S/N           DiscGroup_8-16             DiscGroup_1-8     SN(last two digits)
  0       0            0000 0000                   0000 0001           0000 0000
  1       1            0000 0000                   0000 0010           0000 0001
  2       2            0000 0000                   0000 0100           0000 0010
  3       3            0000 0000                   0000 1000           0000 0011
  4       4            0000 0000                   0001 0000           0000 0100
  5       5            0000 0000                   0010 0000           0000 0101
  6       6            0000 0000                   0100 0000           0000 0110
  7       7            0000 0000                   1000 0000           0000 0111
  8       8            0000 0001                   0000 0000           0000 0111
  9       9            0000 0010                   0000 0000           0000 0111
  10      10           0000 0100                   0000 0000           0000 0111
  11      11           0000 1000                   0000 0000           0000 0111
  12      12           0001 0000                   0000 0000           0000 0111
  13      13           0010 0000                   0000 0000           0000 0111
  14      14           0100 0000                   0000 0000           0000 0111
  15      15           1000 0000                   0000 0000           0000 0111
*/

#define DEVICE 0

//Board:
//NodeMCU 1.0 (ESP-12E Module)


#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <SPI.h>
#include <FS.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <DoubleResetDetector.h>
#include <simpleDSTadjust.h>
#include <coredecls.h>              // settimeofday_cb()

#include "helpers.h"
#include "global.h"
#include "html_api.h"

extern "C" {
#include "user_interface.h"
#include "Arduino.h"
#include "cc1101.h"
#include <KeeloqLib.h>
}

// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 10

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

// User configuration
#define PULSE_SHORT      400    // Defines pulse-width in microseconds. Adapt for your use...
#define PULSE_LONG       800

#define BITS_SIZE          8
byte syncWord            = 199;
int device_key_msb       = 0x0; // stores cryptkey MSB
int device_key_lsb       = 0x0; // stores cryptkey LSB
uint64_t button          = 0x0; // 1000=0x8 up, 0100=0x4 stop, 0010=0x2 down, 0001=0x1 learning
int disc                 = 0x0;
uint32_t dec             = 0;   // stores the 32Bit encrypted code
uint64_t pack            = 0;   // Contains data to send.
byte disc_low[16]        = {0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0,  0x0,  0x0,  0x0};
byte disc_high[16]       = {0x0, 0x0, 0x0, 0x0, 0x0,  0x0,  0x0,  0x0, 0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80};
byte disc_l              = 0;
byte disc_h              = 0;
byte adresses[]          = {5, 11, 17, 23, 29, 35, 41, 47, 53, 59, 65, 71, 77, 85, 91, 97 }; // Defines start addresses of channel data stored in EEPROM 4bytes s/n.
uint64_t new_serial      = 0;
byte marcState;
int MqttRetryCounter = 0;                 // Counter for MQTT reconnect

// RX variables and defines
#define debounce         200              // Ignoring short pulses in reception... no clue if required and if it makes sense ;)
#define pufsize          216              // Pulsepuffer
#define TX_PORT            4              // Outputport for transmission
#define RX_PORT            5              // Inputport for reception
uint32_t rx_serial       = 0;
char rx_serial_array[8]  = {0};
char rx_disc_low[8]      = {0};
char rx_disc_high[8]     = {0};
uint32_t rx_hopcode      = 0;
uint16_t rx_disc_h       = 0;
byte rx_function         = 0;
int rx_device_key_msb    = 0x0;           // stores cryptkey MSB
int rx_device_key_lsb    = 0x0;           // stores cryptkey LSB
volatile uint32_t decoded         = 0x0;  // decoded hop code
volatile byte pbwrite;
volatile unsigned int lowbuf[pufsize];    // ring buffer storing LOW pulse lengths
volatile unsigned int hibuf[pufsize];     // ring buffer storing HIGH pulse lengths
volatile bool iset = false;
volatile byte value = 0;                  // Stores RSSI Value
long rx_time;
int steadycnt = 0;
boolean time_is_set_first = true;
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);
CC1101 cc1101;                            // The connection to the hardware chip CC1101 the RF Chip

// forward declarations
void ICACHE_RAM_ATTR radio_rx_measure();

//Over the Air Update
#include <ArduinoOTA.h>
bool UploadIsOTA = false;
#if DEVICE
const char deviceNameShort[] = "JaroliftESP8266-2";
#else
const char deviceNameShort[] = "JaroliftESP8266";
#endif

//####################################################################
// sketch initialization routine
//####################################################################
void setup()
{
  InitLog();
  EEPROM.begin(4096);
  Serial.begin(115200);
  settimeofday_cb(time_is_set);
  updateNTP(); // Init the NTP time
  WriteLog("[INFO] - starting Jarolift Dongle " + (String)PROGRAM_VERSION, true);
  WriteLog("[INFO] - ESP-ID " + (String)ESP.getChipId() + " // ESP-Core  " + ESP.getCoreVersion() + " // SDK Version " + ESP.getSdkVersion(), true);

  // callback functions for WiFi connect and disconnect
  // placed as early as possible in the setup() function to get the connect
  // message catched when the WiFi connect is really fast
  gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP & event)
  {
    WriteLog("[INFO] - WiFi station connected - IP: " + WiFi.localIP().toString(), true);
    wifi_disconnect_log = true;
  });

  disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected & event)
  {
    if (wifi_disconnect_log) {
      WriteLog("[INFO] - WiFi station disconnected", true);
      // turn off logging disconnect events after first occurrence, otherwise the log is filled up
      wifi_disconnect_log = false;
    }
  });

  InitializeConfigData();
  EEPROM.get(cntadr, devcnt);

  InitCC1101();

  pinMode(led_pin, OUTPUT);   // prepare LED on ESP-Chip

  // test if the WLAN SSID is on default
  // or DoubleReset detected
  if ((drd.detectDoubleReset()) || (config.ssid == "MYSSID")) {
    digitalWrite(led_pin, LOW);  // turn LED on                    // if yes then turn on LED
    AdminEnabled = true;                                           // and go to Admin-Mode
  } else {
    digitalWrite(led_pin, HIGH); // turn LED off                   // turn LED off
  }

  // enable access point mode if Admin-Mode is enabled
  if (AdminEnabled)
  {
    WriteLog("[WARN] - Admin-Mode enabled!", true);
    WriteLog("[WARN] - starting soft-AP ... ", false);
    wifi_disconnect_log = false;
    WiFi.mode(WIFI_AP);
    WriteLog(WiFi.softAP(ACCESS_POINT_NAME, ACCESS_POINT_PASSWORD) ? "Ready" : "Failed!", true);
    WriteLog("[WARN] - Access Point <" + (String)ACCESS_POINT_NAME + "> activated. WPA password is " + ACCESS_POINT_PASSWORD, true);
    WriteLog("[WARN] - you have " + (String)AdminTimeOut + " seconds time to connect and configure!", true);
    WriteLog("[WARN] - configuration webserver is http://" + WiFi.softAPIP().toString(), true);
  }
  else
  {
    // establish Wifi connection in station mode
    ConfigureWifi();
  }

  InitWebServer();
  
  tkHeartBeat.attach(1, HeartBeat);

  MqttInit();

  pinMode(TX_PORT, OUTPUT); // TX Pin

  // RX
  pinMode(RX_PORT, INPUT_PULLUP);
  attachInterrupt(RX_PORT, radio_rx_measure, CHANGE); // Interrupt on change of RX_PORT

  InitSchedules();
  SetupOTA();
} // void setup

//####################################################################
// main loop
//####################################################################
void loop()
{

  // Call the double reset detector loop method every so often,
  // so that it can recognise when the timeout expires.
  // You can also call drd.stop() when you wish to no longer
  // consider the next reset as a double reset.
  drd.loop();

  // disable Admin-Mode after AdminTimeOut
  if (AdminEnabled)
  {
    if (AdminTimeOutCounter > AdminTimeOut / HEART_BEAT_CYCLE)
    {
      AdminEnabled = false;
      digitalWrite(led_pin, HIGH);   // turn LED off
      WriteLog("[WARN] - Admin-Mode disabled, soft-AP terminate ...", false);
      WriteLog(WiFi.softAPdisconnect(true) ? "success" : "fail!", true);
      ConfigureWifi();
    }
  }
  server.handleClient();

  CC1101Loop();

  MqttLoop();

  // run a CMD whenever a web_cmd event has been triggered
  if (web_cmd != "") {

    iset = true;
    detachInterrupt(RX_PORT); // Interrupt on change of RX_PORT
    delay(1);

    if (web_cmd == "up") {
      cmd_up(web_cmd_channel);
    } else if (web_cmd == "down") {
      cmd_down(web_cmd_channel);
    } else if (web_cmd == "stop") {
      cmd_stop(web_cmd_channel);
    } else if (web_cmd == "set shade") {
      cmd_set_shade_position(web_cmd_channel);
    } else if (web_cmd == "shade") {
      cmd_shade(web_cmd_channel);
    } else if (web_cmd == "learn") {
      cmd_learn(web_cmd_channel);
    } else if (web_cmd == "updown") {
      cmd_updown(web_cmd_channel);
    } else if (web_cmd == "save") {
      Serial.println("main loop: in web_cmd save");
      cmd_save_config();
    } else if (web_cmd == "restart") {
      Serial.println("main loop: in web_cmd restart");
      cmd_restart();
    } else {
      WriteLog("[ERR ] - received unknown command from web_cmd.", true);
    }
    web_cmd = "";
  }

  SchedulerLoop();
  OTA_Handle();
} // void loop

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// execute cmd_ functions group
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//####################################################################
// function to move the shutter up
//####################################################################
void cmd_up(int channel) {
  EEPROM.get(adresses[channel], new_serial);
  EEPROM.get(cntadr, devcnt);
  button = 0x8;
  disc_l = disc_low[channel];
  disc_h = disc_high[channel];
  disc = (disc_l << 8) | (new_serial & 0xFF);
  rx_disc_low[0]  = disc_l;
  rx_disc_high[0] = disc_h;
  keygen();
  keeloq();
  entertx();
  radio_tx(2);
  enterrx();
  rx_function = 0x8;
  rx_serial_array[0] = (new_serial >> 24) & 0xFF;
  rx_serial_array[1] = (new_serial >> 16) & 0xFF;
  rx_serial_array[2] = (new_serial >> 8) & 0xFF;
  rx_serial_array[3] = new_serial & 0xFF;
  mqtt_send_percent_closed_state(channel, 0, "UP");
  devcnt_handler(true);
} // void cmd_up

//####################################################################
// function to move the shutter down
//####################################################################
void cmd_down(int channel) {
  EEPROM.get(adresses[channel], new_serial);
  EEPROM.get(cntadr, devcnt);
  button = 0x2;
  disc_l = disc_low[channel];
  disc_h = disc_high[channel];
  disc = (disc_l << 8) | (new_serial & 0xFF);
  rx_disc_low[0]  = disc_l;
  rx_disc_high[0] = disc_h;
  keygen();
  keeloq();  // Generate encrypted message 32Bit hopcode
  entertx();
  radio_tx(2); // Call TX routine
  enterrx();
  rx_function = 0x2;
  rx_serial_array[0] = (new_serial >> 24) & 0xFF;
  rx_serial_array[1] = (new_serial >> 16) & 0xFF;
  rx_serial_array[2] = (new_serial >> 8) & 0xFF;
  rx_serial_array[3] = new_serial & 0xFF;
  mqtt_send_percent_closed_state(channel, 100, "DOWN");
  devcnt_handler(true);
} // void cmd_down

//####################################################################
// function to stop the shutter
//####################################################################
void cmd_stop(int channel) {
  EEPROM.get(adresses[channel], new_serial);
  EEPROM.get(cntadr, devcnt);
  button = 0x4;
  disc_l = disc_low[channel];
  disc_h = disc_high[channel];
  disc = (disc_l << 8) | (new_serial & 0xFF);
  rx_disc_low[0]  = disc_l;
  rx_disc_high[0] = disc_h;
  keygen();
  keeloq();
  entertx();
  radio_tx(2);
  enterrx();
  rx_function = 0x4;
  rx_serial_array[0] = (new_serial >> 24) & 0xFF;
  rx_serial_array[1] = (new_serial >> 16) & 0xFF;
  rx_serial_array[2] = (new_serial >> 8) & 0xFF;
  rx_serial_array[3] = new_serial & 0xFF;
  WriteLog("[INFO] - command STOP for channel " + (String)channel + " (" + config.channel_name[channel] + ") sent.", true);
  devcnt_handler(true);
} // void cmd_stop

//####################################################################
// function to move shutter to shade position
//####################################################################
void cmd_shade(int channel) {
  EEPROM.get(adresses[channel], new_serial);
  EEPROM.get(cntadr, devcnt);
  button = 0x4;
  disc_l = disc_low[channel];
  disc_h = disc_high[channel];
  disc = (disc_l << 8) | (new_serial & 0xFF);
  rx_disc_low[0]  = disc_l;
  rx_disc_high[0] = disc_h;
  keygen();
  keeloq();
  entertx();
  radio_tx(20);
  enterrx();
  rx_function = 0x3;
  rx_serial_array[0] = (new_serial >> 24) & 0xFF;
  rx_serial_array[1] = (new_serial >> 16) & 0xFF;
  rx_serial_array[2] = (new_serial >> 8) & 0xFF;
  rx_serial_array[3] = new_serial & 0xFF;
  mqtt_send_percent_closed_state(channel, 90, "SHADE");
  devcnt_handler(true);
} // void cmd_shade

//####################################################################
// function to set the learn/set the shade position
//####################################################################
void cmd_set_shade_position(int channel) {
  EEPROM.get(adresses[channel], new_serial);
  EEPROM.get(cntadr, devcnt);
  button = 0x4;
  disc_l = disc_low[channel];
  disc_h = disc_high[channel];
  disc = (disc_l << 8) | (new_serial & 0xFF);
  rx_disc_low[0]  = disc_l;
  rx_disc_high[0] = disc_h;
  keygen();

  for (int i = 0; i < 4; i++) {
    entertx();
    keeloq();
    radio_tx(1);
    devcnt++;
    enterrx();
    delay(300);
  }
  rx_function = 0x6;
  rx_serial_array[0] = (new_serial >> 24) & 0xFF;
  rx_serial_array[1] = (new_serial >> 16) & 0xFF;
  rx_serial_array[2] = (new_serial >> 8) & 0xFF;
  rx_serial_array[3] = new_serial & 0xFF;
  WriteLog("[INFO] - command SET SHADE for channel " + (String)channel + " (" + config.channel_name[channel] + ") sent.", true);
  devcnt_handler(false);
  delay(2000); // Safety time to prevent accidentally erase of end-points.
} // void cmd_set_shade_position

//####################################################################
// function to put the dongle into the learn mode and
// send learning packet.
//####################################################################
void cmd_learn(int channel) {
  WriteLog("[INFO] - putting channel " +  (String) channel + " into learn mode ...", false);
  new_serial = EEPROM.get(adresses[channel], new_serial);
  EEPROM.get(cntadr, devcnt);
  if (config.learn_mode == true)
    button = 0xA;                           // New learn method. Up+Down followd by Stop.
  else
    button = 0x1;                           // Old learn method for receiver before Mfg date 2010.
  disc_l = disc_low[channel] ;
  disc_h = disc_high[channel];
  disc = (disc_l << 8) | (new_serial & 0xFF);
  keygen();
  keeloq();
  entertx();
  radio_tx(1);
  enterrx();
  devcnt++;
  if (config.learn_mode == true) {
    delay(1000);
    button = 0x4;   // Stop
    keeloq();
    entertx();
    radio_tx(1);
    enterrx();
    devcnt++;
  }
  devcnt_handler(false);
  WriteLog("Channel learned!", true);
} // void cmd_learn

//####################################################################
// function to send UP+DOWN button at same time
//####################################################################
void cmd_updown(int channel) {
  new_serial = EEPROM.get(adresses[channel], new_serial);
  EEPROM.get(cntadr, devcnt);
  button = 0xA;
  disc_l = disc_low[channel] ;
  disc_h = disc_high[channel];
  disc = (disc_l << 8) | (new_serial & 0xFF);
  keygen();
  keeloq();
  entertx();
  radio_tx(1);
  enterrx();
  devcnt_handler(true);
  WriteLog("[INFO] - command UPDOWN for channel " + (String)channel + " (" + config.channel_name[channel] + ") sent.", true);
} // void cmd_updown

//####################################################################
// webUI save config function
//####################################################################
void cmd_save_config() {
  WriteLog("[CFG ] - save config initiated from WebUI", true);
  // check if mqtt_devicetopic was changed
  if (config.mqtt_devicetopic_new != config.mqtt_devicetopic) {
    // in case the devicetopic has changed, the LWT state with the old devicetopic should go away
    WriteLog("[CFG ] - devicetopic changed, gracefully disconnect from mqtt server", true);
    // first we send an empty message that overwrites the retained "Online" message
    String topicOld = "tele/" + config.mqtt_devicetopic + "/LWT";
    mqtt_client.publish(topicOld.c_str(), "", true);
    // next: remove retained "devicecounter" message
    topicOld = "stat/" + config.mqtt_devicetopic + "/devicecounter";
    mqtt_client.publish(topicOld.c_str(), "", true);
    delay(200);
    // finally we disconnect gracefully from the mqtt broker so the stored LWT "Offline" message is discarded
    mqtt_client.disconnect();
    config.mqtt_devicetopic = config.mqtt_devicetopic_new;
    delay(200);
  }
  if (config.set_and_generate_serial) {
    WriteLog("[CFG ] - set and generate new serial, user input: " + config.new_serial, true);
    if ((config.new_serial[0] == '0') && (config.new_serial[1] == 'x')) {
      Serial.println("config.serial is hex");
      // Serial is 28 bits
      // string serial stores only highest 24 bits,
      // add lowest 4 bits with a shift operation for config.serial_number
      config.serial_number = strtoul(config.new_serial.c_str(), NULL, 16) << 4;
      // be safe an convert number back to clean 6-digit hex string
      char serialNumBuffer[11];
      snprintf(serialNumBuffer, 11, "0x%06x", (config.serial_number >> 4));
      config.serial = serialNumBuffer;
      Serial.printf("config.serial: %08u = 0x%08x \n", config.serial_number, config.serial_number);
      cmd_generate_serials(config.serial_number);
    } else {
      server.send ( 200, "text/plain", "Set new Serial not successful, not a hexadecimal number.");
      return;
    }
  }
  if (config.set_devicecounter) {
    uint16_t new_devcnt = strtoul(config.new_devicecounter.c_str(), NULL, 10);
    WriteLog("[CFG ] - set devicecounter to " + String(new_devcnt), true);
    devcnt = new_devcnt;
    devcnt_handler(false);
  }
  WriteConfig();
  server.send ( 200, "text/plain", "Configuration has been saved, system is restarting. Please refresh manually in about 30 seconds.." );
  cmd_restart();
} // void cmd_save_config

//####################################################################
// webUI restart function
//####################################################################
void cmd_restart() {
  server.send ( 200, "text/plain", "System is restarting. Please refresh manually in about 30 seconds." );
  delay(500);
  wifi_disconnect_log = false;
  ESP.restart();
} // void cmd_restart

//####################################################################
// generates 16 serial numbers
//####################################################################
void cmd_generate_serials(uint32_t sn) {
  WriteLog("[CFG ] - Generate serial numbers starting from" + String(sn), true);
  uint32_t z = sn;
  for (uint32_t i = 0; i <= 15; ++i) { // generate 16 serial numbers and storage in EEPROM
    EEPROM.put(adresses[i], z);   // Serial 4Bytes
    z++;
  }
  devcnt = 0;
  devcnt_handler(false);
  delay(100);
} // void cmd_generate_serials

//####################################################################
// NTP update
//####################################################################
void updateNTP() {
  configTime(TIMEZONE * 3600, 0, NTP_SERVERS);
} // void updateNTP

//####################################################################
// callback function when time is set via SNTP
//####################################################################
void time_is_set(void) {
  if (time_is_set_first) {    // call WriteLog only once for the initial time set
    time_is_set_first = false;
    WriteLog("[INFO] - time set from NTP server", true);
  }
} // void time_is_set
