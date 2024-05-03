/*
  growing_station_v1_0.ino
  
  ---------------------------------------------------------------------------
  Copyright (C) 2024 Guy WEILER www.weigu.lu

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
  ---------------------------------------------------------------------------

  for UDP, listen on Linux PC (UDP_LOG_PC_IP) with netcat command:
  nc -kulw 0 6123
  more infos: www.weigu.lu/microcontroller/esptoolbox/index.html
  
  Olimex ESP32 POE
  IRremoteESP8266 by David Conran
  https://github.com/crankyoldgit/IRremoteESP8266

  
*/

/*!!!!!!       Make your changes in config.h (or secrets_xxx.h)      !!!!!!*/

/*------ Comment or uncomment the following line suiting your needs -------*/
#define USE_SECRETS
#define OTA               // if Over The Air update needed (security risk!)
#define STATIC            // if static IP needed (no DHCP)
#define TSL2561           // 3 TSL2561 light sensors
//#define BME280_I2C      // 1 BME280 temp., hum., press. sensor  
#define BME680_I2C        // 1 BME680 environmental sensor
#define FAN               // EasyAcc foldable stand fan KW-MF03BJ with IR commands


/****** Arduino libraries needed ******/
#include "ESPToolbox.h"            // ESP helper lib (more on weigu.lu)
#ifdef USE_SECRETS
  // The file "secrets_xxx.h" has to be placed in a sketchbook libraries
  // folder. Create a folder named "Secrets" in sketchbook/libraries and copy
  // the config.h file there. Rename it to secrets_xxx.h
  #include <secrets_growing_station.h> // things you need to change are here or
#else
  #include "config.h"              // things you need to change are here
#endif // USE_SECRETS
#include <PubSubClient.h>          // for MQTT
#include <ArduinoJson.h>           // convert MQTT messages to JSON
#if defined BME680_I2C || defined TSL2561 || defined BME280_I2C
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
#endif // BME680_I2C || TSL2561
#ifdef TSL2561
  #include <Adafruit_TSL2561_U.h>
#endif // TSL2561
#ifdef BME280_I2C
  #include <BME280I2C.h>
#endif // ifdef BME280_I2C
#ifdef BME680_I2C
  #include <Adafruit_BME680.h>
#endif // BME680_I2C
#ifdef FAN  
  #include <IRremoteESP8266.h>
  #include <IRsend.h>
#endif // FAN


/****** WiFi and network settings ******/
const char *WIFI_SSID = MY_WIFI_SSID;           // if no secrets, use config.h
const char *WIFI_PASSWORD = MY_WIFI_PASSWORD;   // if no secrets, use config.h
#ifdef STATIC
  IPAddress NET_LOCAL_IP (NET_LOCAL_IP_BYTES);  // 3x optional for static IP
  IPAddress NET_GATEWAY (NET_GATEWAY_BYTES);    // look in config.h
  IPAddress NET_MASK (NET_MASK_BYTES);
  IPAddress NET_DNS (NET_DNS_BYTES);
#endif // ifdef STATIC
#ifdef OTA                                      // Over The Air update settings
  const char *OTA_NAME = MY_OTA_NAME;
  const char *OTA_PASS_HASH = MY_OTA_PASS_HASH; // use the config.h file
#endif // ifdef OTA

IPAddress UDP_LOG_PC_IP(UDP_LOG_PC_IP_BYTES);   // UDP log if enabled in setup

/****** MQTT settings ******/
const short MQTT_PORT = MY_MQTT_PORT;
WiFiClient espClient;
PubSubClient MQTT_Client(espClient);

/******* TSL2591 Light Sensor ******/
// we have 3 sensors Addr: LOW = 0x29, FLOAT = 0x39, HIGH = 0x49
#ifdef TSL2561
  Adafruit_TSL2561_Unified tsl1 = Adafruit_TSL2561_Unified(TSL2561_ADDR_LOW, 11111);
  Adafruit_TSL2561_Unified tsl2 = Adafruit_TSL2561_Unified(TSL2561_ADDR_HIGH, 22222);
  Adafruit_TSL2561_Unified tsl3 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 33333);
  sensors_event_t event;
  double light_sensors_data[4] = {0}; // last is average of 3 sensors
#endif // TSL2561 

/******* BME280 ******/
#ifdef BME280_I2C
  float temp(NAN), hum(NAN), pres(NAN);
  BME280I2C bme280; 
#endif // ifdef BME280_I2C

/******* BME680 ******/
#ifdef BME680_I2C  
  double bme680_data[4] = {0};
  Adafruit_BME680 bme680;
#endif // ifdef BME680_I2C

/******* Fan ******/
#ifdef FAN
  IRsend irsend(PIN_IR_LED);
#endif // ifdef FAN

ESPToolbox Tb;                                // Create an ESPToolbox Object

/****** SETUP *************************************************************/

void setup() {
  Tb.set_serial_log(true);            // enable serial logging on Serial
  Tb.set_udp_log(true, UDP_LOG_PC_IP, UDP_LOG_PORT);    
  init_pwm_pins();
  #ifdef STATIC
    Tb.set_static_ip(true,NET_LOCAL_IP, NET_GATEWAY, NET_MASK, NET_DNS);
  #endif // ifdef STATIC
  Tb.init_wifi_sta(WIFI_SSID, WIFI_PASSWORD, NET_MDNSNAME, NET_HOSTNAME);
  Tb.init_ntp_time();
  MQTT_Client.setBufferSize(MQTT_MAXIMUM_PACKET_SIZE);
  MQTT_Client.setServer(MQTT_SERVER,MQTT_PORT); //open connection MQTT server
  #ifdef OTA
    Tb.init_ota(OTA_NAME, OTA_PASS_HASH);
  #endif // ifdef OTA
  #ifdef TSL2561
    init_3_TSL2561_light_sensors();
  #endif // ifdef TSL2561
  #ifdef BME280_I2C
    init_bme280();
  #endif // ifdef BME280_I2C
  #ifdef BME680_I2C
    init_bme680();
  #endif // ifdef BME680_I2C
  #ifdef FAN
    irsend.begin();    
  #endif // FAN
  Tb.blink_led_x_times(3);  
  delay(5000);
  set_pwm_light_lux(light_lux); // light_lux defined in config.h  
  Tb.log_ln("Setup done!");
}

/****** LOOP **************************************************************/

void loop() {
  #ifdef OTA
    ArduinoOTA.handle();
  #endif // ifdef OTA
  Tb.get_time();
  handle_auto_lightning();
  switch (Tb.non_blocking_delay_x3(PUBLISH_TIME, 10000, 5000)) {
    case 1:
      mqtt_get_temp_and_publish();
      break;
    case 2:    
      break;
    case 3:             
      break;
    case 0:
      break;
  }
  handle_fan();
  if (WiFi.status() != WL_CONNECTED) {   // if WiFi disconnected, reconnect
    Tb.init_wifi_sta(WIFI_SSID, WIFI_PASSWORD, NET_MDNSNAME, NET_HOSTNAME);
  }
  if (!MQTT_Client.connected()) {        // reconnect mqtt client, if needed
    mqtt_connect();
  }
  MQTT_Client.loop();                    // make the MQTT live
  delay(10); // needed for the watchdog! (alt. yield())
}

/********** MQTT functions ***************************************************/
// connect to MQTT server
void mqtt_connect() {
  while (!MQTT_Client.connected()) { // Loop until we're reconnected
    Tb.log("Attempting MQTT connection...");
    if (MQTT_Client.connect(MQTT_CLIENT_ID)) { // Attempt to connect
      Tb.log_ln("MQTT connected");
      MQTT_Client.subscribe(MQTT_TOPIC_IN.c_str());
    }
    else {
      Tb.log("MQTT connection failed, rc=");
      Tb.log(String(MQTT_Client.state()));
      Tb.log_ln(" try again in 5 seconds");
      delay(5000);  // Wait 5 seconds before retrying
    }
  }
}

// MQTT get the time, relay flags ant temperature an publish the data
void mqtt_get_temp_and_publish() {
  DynamicJsonDocument doc_out(1024);
  String mqtt_msg, we_msg;
  Tb.get_time();
  doc_out["datetime"] = Tb.t.datetime;
  #ifdef TSL2561
    get_3_light_sensors();
    doc_out["light_s1_lux"] = light_sensors_data[0];
    doc_out["light_s2_lux"] = light_sensors_data[1];
    doc_out["light_s3_lux"] = light_sensors_data[2];
    doc_out["light_avg_lux"] = light_sensors_data[3];
  #endif // ifdef TSL2561  
  #ifdef BME280_I2C
    if (flag_bme280_available == true) {
      get_data_bme280();
      doc_out["temperature_s1_C"] = (int)(temp*10.0 + 0.5)/10.0;
      doc_out["humidity_s1_%"] = (int)(hum*10.0 + 0.5)/10.0;
      doc_out["pressure_hPa"] = (int)((pres + 5)/10)/10.0;
    }
  #endif // ifdef BME280_I2C
  #ifdef BME680_I2C
    if (flag_bme680_available == true) {
      get_data_bme680(bme680_data);
      doc_out["temperature_s1_C"] = (int)(bme680_data[0]*10.0 + 0.5)/10.0;
      doc_out["humidity_s1_%"] = (int)(bme680_data[1]*10.0 + 0.5)/10.0;
      doc_out["pressure_hPa"] = (int)((bme680_data[2] + 5)/10)/10.0;
      doc_out["gas_resistance_kOhms"] = (int)((bme680_data[3] + 5)/10)/10.0;
    }
  #endif // ifdef BME280_I2C
  mqtt_msg = "";
  doc_out["light_percent"] = String(light_percent);  
  doc_out["light_pwm_0_255"] = String(light_pwm);
  doc_out["auto_flag"] = String(auto_flag); 
  doc_out["nr_of_light_events"] = String(nr_of_light_events); 
  for (byte i=0; i<nr_of_light_events; i++) {
    doc_out["light_event"][i]["time"] = String(light_events[i].start_time);
    doc_out["light_event"][i]["percent"] = String(light_events[i].light_percent);
  }  
  serializeJson(doc_out, mqtt_msg);  
  MQTT_Client.publish(MQTT_TOPIC_OUT.c_str(),mqtt_msg.c_str());  
  Tb.log_ln(mqtt_msg);  
}

/*
// MQTT callback
// Commands in JSON: {"Relay_(0-4)":1,"Time_min":20}
//                   {"Auto_(0-1)":1}
//                   {"Event_(nr_relay_start_duration)":"2 3 1900 15"}

void MQTT_callback(char* topic, byte* payload, unsigned int length) {
  DynamicJsonDocument doc_in(256);
  byte nr;
  unsigned int i, tmp;
  unsigned long time_ms;
  String message;
  Tb.log_ln("Message arrived [" + String(topic) + "] ");
  for (int i=0;i<length;i++) {    
    message += String((char)payload[i]);
  }
  Tb.log_ln(message);
  deserializeJson(doc_in, message);
  if (doc_in.containsKey("Auto_(0-1)")) {
    auto_flag = bool(doc_in["Auto_(0-1)"]); 
    Tb.log_ln("Auto_flag arrived [" + String(auto_flag) + "] ");
  }
  else if (doc_in.containsKey("Event_(nr_relay_start_duration)")) {
      String event_str = doc_in["Event_(nr_relay_start_duration)"]; 
      Tb.log_ln(event_str);
      i = event_str.substring(0,1).toInt();      
      tmp = event_str.substring(2,3).toInt();
      watering_events[i].relay_nr = tmp;
      tmp = event_str.substring(4,8).toInt();
      watering_events[i].start_time = tmp;
      tmp = event_str.substring(9).toInt();
      watering_events[i].duration = tmp;  
  }
  else {
    nr = doc_in["Relay_(0-4)"];  
    time_ms = doc_in["Time_min"];
    time_ms = time_ms * 60UL *1000UL;  
    Tb.log_ln("Nr: " + String(nr) + " time: " + String(time_ms));
    if ((nr >= 0) && (nr < NR_OF_RELAYS)) {    
      if (time_ms != 0) {
        relay_times_ms[nr] = time_ms;
        relay_start_flags[nr] = true;
      }
      else {
        relay_times_ms[nr] = 0;
      }    
    }
  }  
}
*/



/********** PWM and Light functions *************************************************/

void init_pwm_pins() {
  ledcAttachPin(LED_PWM_PIN, LED_PWM_CHANNEL);
  ledcSetup(LED_PWM_CHANNEL, PWM_FREQ, PWM_RES);  
}

void init_3_TSL2561_light_sensors() {
  if(!tsl1.begin()) {
    Tb.log_ln("Could not find first TSL2561 sensor!");
    delay(2000);
    if (!tsl1.begin()) { // retry 
        Tb.log_ln("Could not find first TSL2561 sensor, check wiring!");
        delay(2000);
        flag_TSL2561_1_available = false;
      }
    }
    else {
      flag_TSL2561_1_available = true;
      tsl1.enableAutoRange(true); //Auto-gain (between 1x and 16x) 
      tsl1.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  // medium res+speed
    }
  if(!tsl2.begin()) {
    Tb.log_ln("Could not find second TSL2561 sensor!");
    delay(2000);
    if (!tsl2.begin()) { // retry 
        Tb.log_ln("Could not find second TSL2561 sensor, check wiring!");
        delay(2000);
        flag_TSL2561_2_available = false;
      }
    }
    else {
      flag_TSL2561_2_available = true;
      tsl2.enableAutoRange(true); //Auto-gain (between 1x and 16x) 
      tsl2.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  // medium res+speed
    }
  if(!tsl3.begin()) {
    Tb.log_ln("Could not find third TSL2561 sensor!");
    delay(2000);
    if (!tsl3.begin()) { // retry 
        Tb.log_ln("Could not find third TSL2561 sensor, check wiring!");
        delay(2000);
        flag_TSL2561_3_available = false;
      }
    }
    else {
      flag_TSL2561_3_available = true;
      tsl3.enableAutoRange(true); //Auto-gain (between 1x and 16x) 
      tsl3.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  // medium res+speed
    }  
}

void get_3_light_sensors() {  
  if (flag_TSL2561_1_available == true) {
    tsl1.getEvent(&event);
    if (event.light) {
      light_sensors_data[0] = event.light;
    }
    else {
      Tb.log_ln("Sensor1 overload"); 
    }
  }
  if (flag_TSL2561_2_available == true) {
    tsl2.getEvent(&event);
    if (event.light) {
      light_sensors_data[1] = event.light;
    }
    else {
      Tb.log_ln("Sensor2 overload"); 
    }
  }  
  if (flag_TSL2561_3_available == true) {
    tsl3.getEvent(&event);
    if (event.light) {
      light_sensors_data[2] = event.light;
    }
    else {
      Tb.log_ln("Sensor3 overload"); 
    }
  }  
  calculate_average_light_from_3_sensors();
}  

void set_pwm_light_percent(byte light_percent) { 
  byte table_index = byte((light_percent/5)+0.49); 
  light_pwm =  light_pwm_table[table_index].light_pwm;
  ledcWrite(LED_PWM_CHANNEL, light_pwm);
}

void set_pwm_light_0_255(byte light_pwm) {    
  ledcWrite(LED_PWM_CHANNEL, light_pwm);
}

void set_pwm_light_lux(unsigned long light_lux) {  
  Tb.log_ln("Setting pwm from lux"); 
  byte light_percent = 0;  
  set_pwm_light_percent(light_percent);
  delay(1000);  
  while(light_sensors_data[3] < light_lux) {
    light_percent += 5;
    set_pwm_light_percent(light_percent);
    delay(1000);  
    get_3_light_sensors();
    Tb.log_ln(String(light_percent) + "     " + String(light_sensors_data[3])); 
    if (light_percent == 100) {
      break;
    }    
  } 
  Tb.log_ln(String(nr_of_light_events)); 
  for (byte i = 0; i < nr_of_light_events/2-1; i++) {
    Tb.log_ln(String(i) + "  " + String(nr_of_light_events-2-i) + " " +String((i+1)*light_percent/(nr_of_light_events/2)));
    light_events[i].light_percent = (i+1)*light_percent/(nr_of_light_events/2);
    light_events[nr_of_light_events-2-i].light_percent = (i+1)*light_percent/(nr_of_light_events/2);
  }  
  light_events[nr_of_light_events/2-1].light_percent = light_percent;
}

void calculate_average_light_from_3_sensors() {
  unsigned long light_sensors_data_sum = 0;
  light_sensors_data[3] = 0;
  byte divider = 0;
  if (light_sensors_data[0] < 65000) { // check if no sensor error
    light_sensors_data_sum += light_sensors_data[0];
    divider += 1;
  }
  if (light_sensors_data[1] < 65000) { // check if no sensor error
    light_sensors_data_sum += light_sensors_data[1];
    divider += 1;
  }
  if (light_sensors_data[2] < 65000) { // check if no sensor error
    light_sensors_data_sum += light_sensors_data[2];
    divider += 1;
  }
  if (divider != 0) { // check if no sensor error
    light_sensors_data[3] = light_sensors_data_sum/divider;
  }  
}

void handle_auto_lightning() {  
  unsigned int time_now = Tb.t.hour*100+Tb.t.minute;   
  if (auto_flag) {
    if (time_now < light_events[0].start_time) { //still night
      light_percent = 0;
      set_pwm_light_percent(light_percent);
    }  
    else if (time_now >= light_events[nr_of_light_events-1].start_time) { //night again
      light_percent = 0;
      set_pwm_light_percent(light_percent);
    }  
    else {
      for (byte i=0; i<nr_of_light_events-1; i++) {
        if ((time_now >= light_events[i].start_time) && (time_now < light_events[i+1].start_time)) {
          light_percent = light_events[i].light_percent;
          set_pwm_light_percent(light_percent);
        }
      }      
    }
  }  
}


/********** BME280 functions *************************************************/
#ifdef BME280_I2C
  // initialise the BME280 sensor
  // Default : forced mode, standby time = 1000 ms
  // Oversampling = press. ×1, temp. ×1, hum. ×1, filter off
  void init_bme280() {
    Wire.begin();
    if (!bme280.begin()) {
      Tb.log_ln("Could not find BME280 sensor!");
      delay(2000);
      if (!bme280.begin()) { // retry 
        Tb.log_ln("Could not find a valid BME280 sensor, check wiring!");
        delay(2000);
        flag_bme280_available = false;
      }
    }
    else {
      flag_bme280_available = true;
      switch(bme280.chipModel())  {
        case BME280::ChipModel_BME280:
           Tb.log_ln("Found BME280 sensor! Success.");
           break;
        case BME280::ChipModel_BMP280:
           Tb.log_ln("Found BMP280 sensor! No Humidity available.");
           break;
        default:
           Tb.log_ln("Found UNKNOWN sensor! Error!");
      }
    }
  }

  // get BME280 data and log it
  void get_data_bme280() {
    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);
    bme280.read(pres, temp, hum, tempUnit, presUnit);
    Tb.log_ln("Temp: " + (String(temp)) + " Hum: " + (String(hum)) +
            " Pres: " + (String(pres)));
  }
#endif // ifdef BME280_I2C

/********** BME680 functions *************************************************/
#ifdef BME680_I2C
  // initialise the BME680 sensor
  void init_bme680() {
    if (!bme680.begin()) {
      Tb.log_ln("Could not find a valid BME680 sensor, check wiring!");
      delay(2000);
      if (!bme680.begin()) { // retry 
        Tb.log_ln("Could not find a valid BME680 sensor, check wiring!");
        delay(2000);
        flag_bme680_available = false;
      }
    }
    else {
      flag_bme680_available = true;
          // Set up oversampling and filter initialization
      bme680.setTemperatureOversampling(BME680_OS_8X);
      bme680.setHumidityOversampling(BME680_OS_2X);
      bme680.setPressureOversampling(BME680_OS_4X);
      bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
      bme680.setGasHeater(320, 150); // 320*C for 150 ms
    }
  }

  // get BME680 data and log it
  void get_data_bme680(double bme680_data[]) {  
  unsigned long endTime = bme680.beginReading(); // Begin measurement
  if (endTime == 0) {
    Tb.log_ln("Failed to begin reading :(");
    return;
  }
  if (!bme680.endReading()) {
    Tb.log_ln("Failed to complete reading :(");
    return;
  }  
    bme680_data[0] = bme680.temperature;
    bme680_data[1] = bme680.humidity;
    bme680_data[2] = bme680.pressure;
    bme680_data[3] = bme680.gas_resistance;
  }
#endif // ifdef BME680_I2C

/****** fan functions ******/
/* Commands for EasyAcc foldable stand fan KW-MF03BJ
 * address = 0x80
 * commands:
 * ON/OFF       0x12
 * Timing       0x1E
 * Oscillate    0x4
 * Naturel wind 0x6
 * -            0xA
 * +            0x1F
 */ 
#ifdef FAN

void handle_fan() {
  if ((Tb.t.hour >= 6) && (Tb.t.hour <= 22) && (Tb.t.hour%2 == 0)
     && (Tb.t.minute == 0) && (Tb.t.second == 0)) {    
    fan_1_hour_natural_oscillate();      
    Tb.log_ln("IR control for fan sent!");
  }
}

void fan_1_hour_natural_oscillate() {
  fan_send_command_on_off(); // on
  delay(100);
  fan_send_command_timing(); // 1h
  delay(100);
  fan_send_command_plus();
  delay(100);
  fan_send_command_oscillate();
  delay(100);
  fan_send_command_oscillate(); // reduce to 30 degree
  delay(100);
  fan_send_command_natural_wind();
}    

void fan_send_command_on_off() { // 1x on 2x off
  // address = 0x80, command = 0x12 raw = 0xED127F80 LSB 0x01FE48B7 MSB; 
  irsend.sendNEC(0x01FE48B7UL);
}

void fan_send_command_timing() { // timer 1-8h (+/- to adjust) 
  // address = 0x80, command = 0x1E raw = 0xE11E7F80 LSB 0x01FE7887 MSB; 
  irsend.sendNEC(0x01FE7887UL);
}

void fan_send_command_oscillate() { // 1x 60° 2x 30° 3x off
  // address = 0x80, command = 0x04 raw = 0xFB047F80 LSB 0x01FE20BF MSB; 
  irsend.sendNEC(0x01FE20BFUL);
}

void fan_send_command_natural_wind() { // 1x on 2x off
  // address = 0x80, command = 0x06 raw = 0xF9067F80 LSB 0x01FE609F MSB;
  irsend.sendNEC(0x01FE609FUL);
}

void fan_send_command_plus() { // +
  // address = 0x80, command = 0x1F raw = 0xE01F7F80 LSB 0x01FEF807 MSB; 
  irsend.sendNEC(0x01FEF807UL);
}

void fan_send_command_minus() { // -
  // address = 0x80, command = 0x0A raw = 0xF50A7F80 LSB 0x01FE50AF MSB; 
  irsend.sendNEC(0x01FE50AFUL);
}

#endif // ifdef FAN

/****** helper functions ******/

void displaySensorDetails() {
  sensor_t sensor;
  tsl1.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
