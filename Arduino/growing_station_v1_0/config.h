/****** WiFi SSID and PASSWORD ******/
const char *MY_WIFI_SSID = "your_ssid";
const char *MY_WIFI_PASSWORD = "your_password";

/****** WiFi and network settings ******/
// UDP logging settings if enabled in setup(); Port used for UDP logging
const word UDP_LOG_PORT = 6464;
// IP address of the computer receiving UDP log messages
const byte UDP_LOG_PC_IP_BYTES[4] = {192, 168, 178, 100};
// optional (access with UDP_logger.local)
const char *NET_MDNSNAME = "growing_station";
// optional hostname
const char *NET_HOSTNAME = "growing_station";
// only if you use a static address (uncomment //#define STATIC in ino file)
const byte NET_LOCAL_IP_BYTES[4] = {192, 168, 178, 155};
const byte NET_GATEWAY_BYTES[4] = {192, 168, 178, 1};
const byte NET_MASK_BYTES[4] = {255,255,255,0};
const byte NET_DNS_BYTES[4] = {8,8,8,8}; //  second dns (first = gateway), 8.8.8.8 = google
// only if you use OTA (uncomment //#define OTA in ino file)
const char *MY_OTA_NAME = "growing_station"; // optional (access with esp_with_ota.local)
// Linux Create Hasgh with: echo -n 'P@ssword1' | md5sum
const char *MY_OTA_PASS_HASH = "myHash";     // Hash for password

/****** MQTT settings ******/
const char *MQTT_SERVER = "192.168.178.222";
const long PUBLISH_TIME = 10000; //Publishes every in milliseconds
const int MQTT_MAXIMUM_PACKET_SIZE = 1024; // look in setup()
const char *MQTT_CLIENT_ID = "growing_station_15287952425"; // this must be unique!!!
String MQTT_TOPIC_OUT = "weigu/growing_station/data";
String MQTT_TOPIC_IN = "weigu/growing_station/command";
const short MY_MQTT_PORT = 1883; // or 8883
// only if you use MQTTPASSWORD (uncomment //#define MQTTPASSWORD in ino file)
const char *MY_MQTT_USER = "me";
const char *MY_MQTT_PASS = "meagain";

/****** BME sensors ******/
bool flag_bme280_available = false;
bool flag_bme680_available = false;

/****** Fan IR control ******/
const byte PIN_IR_LED = 5;

/****** Light control ******/
bool flag_TSL2561_1_available = false;
bool flag_TSL2561_2_available = false;
bool flag_TSL2561_3_available = false;

const byte LED_PWM_PIN = 3;
const byte LED_PWM_CHANNEL = 0;      // channels 0-15
const unsigned int PWM_FREQ = 1000;  // freq limits depend on resolution
const byte PWM_RES = 8;              // resolution 1-16 bits
byte light_percent = 0;
byte light_pwm = 0;
unsigned int light_lux = 7500;

struct pwm {
  byte light_percent;
  byte light_pwm;
};

pwm light_pwm_table[] = {
  {  0, 255},
  {  5, 195},
  { 10, 178},
  { 15, 162},
  { 20, 147},
  { 25, 136},
  { 30, 125},
  { 35, 114},
  { 40, 104},
  { 45,  94},
  { 50,  83},
  { 55,  73},
  { 60,  62},
  { 65,  51},
  { 70,  41},
  { 75,  30},
  { 80,  20},
  { 85,   8},
  { 90,   2},
  { 95,   1},
  {100,   0}
};

/******* Automated light control *******/
bool auto_flag = 1;

struct le {
  unsigned int start_time;
  unsigned int light_percent;
};
// light events should be an even number, last event light percent = 0
le light_events[] = {
  {530,10},
  {535,20},
  {540,30},
  {545,40},
  {550,50},
  {555,60},
  {2230,50},
  {2235,40},
  {2240,30},
  {2245,20},
  {2250,10},
  {2255,0}
};

byte nr_of_light_events = sizeof(light_events)/sizeof(light_events[0]);