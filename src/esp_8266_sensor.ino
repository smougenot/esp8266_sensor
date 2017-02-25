#include <FS.h>                   //this needs to be first, or it all crashes and burns...

//needed for library
#include <DNSServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

// I2C sensors
#include <Wire.h>
#include <Adafruit_BMP085_U.h>
// Display
#include <TM1637Display.h>
// send MQTT messages
#include <PubSubClient.h>

#define PRJ_VERSION 2

// wait time (ms)
#define LOOP_WAIT 60 * 100

// I2C pins
#define IC_CLK  14
#define IC_DATA 12

// display pins
#define CLK   2
#define DATA  0

// Display
TM1637Display display(CLK, DATA);

//Sensor

// address pin is low
int BH1750address = 0x23; //setting i2c address
byte buff[2];

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5 esp8266 -> #5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4  esp8266 -> #4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

Adafruit_BMP085_Unified bmp;

// MQTT
#define CLIENT_ID     "1"
char mqtt_server[120]="mqtt.broker.net";
char mqtt_port_config[5]="1883";
int mqtt_port = 1883;
const char* mqtt_server_home = "192.168.1.17";
const char* mqtt_server_mobile = "192.168.43.219";
const char* topicCmd    = "/esp/1/cmd";
const char* topicStatus = "/esp/1/status";
const char* clientId    = "ESP8266Client1";

// Home router
const char* ssid_home = "MaisonSMT";
const char* password_home = "m3f13t01";

// Alternate router for mobile demos
const char* ssid_mobile = "Xperia S_78d6";
const char* password_mobile = "pobeda0117";

WiFiClient espClient;
PubSubClient client(espClient);

// Datas
float temperature = -1;
float altitude = -1;
int pressure = -1;
uint16_t light = 0;

short loopCnt = 0;
// timing
long lastCheck = 0;

// msg
char msg[50];
char topic[50];

/** flag for saving data */
bool shouldSaveConfig = false;

void info(){
    Serial.println("sensor to mqtt");
    Serial.print("VERSION: ");
    Serial.println(PRJ_VERSION);
    Serial.println();
    Serial.print("Wifi: ");
    Serial.print(ssid_home);
    Serial.print("|");
    Serial.println(ssid_mobile);
    Serial.print("mqtt: ");
    Serial.print(mqtt_server);
    Serial.print(":");
    Serial.println(mqtt_port);
    Serial.print("topics: ");
    Serial.print(topicStatus);
    Serial.print(" , ");
    Serial.println(topicCmd);
    Serial.print("wait (ms)");
    Serial.println(LOOP_WAIT);
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

//save the custom parameters to FS
void doSaveConfig() {
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    // json["blynk_token"] = blynk_token;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
}

void fsInit() {

  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port_config, json["mqtt_port"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
}


WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 120);
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port_config, 5);

void initWifiManager() {


  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  // WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 120);
  // WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port_config, 5);
  // WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 32);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  // wifiManager.setSTAStaticIPConfig(IPAddress(192,168,4,127), IPAddress(192,168,4,1), IPAddress(255,255,255,0));

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  // wifiManager.addParameter(&custom_blynk_token);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  String ssid = "ESP_autoconnect_" + String(ESP.getChipId());
  if (!wifiManager.autoConnect(ssid.c_str(), "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
}

void setup()
{
    Serial.begin(115200);

    fsInit();

    info();

    initWifiManager();
    doSaveConfig();

    //read updated parameters
    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_port_config, custom_mqtt_port.getValue());
    // strcpy(blynk_token, custom_blynk_token.getValue());

    // TODO: convert port

    // network
    // setup_wifi_multi();

    // MQTT
    client.setCallback(callback);
    client.setServer(mqtt_server, mqtt_port);
    reconnect();

    //
    // Display
    //
    display.setBrightness(0x0f);
    // All segments on
    displayAllOn();

    // Sensor
    Serial.println("Setting sensor");
    Wire.pins(IC_DATA, IC_CLK);

    if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP180 sensor, check wiring!");
      // loop will force reboot
      while (1) {}
    }
    Serial.println("Sensor ready");
    displaySensorDetails();

    // première mesure
    Serial.println("Lancement de la première mesure");
    displayAllOff();
    work();

    Serial.println("Type,\tstatus,\tHumidity (%),\tTemperature (C)\tTime (us)");
}


void loop()
{

  // check incomming/connection
  checkClient();

  long now = millis();
  if (now - lastCheck > LOOP_WAIT) {
    lastCheck = now;
    work();
  }
}

// void setup_wifi_multi() {
//     if(!setup_wifi(ssid_mobile, password_mobile)){
//       if(!setup_wifi(ssid_home, password_home)){
//         // loop will force reboot
//         while (1) {}
//       }else {
//         mqtt_server = mqtt_server_home;
//       }
//     }else {
//       mqtt_server = mqtt_server_mobile;
//     }
//     Serial.print("Mqtt server set to ");
//     Serial.println(mqtt_server);
//     client.setServer(mqtt_server, mqtt_port);
// }
//
// boolean setup_wifi(const char* ssid, const char* password) {
//
//   delay(10);
//   // We start by connecting to a WiFi network
//   Serial.println();
//   Serial.print("Connecting to ");
//   Serial.println(ssid);
//
//   WiFi.begin(ssid, password);
//
//   int cpt=0;
//   while (WiFi.status() != WL_CONNECTED) {
//     if(++cpt > 50){
//       return false;
//     }
//     Serial.print(".");
//     delay(200);
//   }
//
//   Serial.println("");
//   Serial.println("WiFi connected");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.localIP());
//   return true;
// }

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void checkClient() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

void reconnect() {
  Serial.println("reconnect");

  //reconnect wifi first
  if(WiFi.status() != WL_CONNECTED){
    ESP.reset();
  }

  // Loop until we're reconnected to MQTT server
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId)) {
      Serial.print("connected with id : ");
      Serial.println(clientId);
      // Once connected, publish an announcement...
      client.publish(topicStatus, "hello world");
      Serial.print("subscribing to : ");
      Serial.print(topicCmd);
      if(client.subscribe(topicCmd)){
        Serial.println(" succeeded");
      }else{
        Serial.println(" failed");
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void work(){
    Serial.println("work");
    readSensor();
    trace();
    displayTemperature();
    sendDatas();
}

char *ftoa(char *a, double f, int precision){
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};

 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 return ret;
}

void sendDatas(){
  sendTemperature(temperature);
  sendPressure(pressure);
  sendLight(light);
}

void sendTemperature(float aTemperature){
  ftoa(msg, aTemperature, 2);
  snprintf (topic, sizeof(topic), "%s/%s", topicStatus, "temperature");
  sendmsgToTopic();
}

void sendPressure(int aPresure){
  snprintf (msg, sizeof(msg), "%d", aPresure);
  snprintf (topic, sizeof(topic), "%s/%s", topicStatus, "pressure");
  sendmsgToTopic();
}

void sendLight(uint16_t aLight){
  snprintf (msg, sizeof(msg), "%d", aLight);
  snprintf (topic, sizeof(topic), "%s/%s", topicStatus, "light");
  sendmsgToTopic();
}

void sendmsgToTopic(){
  Serial.print("Publish on: ");
  Serial.print(topic);
  Serial.print(" message: ");
  Serial.println(msg);
  client.publish(topic, msg);
}

void readSensor(){
    //Serial.println("Read bpm180");
    sensors_event_t event;
    bmp.getEvent(&event);

    // Do we have datas
    if (event.pressure) {
      float myTemperature;
      bmp.getTemperature(&myTemperature);
      if(myTemperature<150){
        // assumed correct read
        temperature = myTemperature;
        pressure = event.pressure;
        altitude = bmp.pressureToAltitude(
          SENSORS_PRESSURE_SEALEVELHPA,
          event.pressure
        );
      }
    }
    //Serial.println("Read bh1750");
    uint16_t myLight = readLight();
    if(myLight>=0){
      // assumed correct read
      light = myLight;
    }
}
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void) {
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("              BMP                   ");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

int BH1750_Read(int address) //
{
  int i=0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()) //
  {
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission();
  return i;
}

void BH1750_Init(int address)
{
  Wire.beginTransmission(address);
  Wire.write(0x10);//1lx reolution 120ms
  Wire.endTransmission();
}

uint16_t readLight(){
  int i;
  uint16_t val=-1;
  BH1750_Init(BH1750address);
  delay(100);

  if(2==BH1750_Read(BH1750address)) {
    val=((buff[0]<<8)|buff[1])/1.2;
    //Serial.print(val,DEC);
    //Serial.println("lx");
  } else {
    Serial.println("can't read light");
  }
  return val;
}

void trace(){
    // DISPLAY DATA
    if(loopCnt++ %20 == 0){
      printHeader();
    }
    printData(temperature, pressure, altitude, light);
}

void printData(float aTemperature, int aPressure, float aAltitude, uint16_t aLight){
  Serial.print(aTemperature);
  Serial.print("\t");
  Serial.print(aPressure);
  Serial.print("\t");
  Serial.print(aAltitude);
  Serial.print("\t");
  Serial.println(aLight);
}

void printHeader(){
  Serial.println("Temperature °C\tPressure\tAltitude\tlight");
}


/**
 * All segments on the display to on
 */
void displayAllOn(){
  // All segments on
  displayAll(0xff);
}

/**
 * All segments on the display to off
 */
void displayAllOff(){
  // All segments off
  displayAll(0);
}

/**
 * Display same pattern on all segments
 */
void displayAll(uint8_t aValue){
  // All segments on the same value
  Serial.print("all to ");
  Serial.println(aValue);
  uint8_t data[] = { aValue, aValue, aValue, aValue };
  display.setSegments(data);
}

/**
 * Display on the embedded hardware
 */
void displayTemperature(){
    //Serial.print("Display temperature : ");
    //Serial.print(temperature, 1);
    //Serial.println("°C");

    display.showNumberDec(temperature * 10, false, 4, 0);
}
