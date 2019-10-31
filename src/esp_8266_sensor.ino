//needed for library
#include <DNSServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <EEPROM.h>

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

// I2C sensors
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_VEML6070.h>
// Display
#include <TM1637Display.h>
// send MQTT messages
#include <PubSubClient.h>

#define PRJ_VERSION 4

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

Adafruit_BME280 bme;
boolean bme_available=false;

// VEML6070
Adafruit_VEML6070 uv = Adafruit_VEML6070();

// MQTT Settings
#define EEPROM_SALT 311276 // to check version
typedef struct {
  char mqtt_server[120]   = "";
  char mqtt_port[6]       = "1883";
  char mqtt_user[120]     = "";
  char mqtt_password[120] = "";
  int  salt               = EEPROM_SALT;
} WMSettings;

WMSettings settings;

const String topicCmd    = "esp/" + String(ESP.getChipId()) + "/cmd";
const String topicStatus = "esp/" + String(ESP.getChipId()) + "/status";
const String clientId = "ESP8266Client" + String(ESP.getChipId()) ;

WiFiClient espClient;
PubSubClient client(espClient);

// Datas
float temperature = -1;
float altitude = -1;
int pressure = -1;
uint16_t light = 0;
uint16_t uvLight = 0;
float humidity = -1;

short loopCnt = 0;
// timing
long lastCheck = 0;

// msg
char msg[50];
char topic[50];

/** flag for saving data */
bool shouldSaveConfig = false;
// The extra parameters to be configured (can be either global or just in the setup)
// After connecting, parameter.getValue() will get you the configured value
// id/name placeholder/prompt default length

WiFiManagerParameter custom_mqtt_server("server", "mqtt server", settings.mqtt_server, 120);
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", settings.mqtt_port, 6);
WiFiManagerParameter custom_mqtt_user("user", "mqtt user", settings.mqtt_user, 120);
WiFiManagerParameter custom_mqtt_password("password", "mqtt password", settings.mqtt_password, 120);

void info(){
    Serial.println("sensor to mqtt");
    Serial.print("VERSION: ");
    Serial.println(PRJ_VERSION);
    Serial.println();
    Serial.print("mqtt: ");
    Serial.print(settings.mqtt_server);
    Serial.print(":");
    Serial.println(settings.mqtt_port);
    Serial.print("mqtt user: ");
    Serial.print(settings.mqtt_user);
    Serial.print("/");
    Serial.println(settings.mqtt_password);
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

    strcpy(settings.mqtt_server, custom_mqtt_server.getValue());
    strcpy(settings.mqtt_port, custom_mqtt_port.getValue());
    strcpy(settings.mqtt_user, custom_mqtt_user.getValue());
    strcpy(settings.mqtt_password, custom_mqtt_password.getValue());

    EEPROM.begin(512);
    EEPROM.put(0, settings);
    EEPROM.end();

    //end save
  }else {
    Serial.println("do not save config");
  }
}

void configInit() {
  Serial.println("configInit");

  EEPROM.begin(512);
  EEPROM.get(0, settings);
  EEPROM.end();
}

void resetConfig() {
  Serial.println("Config : reset");
  WMSettings blank;
  settings = blank;
  // Forget Wifi setup
  WiFi.disconnect(true);
}

void initWifiManager() {
  Serial.println("initWifiManager");
  info();
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
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_password);

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
    Serial.println("Setup");

    // Setup
    configInit();
    initWifiManager();
    doSaveConfig();
    info();

    // MQTT
    client.setCallback(callback);
    client.setServer(settings.mqtt_server, String(settings.mqtt_port).toInt());
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

    // pass in the integration time constant
    // higher increase accuracy but takes more time
    uv.begin(VEML6070_2_T);

    bme_available=bme.begin();
    if (!bme_available) {
      Serial.println("Could not find a valid BMP180 sensor, check wiring!");
      // reboot
      ESP.restart();
    }

    Serial.println("Sensor ready");
    displaySensorDetails();

    // première mesure
    Serial.println("Starting first sensor reading");
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

  int cpt=5;

  // Loop until we're reconnected to MQTT server
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId.c_str(), settings.mqtt_user, settings.mqtt_password)) {
      Serial.print("connected with id : ");
      Serial.println(clientId);
      // Once connected, publish an announcement...
      client.publish(topicStatus.c_str(), "hello world");
      Serial.print("subscribing to : ");
      Serial.print(topicCmd);
      if(client.subscribe(topicCmd.c_str())){
        Serial.println(" succeeded");
      }else{
        Serial.println(" failed");
      }
    } else {

      if(--cpt==0){
        resetConfig();
        ESP.reset();
        return;
      }

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
  sendUVLight(uvLight);
  sendHumidity(humidity);
}

void sendTemperature(float aTemperature){
  ftoa(msg, aTemperature, 2);
  snprintf (topic, sizeof(topic), "%s/%s", topicStatus.c_str(), "temperature");
  sendmsgToTopic();
}

void sendPressure(int aPresure){
  snprintf (msg, sizeof(msg), "%d", aPresure);
  snprintf (topic, sizeof(topic), "%s/%s", topicStatus.c_str(), "pressure");
  sendmsgToTopic();
}

void sendLight(uint16_t aLight){
  snprintf (msg, sizeof(msg), "%d", aLight);
  snprintf (topic, sizeof(topic), "%s/%s", topicStatus.c_str(), "light");
  sendmsgToTopic();
}

void sendUVLight(uint16_t aUvLight){
  snprintf (msg, sizeof(msg), "%d", aUvLight);
  snprintf (topic, sizeof(topic), "%s/%s", topicStatus.c_str(), "UV");
  sendmsgToTopic();
}

void sendHumidity(float aHumidity){
  ftoa(msg, aHumidity, 2);
  snprintf (topic, sizeof(topic), "%s/%s", topicStatus.c_str(), "humidity");
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
    readAtmosphere();
    //Serial.println("Read bh1750");
    uint16_t myLight = readLight();
    if(myLight>=0){
      // assumed correct read
      light = myLight;
    }
    //Serial.println("Read VEML6070");
    uint16_t myUvLight = readUvLight();
    if(myUvLight>=0){
      // assumed correct read
      uvLight = myUvLight;
    }
}
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void) {
  Serial.println("------------------------------------");
  Serial.println("              BME                   ");
  Serial.print  ("Unique ID:    "); Serial.println(bme.sensorID());
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void readAtmosphere() {
    //Serial.println("Read bpm180");
    if(!bme_available){
      return;
    }

    // assumed correct read
    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100;
    altitude = bme.readAltitude(
      SENSORS_PRESSURE_SEALEVELHPA
    );
    humidity = bme.readHumidity();
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

uint16_t readUvLight(){
  return uv.readUV();
}

// float readHumidity(){
//   return SHT2x.GetHumidity();
// }

void trace(){
    // DISPLAY DATA
    if(loopCnt++ %20 == 0){
      printHeader();
    }
    printData(temperature, pressure, altitude, light, uvLight, humidity);
}

void printData(float aTemperature, int aPressure, float aAltitude, uint16_t aLight, uint16_t aUvLight, float aHumidity){
  Serial.print(aTemperature);
  Serial.print("\t");
  Serial.print(aPressure);
  Serial.print("\t");
  Serial.print(aAltitude);
  Serial.print("\t");
  Serial.print(aLight);
  Serial.print("\t");
  Serial.print(aUvLight);
  Serial.print("\t");
  Serial.println(aHumidity);
}

void printHeader(){
  Serial.println("Temperature °C\tPressure\tAltitude\tLight\tUV\tHumidity");
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
