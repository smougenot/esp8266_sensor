// BMP180
#include <Wire.h>
#include <Adafruit_BMP085.h>
// Display
#include <TM1637Display.h>
// send messages
#include <ESP8266WiFi.h>
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

Adafruit_BMP085 bmp;

// MQTT
#define CLIENT_ID     "1"
const char* mqtt_server;
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
    Serial.println(mqtt_server);
    Serial.print("topics: ");
    Serial.print(topicStatus);
    Serial.print(" , ");
    Serial.println(topicCmd);
    Serial.print("wait (ms)");
    Serial.println(LOOP_WAIT);
}

void setup()
{
    Serial.begin(115200);
    info();

    // network
    setup_wifi_multi();

    client.setCallback(callback);
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

void setup_wifi_multi() {
    if(!setup_wifi(ssid_mobile, password_mobile)){
      if(!setup_wifi(ssid_home, password_home)){
        // loop will force reboot
        while (1) {}
      }else {
        mqtt_server = mqtt_server_home;
      }
    }else {
      mqtt_server = mqtt_server_mobile;
    }
    Serial.print("Mqtt server set to ");
    Serial.println(mqtt_server);
    client.setServer(mqtt_server, 1883);
}

boolean setup_wifi(const char* ssid, const char* password) {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int cpt=0;
  while (WiFi.status() != WL_CONNECTED) {
    if(++cpt > 50){
      return false;
    }
    Serial.print(".");
    delay(200);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
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
    setup_wifi_multi();
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
    float myTemperature = bmp.readTemperature();
    if(myTemperature<150){
      // assumed correct read
      temperature = myTemperature;
      pressure = bmp.readPressure();
      altitude = bmp.readAltitude();
    }
    //Serial.println("Read bh1750");
    uint16_t myLight = readLight();
    if(myLight>=0){
      // assumed correct read
      light = myLight;
    }
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
