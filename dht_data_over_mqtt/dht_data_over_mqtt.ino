#include "DHT.h"
#include "NewPing.h"
#include "WifiCreds"
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

// MQTT broker
// #define MQTT_HOST IPAddress(192, 168, 2, 57) // Ubuntu mini
#define MQTT_HOST IPAddress(192, 168, 2, 11) // Macbook
#define MQTT_PORT 1883

// Temperature topics
#define MQTT_PUB_TEMP "sneferu/dht/temperature"
#define MQTT_PUB_HUM "sneferu/dht/humidity"

// Moisture topics
#define MQTT_PUB_MOIS "sneferu/moisture"

// Distance topics
#define MQTT_PUB_DIST_CM "sneferu/sr04/cm"
#define MQTT_PUB_DIST_IN "sneferu/sr04/in"

// DHT data
#define DHTPIN 14
#define DHTTYPE DHT11

// Moisture data pin
#define AOUT_PIN A0

// SR04 data
#define TRIG_PIN 12
#define ECHO_PIN 13

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Initialize new PING
NewPing sonar(TRIG_PIN, ECHO_PIN);

// Sensor readings variables
float temp;
float hum;

// Moisture extremes
const int dry = 845;
const int wet = 385;

// Declare MQTT client
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

// Declare wifi handlers
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

unsigned long previousMillis = 0;
unsigned long previousSr04Millis = 0;
const long interval = 2000;
const long sr04Interval = 0.01;

void connectToWifi() {
  Serial.print("Connecting to wifi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to wifi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // Don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  // Put your setup code here, to run once:
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(115200);

  dht.begin();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}

void loop() {
  // Put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();

  // Publish a new mqtt message for temp, hum & mois
  if (currentMillis - previousMillis >= interval) {
    // Last time a reading was taken
    previousMillis = currentMillis;

    // Read humidity
    hum = dht.readHumidity();
    // Read temperature in celcius
    temp = dht.readTemperature();
    
    // Read moisture
    int moisture_value = analogRead(AOUT_PIN);
    int moisture_percentage = map(moisture_value, wet, dry, 100, 0);

    // Publish mqtt message to MQTT_PUB_TEMP
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %.2f \n", temp);

    // Publish mqtt message to MQTT_PUB_HUM
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
    Serial.printf("Message: %.2f \n", hum);

    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_MOIS, 1, true, String(moisture_percentage).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i: ", MQTT_PUB_MOIS, packetIdPub3);
    Serial.printf("Message: %.2f \n", moisture_percentage);
  }

  if (currentMillis - previousSr04Millis >= sr04Interval) {
    previousSr04Millis = currentMillis;

    // Read distances
    float distance_cm = sonar.ping_cm();
    float distance_in = sonar.ping_in();

    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_DIST_CM, 1, true, String(distance_cm).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i: ", MQTT_PUB_DIST_CM, packetIdPub4);
    Serial.printf("Message: %.2f \n", distance_cm);

    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_DIST_IN, 1, true, String(distance_in).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i: ", MQTT_PUB_DIST_IN, packetIdPub5);
    Serial.printf("Message: %.2f \n", distance_in);
  }
}
