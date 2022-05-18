
#include "DHT.h"
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID "LEO1_TEAM_03"
#define WIFI_PASSWORD "gruppe03"

// Raspberry Pi Mosquitto MQTT Broker

#define MQTT_HOST IPAddress(192, 168, 10, 1)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

#define MQTT_PUB_PINX "/joydata"

const int pinSW = 2;
const int pinX = 35;
const int pinY = 32;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
  mqttClient.subscribe("/blue_light", 1); // test topic to on/off blue light
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
 String messageTemp;
 for (int i = 0; i < len; i++) {
  //Serial.print((char)payload[i]);
  messageTemp += (char)payload[i];
 }
 
 Serial.println("Publish received.");
 Serial.print(" message: ");
 Serial.println(messageTemp);
 Serial.print(" topic: ");
 Serial.println(topic);
 Serial.print(" qos: ");
 Serial.println(properties.qos);
 Serial.print(" dup: ");
 Serial.println(properties.dup);
 Serial.print(" retain: ");
 Serial.println(properties.retain);
 Serial.print(" len: ");
 Serial.println(len);
 Serial.print(" index: ");
 Serial.println(index);
 Serial.print(" total: ");
 Serial.println(total);
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(pinSW, INPUT_PULLUP);
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials("pi", "gruppe03");
  connectToWifi();
}

void loop() {
  int 
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
        
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_PINX, 1, true, String(analogRead(34)).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_PINX, packetIdPub3);
    Serial.printf("Message: %.2f \n", pinX);

    Serial.println("pinX");
    Serial.println(analogRead(pinX));
    Serial.println("pinY");
    Serial.println(analogRead(pinY));
    Serial.println("Switch");
    Serial.println(digitalRead(pinSW));

  }
  }


/*
#include <Adafruit_MCP3008.h>
#include <SPI.h>
const int pinSW = 2;
const int pinX = 35;
const int pinY = 32;
void setup(){
  pinMode(pinSW, INPUT_PULLUP);
  Serial.begin(9600);
  delay(1000);
}
//void loop(){
//  Serial.println("pinX");
//  Serial.println(analogRead(pinX));
//  Serial.println("pinY");
//  Serial.println(analogRead(pinY));
//  Serial.println("Switch");
//  Serial.println(digitalRead(pinSW));
// delay(1000);
}*/
