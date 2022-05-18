#include <Arduino.h>
#include <math.h>

#ifdef U8X8_HAVE_HW_SPI
  #include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
  #include <Wire.h>
#endif

#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <ESP32Servo.h> // ESP32Servo library installed by Library Manager
#include "ESC.h" // RC_ESP library installed by Library Manager

#define ESC_PIN (33) // connected to ESC control wire
#define LED_BUILTIN (2) // not defaulted properly for ESP32s/you must define it
#define POT_PIN (34) // Analog pin used to connect the potentiometer center pin

// Note: the following speeds may need to be modified for your particular hardware.
#define MIN_SPEED 1040 // speed just slow enough to turn motor off
#define MAX_SPEED 1240 // speed where my motor drew 3.6 amps at 12v.


#define WIFI_SSID "LEO1_TEAM_03"
#define WIFI_PASSWORD "gruppe03"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 10, 1)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// Joydata MQTT Topics
#define MQTT_PUB_JOY "/joydata" //"esp32/dht/joystick"
ESC myESC (ESC_PIN, 1000, 2000, 500); 
long int val;


// values from mqtt
float DualAxisXVal = 0;
int tempVal;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 1000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
  mqttClient.subscribe(MQTT_PUB_JOY, 1); // test topic to on/off blue light
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  
String messageTemp;
 
for (int i = 0; i < len; i++) {
 messageTemp += (char)payload[i];
}
 
 // Check if the MQTT message was received on topic esp32/relay1
if(String(topic) == MQTT_PUB_JOY){
  Serial.println("this is Joy data");
  Serial.println(messageTemp);
  tempVal = messageTemp.toInt();
  Serial.println("Topic is Joydata");
}
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

  pinMode(ESC_PIN, OUTPUT);
pinMode(LED_BUILTIN, OUTPUT);
digitalWrite(LED_BUILTIN, HIGH); // set led to on to indicate arming
myESC.arm(); // Send the Arm command to ESC
delay(5000); // Wait a while
digitalWrite(LED_BUILTIN, LOW); // led off to indicate arming completed

// the following loop turns on the motor slowly, so get ready
for (int i=0; i<350; i++){ // run speed from 840 to 1190
myESC.speed(MIN_SPEED-200+i); // motor starts up about half way through loop
delay(10);
}
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Every X number of seconds (interval = 10 seconds) it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;

   

    // Subscripe an MQTT message on topic esp32/dht/Joydata
    uint16_t packetIdSub = mqttClient.subscribe(MQTT_PUB_JOY, 2);
    Serial.print("Subscribing at QoS 2, packetId: ");
    Serial.println(packetIdSub);

    
  }
Serial.println(val);
Serial.println(tempVal);
val = map(tempVal, 0, 4095, MIN_SPEED, MAX_SPEED); // scale pot reading to valid speed range
myESC.speed(val); // sets the ESC speed
delay(10); // Wait for a while
  

}
