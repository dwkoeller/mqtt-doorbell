//This can be used to output the date the code was compiled
const char compile_date[] = __DATE__ " " __TIME__;

/************ WIFI, OTA and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
//#define WIFI_SSID "" //enter your WIFI SSID
//#define WIFI_PASSWORD "" //enter your WIFI Password
//#define MQTT_SERVER "" // Enter your MQTT server address or IP.
//#define MQTT_USER "" //enter your MQTT username
//#define MQTT_PASSWORD "" //enter your password
#define MQTT_DEVICE "mqtt-doorbell" // Enter your MQTT device
#define MQTT_DEVICE_NAME "Doorbell" // Enter your MQTT device
#define MQTT_PORT 8883 // Enter your MQTT server port.
#define MQTT_SOCKET_TIMEOUT 120
#define FW_UPDATE_INTERVAL_SEC 24*3600
#define STATUS_UPDATE_INTERVAL_SEC 120
#define BELL_UPDATE_INTERVAL_MS 250
#define BELL_DELAY_MS 500
#define UPDATE_SERVER "http://192.168.100.15/firmware/"
#define FIRMWARE_VERSION "-1.00"

/****************************** MQTT TOPICS (change these topics as you wish)  ***************************************/
#define MQTT_HEARTBEAT_SUB "heartbeat/#"
#define MQTT_HEARTBEAT_TOPIC "heartbeat"
#define MQTT_UPDATE_REQUEST "update"
#define MQTT_DISCOVERY_BINARY_SENSOR_PREFIX  "homeassistant/binary_sensor/"
#define MQTT_DISCOVERY_SWITCH_PREFIX  "homeassistant/switch/"
#define MQTT_DISCOVERY_SENSOR_PREFIX  "homeassistant/sensor/"
#define HA_TELEMETRY                         "ha"

#define WATCHDOG_PIN 14     // D5
#define RELAY_PIN 5         // D1
#define BELL_BUTTON_PIN 13  // D7

#define RELAY_ON 1
#define RELAY_OFF 0

#define SWITCH_ON "ON"
#define SWITCH_OFF "OFF"

#include <ESP8266SSDP.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include "credentials.h" // Place credentials for wifi and mqtt in this file
#include "certificates.h" // Place certificates for mqtt in this file

Ticker ticker_fw, tickerBellState, ticker_status;

bool readyForFwUpdate = false;
bool readyForBellUpdate = false;
bool registered = false;
bool bellEnabled = false;

String switchStateTopic = String(MQTT_DISCOVERY_SWITCH_PREFIX) + MQTT_DEVICE + "/state";
String switchCommandTopic = String(MQTT_DISCOVERY_SWITCH_PREFIX) + MQTT_DEVICE + "/command";

String state = "";
String lastState = "";

WiFiClientSecure espClient;
PubSubClient client(espClient);

#include "common.h"
 
void setup() {
 
  Serial.begin(115200);

  pinMode(WATCHDOG_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BELL_BUTTON_PIN, INPUT_PULLUP);

  setup_wifi();
  IPAddress result;
  int err = WiFi.hostByName(MQTT_SERVER, result) ;
  if(err == 1){
        Serial.print("MQTT Server IP address: ");
        Serial.println(result);
        MQTTServerIP = result.toString();
  } else {
        Serial.print("Error code: ");
        Serial.println(err);
  }  
  
  client.setBufferSize(512);
  client.setServer(MQTT_SERVER, MQTT_PORT); //1883 is the port number you have forwared for mqtt messages. You will need to change this if you've used a different port 
  client.setCallback(callback); //callback is the function that gets called for a topic sub

  ticker_fw.attach_ms(FW_UPDATE_INTERVAL_SEC * 1000, fwTicker);
  tickerBellState.attach_ms(BELL_UPDATE_INTERVAL_MS, bellStateTickerFunc);

  digitalWrite(RELAY_PIN, RELAY_OFF);
  digitalWrite(WATCHDOG_PIN, LOW);

  ticker_status.attach_ms(STATUS_UPDATE_INTERVAL_SEC * 1000, statusTicker);

  checkForUpdates();
  resetWatchdog();
}

void loop() {
  
  //If MQTT client can't connect to broker, then reconnect
  if (!client.connected()) {
    reconnect();
    client.subscribe(switchCommandTopic.c_str());
  }

  if(readyForBellUpdate) {
    readyForBellUpdate = false;
    checkBellState();
  }

  if(readyForFwUpdate) {
    readyForFwUpdate = false;
    checkForUpdates();
  }

  client.loop(); //the mqtt function that processes MQTT messages
  if (! registered) {
    registerTelemetry();
    updateTelemetry("Unknown");
    createBinarySensors(MQTT_DEVICE, MQTT_DEVICE_NAME);
    createSwitch(MQTT_DEVICE, MQTT_DEVICE_NAME);
    registered = true;
  }

}

void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  String strTopic;
  String payload;

  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }

  strTopic = String((char*)p_topic);
  if (strTopic == MQTT_HEARTBEAT_TOPIC) {
    resetWatchdog();
    updateTelemetry(payload);      
    if (payload.equals(String(MQTT_UPDATE_REQUEST))) {
      checkForUpdates();
    }
    return;    
  }
  if (payload.equals(String(SWITCH_ON))) {
    updateSwitch(MQTT_DEVICE, SWITCH_ON);
    bellEnabled = true;
  }
  else if (payload.equals(String(SWITCH_OFF))) {
    updateSwitch(MQTT_DEVICE, SWITCH_OFF);
    bellEnabled = false;
  }  
}

void checkBellState() {
 
  state = getCurrentState(BELL_BUTTON_PIN);
  if(state != lastState) {
    lastState = state;
    if(state == "ON") {
      updateBinarySensor(MQTT_DEVICE, "ON");
      ringBell();
    }
    else {
      updateBinarySensor(MQTT_DEVICE, "OFF");    
    }
  }
}

String getCurrentState(int pin) {
  String state;
  int val;
  val = digitalRead(pin);
  if(val == LOW) {
    state = "ON";
    client.publish("debug", "ON", true);
  }
  else {
    state = "OFF";    
    client.publish("debug", "OFF", true);

  }
  return state;
}

void bellStateTickerFunc() {
  readyForBellUpdate = true;
}

void createBinarySensors(String sensor, String sensor_name) {
  String topic = String(MQTT_DISCOVERY_BINARY_SENSOR_PREFIX) + sensor + "/config";
  String message = String("{\"name\": \"") + sensor_name +
                   String("\", \"unique_id\": \"") + sensor_name + getUUID() +
                   String("\", \"state_topic\": \"") + String(MQTT_DISCOVERY_BINARY_SENSOR_PREFIX) + sensor +
                   String("/state\", \"device_class\": \"door\"}");
  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(message.c_str());

  client.publish(topic.c_str(), message.c_str(), true);  

}

void updateBinarySensor(String sensor, String state) {
  String topic = String(MQTT_DISCOVERY_BINARY_SENSOR_PREFIX) + sensor + "/state";
  
  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(state);
  client.publish(topic.c_str(), state.c_str(), true);

}

void ringBell() {
  if (bellEnabled) {
    digitalWrite(RELAY_PIN, LOW);
    delay(500);
    digitalWrite(RELAY_PIN, HIGH);
  }
}

void createSwitch(String switch_device, String switch_name) {
  String topic = String(MQTT_DISCOVERY_SWITCH_PREFIX) + switch_device + "/config";
  String message = String("{\"name\": \"") + switch_name +
                   String("\", \"retain\": \"true") +
                   String("\", \"unique_id\": \"") + switch_device + getUUID() +
                   String("\", \"optimistic\": \"false") +
                   String("\", \"command_topic\": \"") + String(MQTT_DISCOVERY_SWITCH_PREFIX) + switch_device +
                   String("/command\", \"state_topic\": \"") + String(MQTT_DISCOVERY_SWITCH_PREFIX) + switch_device +
                   String("/state\"}");
  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(message.c_str());

  client.publish(topic.c_str(), message.c_str(), true);

}

void updateSwitch(String switch_device, String state) {
  String topic = String(MQTT_DISCOVERY_SWITCH_PREFIX) + switch_device + "/state";

  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(state);
  client.publish(topic.c_str(), state.c_str(), true);

}

void statusTicker() {
  String status;
  if (bellEnabled) {
    status = "ON";
  }
  else {
    status = "OFF";
  }
  updateSwitch(MQTT_DEVICE, status.c_str());
}
