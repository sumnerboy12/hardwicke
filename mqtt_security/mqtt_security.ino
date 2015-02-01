#include <avr/wdt.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#include "mqtt_security.h"

// serial rate
#define SERIAL_BAUD     9600

// ethernet MAC address (must be unique on the LAN)
byte mac[]              = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x50 };
byte ip[]               = { 192, 168, 1, 80 };

// MQTT broker - subscribe to <topic>/in/+, publish to <topic>/out/+
byte mqttBroker[]       = { 192, 168, 1, 21 };
char mqttClientId[]     = "security";
char mqttUsername[]     = "security";
char mqttPassword[]     = "password";
char mqttTopic[]        = "/openhab/security";
char mqttLwtTopic[]     = "/clients/security";
int  mqttLwtQos         = 0;
int  mqttLwtRetain      = 1;

// NOTE: pins 0 & 1 are used for serial RX/TX
// NOTE: ethernet shield uses pins 4 for SD card and 10-13 for SPI
Pin  monitorPins[5] = { 
  { 2, -1, -1, 0 }, 
  { 3, -1, -1, 0 }, 
  { 5, -1, -1, 0 }, 
  { 7, -1, -1, 0 }, 
  { 9, -1, -1, 0 } 
};
int monitorPinCount = 5;

// debouncing (ms)
long debounceTime = 100;

EthernetClient ethernet;
PubSubClient mqtt(mqttBroker, 1883, mqtt_callback, ethernet);

void setup() {
  // ensure the watchdog is disabled
  wdt_disable();

  // initialise the serial interface  
  Serial.begin(SERIAL_BAUD);
  
  // dump the sketch details
  Serial.println("---------------------------------------");
  Serial.println("Sketch ID:            mqtt_security.ino");
  Serial.println("---------------------------------------");
  Serial.println();

  // configure our monitor pins
  for (int i = 0; i < monitorPinCount; i++) {
    pinMode(monitorPins[i].pin, INPUT_PULLUP); 
  }
  
  // initialise the ethernet module with our MAC and IP addresses
  Serial.print("Initialising ethernet connection...");
  Ethernet.begin(mac, ip);
  Serial.println("done");
  
  // attempt to connect to our MQTT broker
  mqttConnect();
  
  // enable the watchdog timer - 8s timeout
  wdt_enable(WDTO_8S);
  wdt_reset();
  
  Serial.println("Initialisation complete");
}

void loop() {
  // reset the watchdog timer
  wdt_reset();
  
  // check our DHCP lease is still ok
  Ethernet.maintain();

  // process any MQTT messages - will return false if not connected
  // so attempt to reconnect if required
  if (mqtt.loop() || mqttConnect()) {  
    // check our monitor pins and publish any changes
    for (int i = 0; i < monitorPinCount; i++) {
      readDigital(monitorPins[i]);
    }
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // tokenise the topic
  char * topictoken;
  topictoken = strtok(topic, "/");

  // junk the first three tokens - i.e. /openhab/security/in/...
  topictoken = strtok(NULL, "/");
  topictoken = strtok(NULL, "/");
  topictoken = strtok(NULL, "/");
  
  // check for the special 'refresh' token
  if (strcmp(topictoken, "refresh") == 0) {
    // debug
    Serial.println("Received 'refresh' request");
    // send MQTT updates for all our monitor pins
    refresh();
  } else {
    // get the pin from the last token
    int pin = strtol(topictoken, NULL, 10);
    // debug
    Serial.print("Received state change request for pin ");
    Serial.print(pin);
    Serial.print(" -> ");
    Serial.println(*payload == '0' ? "OFF" : "ON");
    // make sure the pin is configured correctly
    pinMode(pin, OUTPUT);
    // update the pin state (HIGH is off, LOW is on)
    digitalWrite(pin, *payload == '0' ? HIGH : LOW); 
  }
}

boolean mqttConnect() {
  Serial.print("Attempting to connect to MQTT broker as ");
  Serial.print(mqttUsername);
  Serial.print("...");
  
  // attempt to connect to the broker and setup our subscriptions etc
  boolean success = mqtt.connect(mqttClientId, mqttUsername, mqttPassword, mqttLwtTopic, mqttLwtQos, mqttLwtRetain, "0");
  if (success) {
    Serial.println("success");
    // subscribe to our 'in' topic
    mqtt.subscribe(getInTopic());
    // publish retained LWT so anything listening knows we are alive
    byte data[] = { '1' };
    mqtt.publish(mqttLwtTopic, data, 1, mqttLwtRetain);
  } else {
    Serial.println("failed");
  }
  return success; 
}

void refresh() {
  for (int i = 0; i < monitorPinCount; i++) {
    Pin pin = monitorPins[i];
    byte state = digitalRead(pin.pin);
    
    publishState(pin.pin, state);
    pin.published = state;
  }
}

void readDigital(Pin& pin) {
  byte oldState = pin.state;
  byte newState = digitalRead(pin.pin);
  
  if (oldState != newState) {
    // reset the debounce timer
    pin.time = millis();
  }
  
  if ((millis() - pin.time) > debounceTime) {
    // the pin has been stable so check if we need to publish
    if (pin.published != newState) {
        publishState(pin.pin, newState);
        pin.published = newState;
    }
  }

  // update our pin state
  pin.state = newState;
}

void publishState(byte pin, byte state) {
  static byte payload[1];
  if (state == 0) {
    payload[0] = '1';
  } else {
    payload[0] = '0';
  }
  // debug
  Serial.print("Publishing state for pin ");
  Serial.print(pin);
  Serial.print(" -> ");
  Serial.println(payload[0]);
  // payloadlength=1, retain=0
  mqtt.publish(getOutTopic(pin), payload, 1, 0);
}

char * getInTopic() {
  static char topic[32];
  snprintf(topic, 32, "%s/in/+", mqttTopic);
  return topic;
}

char * getOutTopic(int pin) {
  static char topic[32];
  snprintf(topic, 32, "%s/out/%i", mqttTopic, pin);
  return topic;
}

