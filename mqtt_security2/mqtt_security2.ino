/***************************************************************************
* File Name: mqtt_security2.ino
* Processor/Platform: Arduino Uno R3 (tested)
* Development Environment: Arduino 1.6.1
*
* Description: standard IO monitoring/control (via relay board)
***************************************************************************/
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

// serial rate
#define SERIAL_BAUD          9600

// number of digital IO pins we are monitoring
#define MONITOR_PIN_COUNT    5
#define MONITOR_PIN_DEBOUNCE 100

// custom structs
typedef struct 
{
  byte pin;
  byte state;
  byte published;
  unsigned long time;
} Pin;

void readDigitalPin(Pin& pin);

// NOTE: pins 0 & 1 are used for serial RX/TX
// NOTE: ethernet shield uses pins 4 for SD card and 10-13 for SPI
Pin  monitorPins[MONITOR_PIN_COUNT] = { 
  { 2, -1, -1, 0L }, 
  { 3, -1, -1, 0L }, 
  { 5, -1, -1, 0L }, 
  { 7, -1, -1, 0L }, 
  { 9, -1, -1, 0L } 
};

// ethernet MAC address (must be unique on the LAN)
byte mac[]              = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x50 };
byte ip[]               = { 192, 168, 1, 80 };

// MQTT broker - subscribe to <topic>/in/+, publish to <topic>/out/+
byte mqttBroker[]       = { 192, 168, 1, 21 };
char mqttClientId[]     = "security";
char mqttUsername[]     = "security";
char mqttPassword[]     = "password";
char mqttOpenhabTopic[] = "/openhab/security";
char mqttLwtTopic[]     = "/clients/security";
int  mqttLwtQos         = 0;
int  mqttLwtRetain      = 1;

// ethernet and MQTT clients
EthernetClient ethernet;
PubSubClient mqtt(mqttBroker, 1883, mqtt_callback, ethernet);

void setup() 
{
  // initialise the serial interface  
  Serial.begin(SERIAL_BAUD);
  
  // dump the sketch details
  Serial.println("---------------------------------------");
  Serial.println("Sketch ID:           mqtt_security2.ino");
  Serial.println("---------------------------------------");

  // configure our monitor pins
  Serial.print("Initialising digital monitor pins...");
  for (int i = 0; i < MONITOR_PIN_COUNT; i++) 
  {
    Serial.print("pin ");
    Serial.print(monitorPins[i].pin);
    Serial.print("...");
    pinMode(monitorPins[i].pin, INPUT_PULLUP); 
  }
  Serial.println("done");
      
  // initialise the ethernet module with our MAC and IP addresses
  Serial.print("Initialising ethernet connection...");
  Ethernet.begin(mac, ip);
  Serial.println("done");
  
  // attempt to connect to our MQTT broker
  mqttConnect();

  Serial.println("Initialisation complete");
  Serial.println();
}

void loop() 
{
  // check our DHCP lease is still ok
  Ethernet.maintain();

  // process any MQTT messages - will return false if not connected
  // so attempt to reconnect if required
  if (mqtt.loop() || mqttConnect()) {  
    // check our monitor pins and publish any changes
    for (int i = 0; i < MONITOR_PIN_COUNT; i++) {
      readDigitalPin(monitorPins[i]);
    }
  }
}

/*
** MQTT callback handler - for receiving MQTT publishes and 
** switching digital pins (or handling a refresh)
*/
void mqtt_callback(char * topic, byte * payload, unsigned int length) 
{
  // tokenise the topic
  char * topictoken;
  topictoken = strtok(topic, "/");
  
  // junk the first three tokens - i.e. /openhab/security/in/...
  topictoken = strtok(NULL, "/");
  topictoken = strtok(NULL, "/");
  topictoken = strtok(NULL, "/");
  
  // check for the special 'refresh' token
  if (strcmp(topictoken, "refresh") == 0) {
    // send MQTT updates for all our monitor pins
    refresh();
  } else {
    // get the pin from the last token
    int pin = strtol(topictoken, NULL, 10);
    // make sure the pin is configured correctly
    pinMode(pin, OUTPUT);
    // update the pin state (HIGH is off, LOW is on)
    digitalWrite(pin, *payload == '0' ? HIGH : LOW); 
  }
}

boolean mqttConnect() 
{
  Serial.print("Attempting to connect to MQTT broker as ");
  Serial.print(mqttUsername);
  Serial.print("...");
  
  // attempt to connect to the broker and setup our subscriptions etc
  boolean success = mqtt.connect(mqttClientId, mqttUsername, mqttPassword, mqttLwtTopic, mqttLwtQos, mqttLwtRetain, "0");
  if (success) {
    Serial.println("success");
    // subscribe to our security 'in' topic
    mqtt.subscribe(getOpenhabTopic("in/+"));
    // publish retained LWT so anything listening knows we are alive
    byte data[] = { '1' };
    mqtt.publish(mqttLwtTopic, data, 1, mqttLwtRetain);
  } else {
    Serial.println("failed");
  }
  return success; 
}

/*
** Force publish the current state of all digital monitor pins
*/
void refresh() 
{
  for (int i = 0; i < MONITOR_PIN_COUNT; i++) {
    Pin pin = monitorPins[i];
    byte state = digitalRead(pin.pin);
    
    publishPinState(pin.pin, state);
    pin.published = state;
  }
}

/*
** Digital IO pin monitoring/publishing methods
*/
void readDigitalPin(Pin& pin) 
{
  byte oldState = pin.state;
  byte newState = digitalRead(pin.pin);
  
  if (oldState != newState) {
    // reset the debounce timer
    pin.time = millis();
  }
  
  if ((millis() - pin.time) > MONITOR_PIN_DEBOUNCE) {
    // the pin has been stable so check if we need to publish
    if (pin.published != newState) {
        publishPinState(pin.pin, newState);
        pin.published = newState;
    }
  }

  // update our pin state
  pin.state = newState;
}

void publishPinState(byte pin, byte state) 
{
  // build the sub-topic
  static char subTopic[8]; 
  snprintf(subTopic, 8, "out/%i", pin);

  // convert to payload
  static char payload[1];
  if (state) {
    payload[0] = '0';
  } else {
    payload[0] = '1';
  }
  
  // publish
  mqtt.publish(getOpenhabTopic(subTopic), (byte *)payload, sizeof(payload), 0);
}

/*
** MQTT topic builders
*/
char * getOpenhabTopic(char * subTopic) 
{
  static char topic[32];
  snprintf(topic, 32, "%s/%s", mqttOpenhabTopic, subTopic);
  return topic;
}

