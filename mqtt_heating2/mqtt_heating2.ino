/***************************************************************************
* File Name: mqtt_heating2.ino
* Processor/Platform: Arduino Uno R3 (tested)
* Development Environment: Arduino 1.0.6
***************************************************************************/
#include <avr/wdt.h>
#include <math.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#include "mqtt_heating2.h"

// serial rate
#define SERIAL_BAUD      9600

// analog pins patched to NTC 10K thermistors
#define TOP_V_PIN        A3
#define MID_V_PIN        A4
#define BOT_V_PIN        A5

// number of samples to read
#define SAMPLE_WASTAGE   3
#define SAMPLE_COUNT     12
#define SENSOR_ERROR	 -1

// tx at at most every 5s, and at least every 60s
#define MIN_TX_SILENCE   5000
#define MAX_TX_SILENCE   60000
#define PUBLISH_DELTA    0.5

// NTC 10K thermistor
double R1                = 10000.0;
double V_REF             = 4.91;

// Steinhart coefficients
double A                 = 1.129148e-3;
double B                 = 2.34125e-4;
double C                 = 8.76741e-8;
double K                 = 9.5;

// current values, so we can detect changes
double currentTop, currentMid, currentBot;
unsigned long lastTxTop, lastTxMid, lastTxBot;

// ethernet MAC address (must be unique on the LAN)
byte mac[]               = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x51 };
byte ip[]                = { 192, 168, 1, 81 };

// MQTT broker - subscribe to <topic>/in/+, publish to <topic>/out/+
byte mqttBroker[]        = { 192, 168, 1, 21 };
char mqttClientId[]      = "heating";
char mqttUsername[]      = "heating";
char mqttPassword[]      = "password";
char mqttOpenhabTopic[]  = "/openhab/heating";
char mqttHwcTopic[]      = "/heating/hwc";
char mqttLwtTopic[]      = "/clients/heating";
int  mqttLwtQos          = 0;
int  mqttLwtRetain       = 1;

// NOTE: pins 0 & 1 are used for serial RX/TX
// NOTE: ethernet shield uses pins 4 for SD card and 10-13 for SPI
Pin monitorPins[2] = { 
  { A0, -1, -1, 0 }, 
  { A1, -1, -1, 0 } 
};
int monitorPinCount = 2;

// debouncing (ms)
long debounceTime = 100;

// ethernet and MQTT clients
EthernetClient ethernet;
PubSubClient mqtt(mqttBroker, 1883, mqtt_callback, ethernet);

void setup()
{
  // ensure the watchdog is disabled
  wdt_disable();

  // initialise the serial interface  
  Serial.begin(SERIAL_BAUD);
  
  // dump the sketch details
  Serial.println("---------------------------------------");
  Serial.println("Sketch ID:            mqtt_heating2.ino");
  Serial.println("---------------------------------------");

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

  // setup our ADC to use the external voltage reference
  // (we connect the solar controller VCC to AREF)
  analogReference(EXTERNAL);
  
  // enable the watchdog timer - 8s timeout
  wdt_enable(WDTO_8S);
  wdt_reset();
  
  Serial.println("Initialisation complete");
  Serial.println();
}

void loop()
{
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
    
    // read sensors
    double newTop = readSensor(TOP_V_PIN);
    double newMid = readSensor(MID_V_PIN);
    double newBot = readSensor(BOT_V_PIN);

    // check if we need to publish anything  
    if (readyPublish(lastTxTop, newTop, currentTop)) {
      publishHwcTemp("top", newTop);
      currentTop = newTop;
      lastTxTop = millis();
    }
    
    if (readyPublish(lastTxMid, newMid, currentMid)) {
      publishHwcTemp("middle", newMid);
      currentMid = newMid;
      lastTxMid = millis();
    }
    
    if (readyPublish(lastTxBot, newBot, currentBot)) {
      publishHwcTemp("bottom", newBot);
      currentBot = newBot;
      lastTxBot = millis();
    }
  }
}

void mqtt_callback(char * topic, byte * payload, unsigned int length) {
  // tokenise the topic
  char * topictoken;
  topictoken = strtok(topic, "/");
  
  // junk the first three tokens - i.e. /openhab/heating/in/...
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

boolean mqttConnect() {
  Serial.print("Attempting to connect to MQTT broker as ");
  Serial.print(mqttUsername);
  Serial.print("...");
  
  // attempt to connect to the broker and setup our subscriptions etc
  boolean success = mqtt.connect(mqttClientId, mqttUsername, mqttPassword, mqttLwtTopic, mqttLwtQos, mqttLwtRetain, "0");
  if (success) {
    Serial.println("success");
    // subscribe to our heating 'in' topic
    mqtt.subscribe(getOpenhabTopic("in/+"));
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
    
    publishPinState(pin.pin, state);
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
        publishPinState(pin.pin, newState);
        pin.published = newState;
    }
  }

  // update our pin state
  pin.state = newState;
}

double SteinhartHart(double R)
{
  // calculate temperature
  double logR  = log(R);
  double logR3 = logR * logR * logR;
  
  return 1.0 / (A + B * logR + C * logR3);
}

double readSensor(byte pin) {
  // throw away the first few readings
  for (int i = 0; i < SAMPLE_WASTAGE; i++) {
    analogRead(pin);
  }

  // read the raw ADC value a number of times to get an average
  double adc_raw = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    adc_raw += analogRead(pin);
  }
  adc_raw /= SAMPLE_COUNT;

  // calculate voltage
  double V =  adc_raw / 1024 * V_REF;

  // calculate the voltage drop
  double V_drop = V_REF - V;
  
  // calculate the thermistor resistance
  double R_th = (R1 * V_drop) / (V_REF - V_drop);

  // calculate the temperature
  double kelvin = SteinhartHart(R_th) - V_drop * V_drop / (K * R_th);
  double celsius = kelvin - 273.15;
  
  // do bound checking
  if (celsius < 0 || celsius > 100)
    return SENSOR_ERROR;
	
  return celsius;
}

boolean readyPublish(unsigned long lastTx, double newTemp, double currentTemp) {
  // check for sensor errors
  if (newTemp == SENSOR_ERROR)
    return false;
  // if we have published in the last few seconds then rate limit
  if ((millis() - lastTx) < MIN_TX_SILENCE)
    return false;
  // if we haven't broadcast in the last minute then publish
  if ((millis() - lastTx) > MAX_TX_SILENCE)
    return true;

  // check if we have changed enough to warrant a publish    
  return abs(newTemp - currentTemp) > PUBLISH_DELTA;
}

void publishPinState(byte pin, byte state) {
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

void publishHwcTemp(char * name, double temp) {
  // convert to payload
  static char payload[5];
  dtostrf(temp, 4, 1, payload);
  
  // publish (don't send the last byte which is the line terminator 
  // as it screws with openHAB parsing)
  mqtt.publish(getHwcTopic(name), (byte *)payload, sizeof(payload) - 1, 0);
}

char * getOpenhabTopic(char * subTopic) {
  static char topic[32];
  snprintf(topic, 32, "%s/%s", mqttOpenhabTopic, subTopic);
  return topic;
}

char * getHwcTopic(char * subTopic) {
  static char topic[32];
  snprintf(topic, 32, "%s/%s", mqttHwcTopic, subTopic);
  return topic;
}
