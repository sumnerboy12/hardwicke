/***************************************************************************
* File Name: mqtt_heating3.ino
* Processor/Platform: Arduino Uno R3 (tested)
* Development Environment: Arduino 1.6.0
*
* Description: standard IO monitoring/control (via relay board)
*              reads 3xNTC10K sensors (HWC) and publishes values
*              reads 2xDS18B20 sensors (UFHP) and publishes values
***************************************************************************/
#include <avr/wdt.h>
#include <math.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <OneWire.h>

// serial rate
#define SERIAL_BAUD          9600

// number of digital IO pins we are monitoring
#define MONITOR_PIN_COUNT    2
#define MONITOR_PIN_DEBOUNCE 100

// sensor error indicator
#define SENSOR_ERROR	     -1

// tx at at most every 5s, and at least every 60s
#define PUBLISH_MIN_SILENCE  5000
#define PUBLISH_MAX_SILENCE  60000
#define PUBLISH_DELTA        0.25

// analog pins patched to NTC 10K thermistors
#define NTC_HWC_TOP_PIN      A3
#define NTC_HWC_MID_PIN      A4
#define NTC_HWC_BOT_PIN      A5

// number of samples to read
#define NTC_SAMPLE_WASTAGE   3
#define NTC_SAMPLE_COUNT     12

// DS18B20 onewire sensors
#define OW_RESOLUTION        10
#define OW_DATA_PIN          9
#define OW_READ_TIMEOUT      5000

// custom structs
typedef struct 
{
  byte pin;
  byte state;
  byte published;
  unsigned long time;
} Pin;

void readDigitalPin(Pin& pin);

typedef struct 
{
  char * subTopic;
  char * tempName;
  double newValue;
  double currentValue;
  unsigned long lastPub;
} Temp;

void publishTemp(Temp& temp);
boolean readyPublishTemp(Temp& temp);

// NOTE: pins 0 & 1 are used for serial RX/TX
// NOTE: ethernet shield uses pins 4 for SD card and 10-13 for SPI
Pin monitorPins[MONITOR_PIN_COUNT] = 
{ 
  { A0, -1, -1, 0L }, 
  { A1, -1, -1, 0L } 
};

// NTC 10K thermistor constants
double R1                = 10000.0;
double V_REF             = 4.91;

// Steinhart coefficients
double A                 = 1.129148e-3;
double B                 = 2.34125e-4;
double C                 = 8.76741e-8;
double K                 = 9.5;

// DS18B20 OneWire addresses
byte owAddrUfhpFlow[8]   = { 0x28, 0x2e, 0x3a, 0x15, 0x06, 0x00, 0x00, 0xea };
byte owAddrUfhpReturn[8] = { 0x28, 0x06, 0x09, 0x15, 0x06, 0x00, 0x00, 0x37 };

// OneWire bus
OneWire onewire(OW_DATA_PIN);
byte owReadStage = 0;
unsigned long owLastRead = 0;

// structs to hold our various temp sensor states
Temp hwcTop              = { "hwc", "top", 0, 0, 0L };
Temp hwcMid              = { "hwc", "middle", 0, 0, 0L };
Temp hwcBot              = { "hwc", "bottom", 0, 0, 0L };
Temp ufhpFlow            = { "ufhp", "flow", 0, 0, 0L };
Temp ufhpReturn          = { "ufhp", "return", 0, 0, 0L };

// ethernet MAC address (must be unique on the LAN)
byte mac[]               = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x51 };
byte ip[]                = { 192, 168, 1, 81 };

// MQTT broker - subscribe to <topic>/in/+, publish to <topic>/out/+
byte mqttBroker[]        = { 192, 168, 1, 21 };
char mqttClientId[]      = "heating";
char mqttUsername[]      = "heating";
char mqttPassword[]      = "password";
char mqttOpenhabTopic[]  = "/openhab/heating";
char mqttHeatingTopic[]  = "/heating";
char mqttLwtTopic[]      = "/clients/heating";
int  mqttLwtQos          = 0;
int  mqttLwtRetain       = 1;

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
  Serial.println("Sketch ID:            mqtt_heating3.ino");
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

  // setup our ADC to use the external voltage reference
  // (we connect the solar controller VCC to AREF)
  Serial.print("Setting ADC to use external voltage reference...");
  analogReference(EXTERNAL);
  Serial.println("done");
  
  // set the resolution for each DS18B20 sensor
  Serial.print("Setting DS18B20 sensor resolutions to ");
  Serial.print(OW_RESOLUTION);
  Serial.print(" bits...");
  setOneWireResolution(owAddrUfhpFlow, OW_RESOLUTION);
  setOneWireResolution(owAddrUfhpReturn, OW_RESOLUTION);
  Serial.println("done");

  // enable the watchdog timer - 8s timeout
  Serial.print("Enabling watchdog timeout for ");
  Serial.print(WDTO_8S);
  Serial.print(" secs...");
  wdt_enable(WDTO_8S);
  wdt_reset();
  Serial.println("done");
  
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
    for (int i = 0; i < MONITOR_PIN_COUNT; i++) {
      readDigitalPin(monitorPins[i]);
    }
    
    // read NTC (HWC) sensors
    readNtcSensors();
    // read DS18B20 (UFHP) sensors
    readOneWireSensors();
    
    // publish any temp changes  
    publishTemp(hwcTop);
    publishTemp(hwcMid);
    publishTemp(hwcBot);
    publishTemp(ufhpFlow);
    publishTemp(ufhpReturn);
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

boolean mqttConnect() 
{
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
** HWC NTC10K sensor methods
*/
void readNtcSensors()
{
  hwcTop.newValue = readNtcSensor(NTC_HWC_TOP_PIN);
  hwcMid.newValue = readNtcSensor(NTC_HWC_MID_PIN);
  hwcBot.newValue = readNtcSensor(NTC_HWC_BOT_PIN);
}

double readNtcSensor(byte pin) 
{
  // throw away the first few readings
  for (int i = 0; i < NTC_SAMPLE_WASTAGE; i++) {
    analogRead(pin);
  }

  // read the raw ADC value a number of times to get an average
  double adc_raw = 0;
  for (int i = 0; i < NTC_SAMPLE_COUNT; i++) {
    adc_raw += analogRead(pin);
  }
  adc_raw /= NTC_SAMPLE_COUNT;

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

double SteinhartHart(double R)
{
  // calculate temperature
  double logR  = log(R);
  double logR3 = logR * logR * logR;
  
  return 1.0 / (A + B * logR + C * logR3);
}

/*
** UFHP DB18B20 OneWire temp sensors
*/
void readOneWireSensors() 
{
  if (owReadStage == 0) {
    // fire off a convert command to each sensor first
    convertOneWireCommand(owAddrUfhpFlow);
    convertOneWireCommand(owAddrUfhpReturn);
    owReadStage = 1;
  } else {
    // wait for the sensors to be ready
    if (onewire.read()) {
      // when the sensors have completed conversion then read
      ufhpFlow.newValue = readOneWireTemp(owAddrUfhpFlow); 
      ufhpReturn.newValue = readOneWireTemp(owAddrUfhpReturn); 
      owReadStage = 0;
      owLastRead = millis();
    } else {
      // if we haven't received a reading in a while then fail and try again
      if ((millis() - owLastRead) > OW_READ_TIMEOUT) {
        ufhpFlow.newValue = SENSOR_ERROR; 
        ufhpReturn.newValue = SENSOR_ERROR; 
        owReadStage = 0;
        owLastRead = millis();
      }
    }
  }
}

void setOneWireResolution(byte addr[8], byte resolution) 
{
  // get byte for desired resolution
  byte resbyte;
  if      (resolution == 12) { resbyte = 0x7F; }
  else if (resolution == 11) { resbyte = 0x5F; }
  else if (resolution == 10) { resbyte = 0x3F; }
  else                       { resbyte = 0x1F; }
  
  onewire.reset();
  onewire.select(addr);
  onewire.write(0x4E);         // write scratchpad
  onewire.write(0);            // TL
  onewire.write(0);            // TH
  onewire.write(resbyte);      // configuration register
  onewire.write(0x48);         // copy scratchpad
}

void convertOneWireCommand(byte addr[8])
{
  onewire.reset();
  onewire.select(addr);
  onewire.write(0x44, 0);        // start conversion (non-parasitic power)
}

double readOneWireTemp(byte addr[8]) 
{  
  onewire.reset();
  onewire.select(addr);
  onewire.write(0xBE);          // read scratchpad
  
  byte data[9];
  for (int i = 0; i < 9; i++) {
    data[i] = onewire.read();
  }
  
  // check the reading was valid
  if (onewire.crc8(data, 8) != data[8])
    return SENSOR_ERROR;
    
  // convert the data to actual temperature  
  unsigned int raw = (data[1] << 8) | data[0];
  return (double)raw / 16.0;
}

/*
** Helper methods
*/
// publish a temperature reading to an MQTT topic
//  - /heating/<subtopic>/<tempName>
void publishTemp(Temp& temp) 
{
  // check if this temp requires publishing
  if (!readyPublishTemp(temp))
    return;
  
  // convert to payload
  static char payload[6];
  dtostrf(temp.newValue, 3, 1, payload);
  
  // publish (don't send the last byte which is the line terminator 
  // as it screws with openHAB parsing)
  mqtt.publish(getHeatingTopic(temp.subTopic, temp.tempName), (byte *)payload, strlen(payload), 0);
  
  // update our published state
  temp.currentValue = temp.newValue;
  temp.lastPub = millis();
}

// determine whether a temp reading should be published
//  - will not publish if just recently published
//  - will publish if hasn't published for a while
//  - will publish if the temp has changed more than our delta
boolean readyPublishTemp(Temp& temp) 
{
  // check for sensor errors
  if (temp.newValue == SENSOR_ERROR)
    return false;
  // if we have published in the last few seconds then rate limit
  if ((millis() - temp.lastPub) < PUBLISH_MIN_SILENCE)
    return false;
  // if we haven't broadcast in the last minute then publish
  if ((millis() - temp.lastPub) > PUBLISH_MAX_SILENCE)
    return true;

  // check if we have changed enough to warrant a publish    
  return abs(temp.newValue - temp.currentValue) > PUBLISH_DELTA;
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

char * getHeatingTopic(char * subTopic, char * tempName) 
{
  static char topic[32];
  snprintf(topic, 32, "%s/%s/%s", mqttHeatingTopic, subTopic, tempName);
  return topic;
}

