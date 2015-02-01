/***************************************************************************
* File Name: mqtt_solar_h2o.h
* Processor/Platform: Arduino Uno R3 (tested)
* Development Environment: Arduino 1.0.6
***************************************************************************/
#include <avr/wdt.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <PlayingWithFusion_MAX31865.h>
#include <PlayingWithFusion_MAX31865_STRUCT.h>
#include <OneWire.h>

#include "mqtt_solar_h2o.h"

// serial rate
#define SERIAL_BAUD      9600

// startup delay
#define STARTUP_DELAY    5000

// onewire sensors
#define OW_RESOLUTION    10
#define OW_DATA_PIN      9
#define OW_READ_TIMEOUT  5000

// rtd pt-1000 sensor
#define RTD_CS_PIN       8

// error temp flag
#define TEMP_ERROR       -127

// circulating pump
#define PUMP_PIN         7
#define PUMP_ON          0
#define PUMP_OFF         1

// solar H2O controller status
#define STATUS_OK             0
#define STATUS_DISABLED       1
#define STATUS_HWC_MAX        2
#define STATUS_COL_COOLING    3
#define STATUS_COL_WARMING    4
#define STATUS_EMERG_SHUTDOWN 5
#define STATUS_SENSOR_ERROR   9

// monitor pin debouncing (ms)
#define PIN_DEBOUNCE     100

// ethernet MAC address (must be unique on the LAN)
byte mac[]               = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x51 };
byte ip[]                = { 192, 168, 1, 81 };

// MQTT broker - subscribe to <topic>/in/+, publish to <topic>/out/+
byte mqttBroker[]        = { 192, 168, 1, 21 };
char mqttClientId[]      = "heating";
char mqttUsername[]      = "heating";
char mqttPassword[]      = "password";
char mqttMonitorTopic[]  = "/openhab/heating";
char mqttSolarH2OTopic[] = "/solarh2o";
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

// ethernet and MQTT clients
EthernetClient ethernet;
PubSubClient mqtt(mqttBroker, 1883, mqtt_callback, ethernet);

// DS18B20 onewire addresses
byte ow_addr_hwc_b[8]   = { 0x28, 0x2e, 0x3a, 0x15, 0x06, 0x00, 0x00, 0xea };
byte ow_addr_hwc_m[8]   = { 0x28, 0xc4, 0x1b, 0x15, 0x06, 0x00, 0x00, 0x39 };
byte ow_addr_hwc_t[8]   = { 0x28, 0x06, 0x09, 0x15, 0x06, 0x00, 0x00, 0x37 };

// OneWire bus
OneWire onewire(OW_DATA_PIN);
byte owReadStage = 0;
unsigned long owLastRead = 0;

// RTD PT-1000 sensor
PWFusion_MAX31865_RTD rtd(RTD_CS_PIN);

// adjustable configuration parameters
double temp_diff_publish   = 0.5;
double temp_diff_on        = 10.0;
double temp_diff_off       = 8.0;
double temp_hwc_max        = 85.0;
double temp_col_emerg_off  = 130.0;
double temp_col_emerg_on   = 120.0;
double temp_col_max        = 110.0;
double temp_col_min        = 4.0;

// various protection/safety modes
int    pump_enabled        = 1;
int    holiday_mode        = 0;
int    emerg_shutdown      = 0;
int    col_cooling         = 0;
int    col_warming         = 0;

// current state
int    current_pump;
int    current_status;

// temperature variables used to determine when to turn the pump on/off
double temp_hwc_b          = TEMP_ERROR;
double temp_hwc_m          = TEMP_ERROR;
double temp_hwc_t          = TEMP_ERROR;
double temp_col            = TEMP_ERROR;

// the temperatures and state we have published to MQTT
double published_hwc_b     = temp_hwc_b;
double published_hwc_m     = temp_hwc_m;
double published_hwc_t     = temp_hwc_t;
double published_col       = temp_col;
int    published_pump      = -1;
int    published_status    = -1;

void setup() {
  // ensure the watchdog is disabled
  wdt_disable();

  // initialise the serial interface  
  Serial.begin(SERIAL_BAUD);
  
  // dump the sketch details
  Serial.println("---------------------------------------");
  Serial.println("Sketch ID:           mqtt_solar_h2o.ino");
  Serial.println("---------------------------------------");

  // print startup configuration
  printConfig();
  
  // initialise the SPI bus
  SPI.begin();
  
  // configure our monitor pins
  for (int i = 0; i < monitorPinCount; i++) {
    pinMode(monitorPins[i].pin, INPUT_PULLUP); 
  }
  
  // ensure pin 4 is OFF (HIGH is off)
  // (gets set since it is used by the ethernet shield for the SD card)
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH); 
    
  // initialise the ethernet module with our MAC and IP addresses
  Serial.print("Initialising ethernet connection...");
  Ethernet.begin(mac, ip);
  Serial.println("done");
  
  // attempt to connect to our MQTT broker
  mqttConnect();

  // set the resolution for each DS18B20 sensor
  Serial.print("Setting DS18B20 sensor resolutions to ");
  Serial.print(OW_RESOLUTION);
  Serial.print(" bits...");
  owSetResolution(ow_addr_hwc_b, OW_RESOLUTION);
  owSetResolution(ow_addr_hwc_m, OW_RESOLUTION);
  owSetResolution(ow_addr_hwc_t, OW_RESOLUTION);
  Serial.println("done");
  
  // initalise the MAX31865 chip select pin
  Serial.print("Configuring MAX31865...");
  pinMode(RTD_CS_PIN, OUTPUT);
  SPI.setDataMode(SPI_MODE3);
  rtd.MAX31865_config();
  SPI.setDataMode(SPI_MODE0);
  Serial.println("done");
  
  // initialise pump relay pin - ensure it is off
  Serial.print("Starting up with pump OFF...");
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, PUMP_OFF);
  current_pump = PUMP_OFF;
  Serial.println("done");
  
  // enable the watchdog timer - 8s timeout
  wdt_enable(WDTO_8S);
  wdt_reset();
  
  Serial.println("Initialisation complete");
  Serial.println();
}

void loop() {
  // reset the watchdog timer
  wdt_reset();

  // do our solar h2o controller stuff first as this must still
  // run even if our MQTT broker is down
  owReadTemps();
  rtdReadTemps();  

  // give everything a chance to initialise
  if (millis() < STARTUP_DELAY)
    return;

  // run the pump logic    
  updatePump();
  
  // check our DHCP lease is still ok
  Ethernet.maintain();

  // process any MQTT messages - will return false if not connected
  // so attempt to reconnect if required
  if (mqtt.loop() || mqttConnect()) {  
    // check our monitor pins and publish any changes
    for (int i = 0; i < monitorPinCount; i++) {
      readDigital(monitorPins[i]);
    }
    
    // publish our various temperatures if they have changed
    //if (!isTempError(temp_hwc_b) && abs(temp_hwc_b - published_hwc_b) >= temp_diff_publish) {
    if (abs(temp_hwc_b - published_hwc_b) >= temp_diff_publish) {
      publishTemp("hwc_b", temp_hwc_b);
      published_hwc_b = temp_hwc_b;
    }
    //if (!isTempError(temp_hwc_m) && abs(temp_hwc_m - published_hwc_m) >= temp_diff_publish) {
    if (abs(temp_hwc_m - published_hwc_m) >= temp_diff_publish) {
      publishTemp("hwc_m", temp_hwc_m);
      published_hwc_m = temp_hwc_m;
    }
    //if (!isTempError(temp_hwc_t) && abs(temp_hwc_t - published_hwc_t) >= temp_diff_publish) {
    if (abs(temp_hwc_t - published_hwc_t) >= temp_diff_publish) {
      publishTemp("hwc_t", temp_hwc_t);
      published_hwc_t = temp_hwc_t;
    }
    //if (!isTempError(temp_col) && abs(temp_col - published_col) >= temp_diff_publish) {
    if (abs(temp_col - published_col) >= temp_diff_publish) {
      publishTemp("col", temp_col);
      published_col = temp_col;
    }    

    // publish the pump state if it has changed
    if (current_pump != published_pump) {
      publishPumpState();
      published_pump = current_pump;
    }

    // publish the status if it has changed
    if (current_status != published_status) {
      publishStatus();
      published_status = current_status;
    }
  } else {
    printTemps();
  }
}

void mqtt_callback(char * topic, byte * payload, unsigned int length) {
  // check for either out monitor or solar h2o topic
  if (strncmp(topic, mqttMonitorTopic, sizeof(mqttMonitorTopic) - 1) == 0) {
    handleMonitorMqtt(topic, payload, length);
  } else if (strncmp(topic, mqttSolarH2OTopic, sizeof(mqttSolarH2OTopic) - 1) == 0) {
    handleSolarH2OMqtt(topic, payload, length);
  }
}

void handleMonitorMqtt(char * topic, byte * payload, unsigned int length) {
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

void handleSolarH2OMqtt(char * topic, byte * payload, unsigned int length) {
  // tokenise the topic
  char * topictoken;
  topictoken = strtok(topic, "/");
  
  // junk the first token - i.e. /solarh2o/config/...
  topictoken = strtok(NULL, "/");
  topictoken = strtok(NULL, "/");

  // check for the special 'refresh' token
  if (strcmp(topictoken, "pump_enabled") == 0) {
    pump_enabled = *payload == '1';
  } else if (strcmp(topictoken, "holiday_mode") == 0) {
    holiday_mode = *payload == '1';
  } else if (strcmp(topictoken, "temp_diff_publish") == 0) {
    temp_diff_publish = atof((char *)payload);
  } else if (strcmp(topictoken, "temp_diff_on") == 0) {
    temp_diff_on = atof((char *)payload);
  } else if (strcmp(topictoken, "temp_diff_off") == 0) {
    temp_diff_off = atof((char *)payload);
  } else if (strcmp(topictoken, "temp_hwc_max") == 0) {
    temp_hwc_max = atof((char *)payload);
  } else if (strcmp(topictoken, "temp_col_emerg_off") == 0) {
    temp_col_emerg_off = atof((char *)payload);
  } else if (strcmp(topictoken, "temp_col_emerg_on") == 0) {
    temp_col_emerg_on = atof((char *)payload);
  } else if (strcmp(topictoken, "temp_col_max") == 0) {
    temp_col_max = atof((char *)payload);
  } else if (strcmp(topictoken, "temp_col_min") == 0) {
    temp_col_min = atof((char *)payload);
  } else {
    Serial.println("WARNING: unsupported parameter - ignoring");
  }
  
  // debug
  printConfig();
}

boolean mqttConnect() {
  Serial.print("Attempting to connect to MQTT broker as ");
  Serial.print(mqttUsername);
  Serial.print("...");
  
  // attempt to connect to the broker and setup our subscriptions etc
  boolean success = mqtt.connect(mqttClientId, mqttUsername, mqttPassword, mqttLwtTopic, mqttLwtQos, mqttLwtRetain, "0");
  if (success) {
    Serial.println("success");
    // subscribe to our monitor 'in' topic
    mqtt.subscribe(getMonitorTopic("in/+"));
    // subscribe to our solar h2o config topic
    mqtt.subscribe(getSolarH2OTopic("config/+"));
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
  
  if ((millis() - pin.time) > PIN_DEBOUNCE) {
    // the pin has been stable so check if we need to publish
    if (pin.published != newState) {
        publishPinState(pin.pin, newState);
        pin.published = newState;
    }
  }

  // update our pin state
  pin.state = newState;
}

void publishPinState(byte pin, byte state) {
  // build the sub-topic name
  static char subTopic[8]; 
  snprintf(subTopic, 8, "out/%i", pin);
  // convert to payload
  static char payload[1];
  if (state) {
    payload[0] = '0';
  } else {
    payload[0] = '1';
  }
  mqtt.publish(getMonitorTopic(subTopic), (byte *)payload, sizeof(payload), 0);
}

void publishTemp(char * name, double temp) {
  // build the sub-topic
  static char subTopic[12];
  snprintf(subTopic, 12, "temp/%s", name);
  // convert to payload
  static char payload[8];
  dtostrf(temp, 4, 2, payload);
  mqtt.publish(getSolarH2OTopic((char *)subTopic), (byte *)payload, sizeof(payload), 0);
}

void publishStatus() {
  // convert to payload
  static char payload[1];
  dtostrf(current_status, 1, 0, payload);
  mqtt.publish(getSolarH2OTopic("status"), (byte *)payload, sizeof(payload), 0);
}

void publishPumpState() {
  // convert to byte payload
  static char payload[1];
  if (current_pump) {
    payload[0] = '0';
  } else {
    payload[0] = '1';
  }
  mqtt.publish(getSolarH2OTopic("pump"), (byte *)payload, sizeof(payload), 0);
}

char * getMonitorTopic(char * subTopic) {
  static char topic[32];
  snprintf(topic, 32, "%s/%s", mqttMonitorTopic, subTopic);
  return topic;
}

char * getSolarH2OTopic(char * subTopic) {
  static char topic[32];
  snprintf(topic, 32, "%s/%s", mqttSolarH2OTopic, subTopic);
  return topic;
}

void owReadTemps() {
  if (owReadStage == 0) {
    // fire off a convert command to each sensor first
    owConvertCommand(ow_addr_hwc_b);
    owConvertCommand(ow_addr_hwc_m);
    owConvertCommand(ow_addr_hwc_t);
    owReadStage = 1;
  } else {
    // wait for the sensors to be ready
    if (onewire.read()) {
      // when the sensors have completed conversion then read
      temp_hwc_b = owReadTemp(ow_addr_hwc_b); 
      temp_hwc_m = owReadTemp(ow_addr_hwc_m); 
      temp_hwc_t = owReadTemp(ow_addr_hwc_t); 
      owReadStage = 0;
      owLastRead = millis();
    } else {
      // if we haven't received a reading in a while then fail and try again
      if ((millis() - owLastRead) > OW_READ_TIMEOUT) {
        temp_hwc_b = TEMP_ERROR; 
        temp_hwc_m = TEMP_ERROR; 
        temp_hwc_t = TEMP_ERROR; 
        owReadStage = 0;
        owLastRead = millis();
      }
    }
  }
}

void owSetResolution(byte addr[8], byte resolution) {
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

void owConvertCommand(byte addr[8]){
  onewire.reset();
  onewire.select(addr);
  onewire.write(0x44, 0);        // start conversion (non-parasitic power)
}

double owReadTemp(byte addr[8]) {  
  onewire.reset();
  onewire.select(addr);
  onewire.write(0xBE);          // read scratchpad
  
  byte data[9];
  for (int i = 0; i < 9; i++) {
    data[i] = onewire.read();
  }
  
  // check the reading was valid
  if (onewire.crc8(data, 8) != data[8])
    return TEMP_ERROR;
    
  // convert the data to actual temperature  
  unsigned int raw = (data[1] << 8) | data[0];
  return (double)raw / 16.0;
}

void rtdReadTemps() {
  static struct var_max31865 RTD;  
  RTD.RTD_type = 2;                         // type 1 for PT100, 2 for PT1000

  SPI.setDataMode(SPI_MODE3);
  rtd.MAX31865_full_read(&RTD);             // update MAX31855 readings 
  SPI.setDataMode(SPI_MODE0);

  if (0 == RTD.status) {
    // calculate RTD temperature (simple calc, +/- 2 deg C from -100C to 100C)
    // more accurate curve can be used outside that range
    temp_col = ((double)RTD.rtd_res_raw / 32) - 256;
  } else {
    // mark the collector temp as in error to stop any pump logic
    temp_col = TEMP_ERROR;

    // print the fault details
    Serial.print("RTD Fault, register: ");
    Serial.print(RTD.status);
    Serial.print(" -> ");
    if(0x80 & RTD.status)
      Serial.println("RTD High Threshold Met");  // RTD high threshold fault
    else if(0x40 & RTD.status)
      Serial.println("RTD Low Threshold Met");   // RTD low threshold fault
    else if(0x20 & RTD.status)
      Serial.println("REFin- > 0.85 x Vbias");   // REFin- > 0.85 x Vbias
    else if(0x10 & RTD.status)
      Serial.println("FORCE- open");             // REFin- < 0.85 x Vbias, FORCE- open
    else if(0x08 & RTD.status)
      Serial.println("FORCE- open");             // RTDin- < 0.85 x Vbias, FORCE- open
    else if(0x04 & RTD.status)
      Serial.println("Over/Under voltage fault");  // overvoltage/undervoltage fault
    else
      Serial.println("Unknown fault, check connection"); // print RTD temperature heading
  }
}

void updatePump() {  
  int pump;
  if (!pump_enabled) {
    // shut off the pump if we are disabled
    current_status = STATUS_DISABLED;
    pump = PUMP_OFF;
  } else if (isTempError(temp_hwc_b) || isTempError(temp_hwc_m) || 
             isTempError(temp_hwc_t) || isTempError(temp_col)) {
    // ensure the pump is off if we don't have a full set of temperature readings
    current_status = STATUS_SENSOR_ERROR;
    pump = PUMP_OFF;
    // dump the temps so we can see any issues
    printTemps();
  } else {  
    // start with OK and the current pump state
    current_status = STATUS_OK;
    pump = current_pump;
    
    // first we check if we are within the boundaries of any of our 
    // protection/safety modes to prevent any damage to the system
    
    // emergency overheating
    if (temp_col > temp_col_emerg_off )
      emerg_shutdown = 1;
    if (temp_col < temp_col_emerg_on)
      emerg_shutdown = 0;
     
    // collector cooling mode
    if (temp_col > temp_col_max)
      col_cooling = 1;
    if (temp_col < (temp_col_max - 2))
      col_cooling = 0;
     
    // collector warming mode
    if (temp_col < temp_col_min)
      col_warming = 1;
    if (temp_col > (temp_col_min + 2))
      col_warming = 0;
    
    // calculate the difference between the collector and HWC bottom
    double diff = temp_col - temp_hwc_b;

    // check for our special holiday mode - this needs to be switched
    // on/off each night (by openHAB) so it runs when the sun is down
    // otherwise we won't re-heat the HWC during the day and the collector
    // will overheat
    // TODO: if we had a RTC we could check the time of day ourselves...
    if (holiday_mode) {
      // if the collector is 8 degrees cooler than the HWC bottom and the 
      // tank is above 35 degrees, then switch on pump to cool the HWC
      if (diff < -8.0 && temp_hwc_m > 35.0)
        pump = PUMP_ON;
      
      // switch off pump once the collector gets warmer
      if (diff > -2.0)
        pump = PUMP_OFF;        
    } else {
      // if collector is X degrees above the HWC bottom then pump on
      if (diff > temp_diff_on) 
        pump = PUMP_ON;
      
      // switch off once the collector cools down
      if (diff < temp_diff_off)
        pump = PUMP_OFF;
    }
    
    // check to stop overheating of cylinder 
    if (temp_hwc_m > temp_hwc_max) {
      current_status = STATUS_HWC_MAX;
      pump = PUMP_OFF;  
    }
    
    // check if we are in the collector cooling or warming modes
    // and override our differential calculations above
    if (col_cooling) {
      current_status = STATUS_COL_COOLING;
      pump = PUMP_ON;
    }
    if (col_warming) {
      current_status = STATUS_COL_WARMING;
      pump = PUMP_ON;
    }

    // emergency override so we don't overheat
    if (emerg_shutdown || temp_hwc_m > 95.0 || temp_hwc_t > 95.0) {
      current_status = STATUS_EMERG_SHUTDOWN;
      pump = PUMP_OFF;
    }    
  }

  // only update the pump if changed
  if (pump != current_pump) {
    Serial.print("Switching pump -> ");
    Serial.println(pump == PUMP_ON ? "ON" : "OFF");
    printTemps();
    digitalWrite(PUMP_PIN, pump);
    current_pump = pump;
  }
}

boolean isTempError(double temp) {
  return temp <= TEMP_ERROR;
}

void printTemps() {
  Serial.print("HWC Bot: ");
  Serial.print(temp_hwc_b);
  Serial.print("C, Mid: ");
  Serial.print(temp_hwc_m);
  Serial.print("C, Top: ");
  Serial.print(temp_hwc_t);
  Serial.print("C, Col: ");
  Serial.print(temp_col);
  Serial.println("C");
}

void printConfig() {
//  Serial.println("Current config:");
//  Serial.print(" * pump_enabled:  ");
//  Serial.println(pump_enabled);
//  Serial.print(" * holiday_mode:  ");
//  Serial.println(holiday_mode);
//  Serial.print(" * temp_diff_on:  ");
//  Serial.println(temp_diff_on);
//  Serial.print(" * temp_diff_off: ");
//  Serial.println(temp_diff_off);
//  Serial.print(" * temp_hwc_max:  ");
//  Serial.println(temp_hwc_max);
//  Serial.print(" * temp_col_max:  ");
//  Serial.println(temp_col_max);
//  Serial.print(" * temp_col_min:  ");
//  Serial.println(temp_col_min);
//  Serial.print(" * temp_col_emerg_off: ");
//  Serial.println(temp_col_emerg_off);
//  Serial.print(" * temp_col_emerg_on:  ");
//  Serial.println(temp_col_emerg_on);
//  Serial.print(" * temp_diff_publish:  ");
//  Serial.println(temp_diff_publish);
//  Serial.println();  
}

