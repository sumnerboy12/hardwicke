#include <avr/wdt.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Wire.h>

#include "sensornet_gateway.h"

// serial rate
#define SERIAL_BAUD     9600

// the I2C bus address
#define I2C_ADDRESS     0x9

// unique MAC address on our LAN (0x52 => .82)
byte mac[]              = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x52 };
byte ip[]               = { 192, 168, 1, 82 };

// MQTT broker connection properties
byte mqttBroker[]       = { 192, 168, 1, 21 };
char mqttClientId[]     = "sensornet";
char mqttUsername[]     = "sensornet";
char mqttPassword[]     = "password";
char mqttTopic[]        = "/sensornet/out";
char mqttTopicLwt[]     = "/clients/sensornet";
int  mqttLwtQos         = 0;
int  mqttLwtRetain      = 1;

// ethernet and MQTT clients
EthernetClient ethernet;
PubSubClient mqtt(mqttBroker, 1883, mqtt_callback, ethernet);

// the I2C packet from the RF bridge
I2CPacket payload;

void setup() {
  // ensure the watchdog is disabled
  wdt_disable();

  // initialise the serial interface  
  Serial.begin(SERIAL_BAUD);

  // dump the sketch details
  Serial.println("---------------------------------------");
  Serial.println("Sketch ID:        sensornet_gateway.ino");
  Serial.println("---------------------------------------");
  Serial.println();
  
  // initialise the SPI bus
  SPI.begin();
  
  // initialise the ethernet module with our MAC and IP addresses
  Serial.print("Initialising ethernet connection...");
  Ethernet.begin(mac, ip);
  Serial.println("done");
  
  // attempt to connect to our MQTT broker
  mqttConnect();
  
  // start I2C Bus as slave, listening for events
  Serial.print("Starting up the I2C bus...");
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Serial.println("done");
  
  // enable the watchdog timer - 8s timeout
  wdt_enable(WDTO_8S);
  wdt_reset();

  Serial.println("Initialisation complete");
}

// for joining back to the main thread
int dataReady = 0;

void loop() { 
  // reset the watchdog timer
  wdt_reset();
  
  // check our DHCP lease is still ok
  Ethernet.maintain();

  // process any MQTT messages - will return false if not connected
  // so attempt to reconnect if required
  if (mqtt.loop() || mqttConnect()) {  
    // if we have data then publish an MQTT message
    if (dataReady == 1) {
      static char topic[32];  
      static char message[8];  
  
      // publish the data
      snprintf(topic, 32, "%s/%i/%i", mqttTopic, payload.nodeId, payload.pin);
      snprintf(message, 8, "%i", payload.data);
      mqtt.publish(topic, message);
  
      // publish the rssi
      snprintf(topic, 32, "%s/%i/rssi", mqttTopic, payload.nodeId);
      snprintf(message, 8, "%i", payload.rssi);
      mqtt.publish(topic, message);
  
      // publish the battery
      snprintf(topic, 32, "%s/%i/battery", mqttTopic, payload.nodeId);
      float voltage = ((float)payload.battery * 3.3 * 9.5) / (1023 * 2.98); 
      dtostrf(voltage, 1, 4, message);
      mqtt.publish(topic, message);
  
      // reset the data ready flag
      dataReady = 0;
    }
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // we don't subscribe to any topics
}

boolean mqttConnect() {
  Serial.print("Attempting to connect to MQTT broker as ");
  Serial.print(mqttUsername);
  Serial.print("...");
  
  // attempt to connect to the broker and setup our subscriptions etc
  boolean success = mqtt.connect(mqttClientId, mqttUsername, mqttPassword, mqttTopicLwt, mqttLwtQos, mqttLwtRetain, "0");
  if (success) {
    Serial.println("success");
    // publish retained LWT so anything listening knows we are alive
    byte data[] = { '1' };
    mqtt.publish(mqttTopicLwt, data, 1, mqttLwtRetain);
  } else {
    Serial.println("failed");
  }
  return success;
}

void receiveEvent(int howMany) {
  // check this is a valid data packet
  if (howMany != sizeof(payload)) {
    Serial.print("Invalid payload received - expecting ");
    Serial.print(sizeof(payload));
    Serial.print(" but received ");
    Serial.println(howMany);
    return;
  }
  
  // read the data into our struct
  byte * p = (byte *)&payload;
  for (byte i = 0; i < sizeof(payload); i++)
    *p++ = Wire.read();
   
  // set the flag so the main loop can handle
  dataReady = 1;
}

