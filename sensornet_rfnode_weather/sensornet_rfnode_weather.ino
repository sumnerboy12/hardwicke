// **** INCLUDES *****
#include <RFM69.h>
#include <SPI.h>
#include <LowPower.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "DHT.h"

// uncomment for serial debugging
//#define DEBUG

#define NODEID        3       // unique for each node on same network
#define NETWORKID     1       // the same on all nodes that talk to each other
#define GATEWAYID     1       // gateway node id
#define FREQUENCY     RF69_433MHZ
#define ENCRYPTKEY    "encrypt"
#define ACK_TIME      30      // max # of ms to wait for an ack
#define LED           9       // Moteinos have LEDs on D9

#define SLEEP_MS      600000  // 10 minutes
#define BATTERY_PIN   A0
#define DHT_PWR_PIN   6
#define DHT_PIN       5
#define RAIN_INT      1       // 1 = D3 (for some reason D2 not working)
#define RAIN_PIN      3

// sensors
DHT dht;
Adafruit_BMP085 bmp;

// radio
RFM69 radio;

// last sensor values - to prevent unnecessary RF transmissions
float lastTemperature = 0;
float lastHumidity = 0;
int lastPressure = 0;
int lastRain = 0;

void rainInterrupt() {
  // set the rain event
  lastRain = 1;  
}

void setup() {
  // initialise the serial interface
  Serial.begin(9600);
  Serial.println("-------------------------------------");
  Serial.println("Sketch ID:  sensor_rfnode_weather.ino");
  Serial.println("-------------------------------------");
  
  // log our broadcast interval
  Serial.print("Broadcast interval is ");
  Serial.print(SLEEP_MS / 1000);
  Serial.println(" seconds");
  
  // initialise the radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower();
  radio.encrypt(ENCRYPTKEY);

  Serial.print("Initialised radio on network ");
  Serial.print(NETWORKID);
  Serial.print(" as node ");
  Serial.println(NODEID);
  
  // setup temp/humidity sensor power pin
  pinMode(DHT_PWR_PIN, OUTPUT);
  digitalWrite(DHT_PWR_PIN, LOW);

  // configure the rain sensor pin
  pinMode(RAIN_PIN, INPUT_PULLUP);

  // initialise the sensors
  dht.setup(DHT_PIN);
  bmp.begin();

  Serial.println("Node initialised and ready");
  Serial.flush();
}

void loop() {
  // power down the sensors
  digitalWrite(DHT_PWR_PIN, LOW);

  // allow rain sensor to trigger interrupt on a change
  attachInterrupt(RAIN_INT, rainInterrupt, LOW);
  
  // put the radio to sleep (will wakeup when we try to send)
  radio.sleep();

  // sleep till we are ready to send
  longPowerDown(SLEEP_MS);
  
  // disable external pin interrupt on wake up pin
  detachInterrupt(RAIN_INT); 
  
  // power up the sensors
#ifdef DEBUG
  Serial.println("Powering up sensors...");
#endif
  digitalWrite(DHT_PWR_PIN, HIGH);

  // wait for the sensors to settle (DHT is slowest)
  delay(dht.getMinimumSamplingPeriod());

  // read our sensor values
#ifdef DEBUG
  Serial.println("Reading sensors...");
#endif
  float temperature = dht.getTemperature();
  float humidity = dht.getHumidity();
  int pressure = bmp.readPressure() / 100;

  // power down the sensors
#ifdef DEBUG
  Serial.println("Powering down sensors...");
#endif
  digitalWrite(DHT_PWR_PIN, LOW);

  // read our battery voltage  
  int battery = analogRead(BATTERY_PIN);

  // /sensornet/out/3/1 -> temperature
  if (!isnan(temperature) && (temperature != lastTemperature)) {
    sendPayload(1, (int)temperature, battery);
    lastTemperature = temperature;
  }

  // /sensornet/out/3/2 -> humidity
  if (!isnan(humidity) && (humidity != lastHumidity)) {
    sendPayload(2, (int)humidity, battery);
    lastHumidity = humidity;
  }
  
  // /sensornet/out/3/3 -> pressure
  if (pressure != lastPressure) {
    sendPayload(3, pressure, battery);
    lastPressure = pressure;
  }
  
  // /sensornet/out/3/5 -> rain event
  if (lastRain) { 
    sendPayload(5, 1, battery); 
    lastRain = 0;
  }
  
#ifdef DEBUG
  Serial.println("Sensor loop complete");
  Serial.flush();
#endif
}

void sendPayload(byte pin, int data, int battery) {  
  // build the payload to send over RF
  static byte payload[5]; 
  payload[0] = pin;
  payload[1] = lowByte(data);
  payload[2] = highByte(data);
  payload[3] = lowByte(battery);
  payload[4] = highByte(battery);

#ifdef DEBUG
  Serial.print("Sending payload to pin ");
  Serial.print(pin);
  Serial.print(" -> ");
  Serial.print(data);
  Serial.print(" (battery ");
  Serial.print(battery);
  Serial.println("/1023)");
#endif
  
  // send payload to our gateway (ignore ACK)
  radio.sendWithRetry(GATEWAYID, payload, sizeof(payload));
}

void longPowerDown(unsigned long milliseconds) 
{
  while (milliseconds >= 8000) { LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    milliseconds -= 8000; }
  if (milliseconds >= 4000)    { LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);    milliseconds -= 4000; }
  if (milliseconds >= 2000)    { LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);    milliseconds -= 2000; }
  if (milliseconds >= 1000)    { LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);    milliseconds -= 1000; }
  if (milliseconds >= 500)     { LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF); milliseconds -= 500; }
  if (milliseconds >= 250)     { LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF); milliseconds -= 250; }
  if (milliseconds >= 125)     { LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF); milliseconds -= 120; }
  if (milliseconds >= 64)      { LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);  milliseconds -= 60; }
  if (milliseconds >= 32)      { LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);  milliseconds -= 30; }
  if (milliseconds >= 16)      { LowPower.powerDown(SLEEP_15Ms, ADC_OFF, BOD_OFF);  milliseconds -= 15; }
}