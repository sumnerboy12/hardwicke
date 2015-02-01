// **** INCLUDES *****
#include <RFM69.h>
#include <SPI.h>
#include <LowPower.h>

// uncomment for serial debugging
//#define DEBUG

#define NODEID        2       // unique for each node on same network
#define NETWORKID     1       // the same on all nodes that talk to each other
#define GATEWAYID     1       // gateway node id
#define FREQUENCY     RF69_433MHZ
#define ENCRYPTKEY    "encrypt"
#define ACK_TIME      30      // max # of ms to wait for an ack
#define LED           9       // Moteinos have LEDs on D9

#define INTERRUPT     1       // 1 = D3 (for some reason D2 not working)
#define INTERRUPT_PIN 3
#define BATTERY_PIN   A0

// radio
RFM69 radio;

void gateInterrupt()
{
  // nothing to do since we just want to wakeup
}

void setup() {  
  // initialise the serial interface
  Serial.begin(9600);
  Serial.println("-------------------------------------");
  Serial.println("Sketch ID:     sensor_rfnode_gate.ino");
  Serial.println("-------------------------------------");
  
  // initialise the radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower();
  radio.encrypt(ENCRYPTKEY);

  Serial.print("Initialised radio on network ");
  Serial.print(NETWORKID);
  Serial.print(" as node ");
  Serial.println(NODEID);

  // configure the interrupt pin
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  Serial.println("Node initialised and ready");
  Serial.flush();
}

void loop() {
  // allow wake up pin to trigger interrupt on a change
  attachInterrupt(INTERRUPT, gateInterrupt, CHANGE);
  
  // enter power down state with ADC and BOD module disabled
  // wake up when wake up pin changes
  radio.sleep();
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
  
  // disable external pin interrupt on wake up pin
  detachInterrupt(INTERRUPT); 

  // sleep for a short period in order to debounce
  LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);

#ifdef DEBUG
  Serial.print("Reading sensors...");
#endif

  // read the gate/battery sensors
  int gateReading = digitalRead(INTERRUPT_PIN);
  int batteryReading = analogRead(BATTERY_PIN);

#ifdef DEBUG
  Serial.print("publishing...");
  Serial.print("pin ");
  Serial.print(INTERRUPT_PIN);
  Serial.print(" -> ");
  Serial.print(gateReading);
  Serial.print(" (battery ");
  Serial.print(batteryReading);
  Serial.print(")...");
#endif

  // publish
  sendPayload(INTERRUPT_PIN, gateReading, batteryReading);
  
#ifdef DEBUG
  Serial.println("complete.");
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

  // send payload to our gateway
  radio.sendWithRetry(GATEWAYID, payload, sizeof(payload));
}

