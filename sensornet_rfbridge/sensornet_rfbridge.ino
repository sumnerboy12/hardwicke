// Based on sample RFM69 receiver/gateway sketch, with ACK and encryption
// Library and code by Felix Rusu - felix@lowpowerlab.com
// Get the RFM69 library at: https://github.com/LowPowerLab/

#include <RFM69.h>
#include <SPI.h>
#include <Wire.h>

#define NODEID        1    // unique for each node on same network
#define NETWORKID     1    // the same on all nodes that talk to each other
#define FREQUENCY     RF69_433MHZ
#define ENCRYPTKEY    "encrypt"
#define ACK_TIME      30   // max # of ms to wait for an ack
#define LED           9    // Moteinos have LEDs on D9

// the I2C address of the sensornet gateway
#define I2C_GATEWAY_ADDRESS 0x9

typedef struct {
  // sent from remote nodes
  byte pin;
  int data;
  int battery;
  // filled in by the RF bridge
  byte nodeId;
  int rssi;
} I2CPacket payload;

RFM69 radio;

void setup() 
{ 
  // initialise the radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower();
  radio.encrypt(ENCRYPTKEY);

  // start I2C Bus as master
  Wire.begin();
}

void loop() 
{
  if (radio.receiveDone())
  {
    // read the data into our struct
    byte * p = (byte *)&payload;
    for (byte i = 0; i < radio.DATALEN; i++) 
      *p++ = radio.DATA[i];
	
    // add the radio specific stuff (not tx'ed)
    payload.nodeId = radio.SENDERID;
    payload.rssi = radio.RSSI;

    // send over the I2C bus to the gateway
    Wire.beginTransmission(I2C_GATEWAY_ADDRESS);
    Wire.write((byte *)&payload, sizeof(payload));
    Wire.endTransmission();

    // send an ACK if requested
    if (radio.ACKRequested()) 
      radio.sendACK();

    // flash the LED to indicate a packet has been processed
    blink(LED, 5);
  }
}

void blink(byte pin, int delayMs)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin,HIGH);
  delay(delayMs);
  digitalWrite(pin,LOW);
}
