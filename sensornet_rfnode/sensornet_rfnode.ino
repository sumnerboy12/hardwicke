// **** INCLUDES *****
#include <RFM69.h>
#include <SPI.h>
#include <LowPower.h>

#define NODEID        9       // unique for each node on same network
#define NETWORKID     1       // the same on all nodes that talk to each other
#define GATEWAYID     1       // gateway node id
#define FREQUENCY     RF69_433MHZ
#define ENCRYPTKEY    "encrypt"
#define ACK_TIME      30      // max # of ms to wait for an ack
#define LED           9       // Moteinos have LEDs on D9

#define INTERRUPT     1       // 1 = D3 (for some reason D2 not working)

RFM69 radio;

void setup() {
  // initialise the radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower();
  radio.encrypt(ENCRYPTKEY);

  // configure the input/interrupt pin
  pinMode(3, INPUT);
  attachInterrupt(INTERRUPT, wakeUp, CHANGE);
}

void wakeUp()
{
}

void loop() {
  // will only get here on the first iteration of
  // the processing loop, and after an interrupt
  
  // give the pin some time to settle
  delay(25);
  
  // send the pin value to our gateway node
  byte payload[] = {3, digitalRead(3)};
  if (radio.sendWithRetry(GATEWAYID, payload, sizeof(payload)))
  {
    // blink the LED to indicate we have sent a packet
    blink(LED, 5);
  }
  
  // sleep until the next interrupt
  radio.sleep();
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void blink(byte pin, int delayMs)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(delayMs);
  digitalWrite(pin, LOW);
}
