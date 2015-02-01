//      Code by Robin Emley (calypso_rae on Open Energy Monitor Forum) - September 2013
//      Updated November 2013 to include analog and LED pins for the emonTx V3 by Glyn Hudson
//
//      Updated July 2014 to send readings via MQTT by Ben Jones
//      Updated October 2014 to include PV router functionality by Ben Jones
//      Updated January 2015 to add support for remote config of scale factor (by MQTT)
//
//      The interrupt-based kernel for this sketch was kindly provided by Jorg Becker.

#include <avr/wdt.h>
#include <Arduino.h>         // may not be needed, but it's probably a good idea to include this
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

// serial rate
#define SERIAL_BAUD                9600
// pv router config
#define DATALOG_PERIOD_IN_SECONDS  5
#define CYCLES_PER_SECOND          50
#define SWEETZONE_IN_JOULES        3600 
// to prevent the diverted energy total from 'creeping'
// in Joules per mains cycle (has no effect when set to 0)
#define ANTI_CREEP_LIMIT           5

// 0 is pin HIGH, 1 is pin LOW
#define TRIGGER_ON                 0
#define TRIGGER_OFF                1

#define LED_ON                     1
#define LED_OFF                    0

// We are sending calculated results to an MQTT topic via ethernet

// unique MAC address on our LAN (0x53 => .83)
byte mac[]                 = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x53 };
byte ip[]                  = { 192, 168, 1, 83 };

// MQTT broker connection properties
byte mqttBroker[]          = { 192, 168, 1, 21 };
char mqttClientId[]        = "emontx";
char mqttUsername[]        = "emontx";
char mqttPassword[]        = "password";

// publish to "/emontx/<variable>".
char mqttTopicDataLog[]    = "/emontx";
char mqttTopicPvrConfig[]  = "/emontx/config/+";
char mqttTopicPvrTrigger[] = "/openhab/heating/in/2";
char mqttTopicLwt[]        = "/clients/emontx";
int  mqttLwtQos            = 0;
int  mqttLwtRetain         = 1;

// ethernet shield uses pins 4 for SD card and 10-13 for SPI
EthernetClient ethernet;
PubSubClient mqtt(mqttBroker, 1883, mqtt_callback, ethernet);

typedef struct { 
   int power_CT1;
   int power_CT2;
   int power_CT3;
   int power_CT4; } Tx_struct;
Tx_struct tx_data;

// definition of enumerated types
enum polarities  {NEGATIVE, POSITIVE};
enum outputModes {ANTI_FLICKER, NORMAL};

// The power-diversion logic can operate in either of two modes:
//
// - NORMAL, where the triac switches rapidly on/off to maintain a constant energy level.
// - ANTI_FLICKER, whereby the repetition rate is reduced to avoid rapid fluctuations
//    of the local mains voltage.
enum outputModes outputMode = ANTI_FLICKER;                                                   

// ----------- Pinout assignments  -----------
//
// digital input pins:
// dig pin 0 is for Serial Rx
// dig pin 1 is for Serial Tx
// dig pin 4 is for the ethernet module (IRQ) 
// dig pin 10 is for the ethernet module (SEL) 
// dig pin 11 is for the ethernet module (SDI) 
// dig pin 12 is for the ethernet module (SDO) 
// dig pin 13 is for the ethernet module (CLK) 
const byte ledPin = 9;

// analogue input pins (for emonTx V3): 
const byte voltageSensor = 0;       // analogue
const byte currentSensor_CT1 = 1;   // analogue
const byte currentSensor_CT2 = 2;   // analogue
const byte currentSensor_CT3 = 3;   // analogue 
const byte currentSensor_CT4 = 4;   // analogue 


// --------------  general global variables -----------------
//
// Some of these variables are used in multiple blocks so cannot be static.
// For integer maths, many variables need to be 'long'
//
boolean beyondStartUpPhase = false;    // start-up delay, allows things to settle
const byte startUpPeriod = 3;          // in seconds, to allow LP filter to settle
const int DCoffset_I = 512;            // nominal mid-point value of ADC @ x1 scale

long triggerThreshold_long;            // for determining when the trigger may be safely armed
long capacityOfEnergyBucket_long;      // depends on powerCal, frequency & the 'sweetzone' size.
long lowerEnergyThreshold_long;        // for turning triac off
long upperEnergyThreshold_long;        // for turning triac on

long energyInBucket_long      = 0;     // to record the present level in the energy accumulator for PV diversion
long energyInBucket_long_CT1  = 0;     // to record the present level in the energy accumulator for data logging
long energyInBucket_long_CT2  = 0;     // to record the present level in the energy accumulator for data logging
long energyInBucket_long_CT3  = 0;     // to record the present level in the energy accumulator for data logging
long energyInBucket_long_CT4  = 0;     // to record the present level in the energy accumulator for data logging
int phaseCal_int_CT1;                  // to avoid the need for floating-point maths
int phaseCal_int_CT2;                  // to avoid the need for floating-point maths
int phaseCal_int_CT3;                  // to avoid the need for floating-point maths
int phaseCal_int_CT4;                  // to avoid the need for floating-point maths
long DCoffset_V_long;                  // <--- for LPF
long DCoffset_V_min;                   // <--- for LPF
long DCoffset_V_max;                   // <--- for LPF

// this setting is only used if anti-flicker mode is enabled
float offsetOfEnergyThresholdsInAFmode = 0.1; // <-- must not exceeed 0.5

// for interaction between the main processor and the ISR 
volatile boolean dataReady = false;
int sampleI_CT1;
int sampleI_CT2;
int sampleI_CT3;
int sampleI_CT4;
int sampleV;

unsigned int cycleCount = 0; 

// flag for enabling/disabling the PV router
boolean triggerEnabled = false;
int triggerCurrent = -1;


// Calibration values
//-------------------
// Two calibration values are used in this sketch: powerCal, and phaseCal. 
// With most hardware, the default values are likely to work fine without 
// need for change.  A compact explanation of each of these values now follows:

// When calculating real power, which is what this code does, the individual 
// conversion rates for voltage and current are not of importance.  It is 
// only the conversion rate for POWER which is important.  This is the 
// product of the individual conversion rates for voltage and current.  It 
// therefore has the units of ADC-steps squared per Watt.  Most systems will
// have a power conversion rate of around 20 (ADC-steps squared per Watt).
// 
// powerCal is the RECIPR0CAL of the power conversion rate.  A good value 
// to start with is therefore 1/20 = 0.05 (Watts per ADC-step squared)
//

// Voltage calibration constant:

// AC-AC Voltage adapter is designed to step down the voltage from 230V to 9V
// but the AC Voltage adapter is running open circuit and so output voltage is
// likely to be 20% higher than 9V (9 x 1.2) = 10.8V. 
// Open circuit step down = 230 / 10.8 = 21.3

// The output voltage is then steped down further with the voltage divider which has 
// values Rb = 10k, Rt = 120k (which will reduce the voltage by 13 times.

// The combined step down is therefore 21.3 x 13 = 276.9 which is the 
// theoretical calibration constant entered below.

// NZ voltage = 240
// Output voltage of AC-AC adapter = 11.23
// EmonTx shield voltage divider = 11
// --> ( 240 / 11.23 ) * 11 = 235.1

// Current calibration constant:

// CT ratio / burden resistance for EmonTX shield
// --> (100A / 0.05A) / 33 Ohms = 60.606

// Measured load using the powerCal below with no scale factor
// - first HWC (3kW)  = 3000 / 1126.9 = 2.66217
// - then Enasolar    = 1.71 / 1.78   = 0.96967 = 2.55748 (instantaneous)
// - then Enasolar    = 1.78 / 1.76   = 1.01136 = 2.58654 (instantaneous)
// - then Enasolar    = 6.47 / 6.70   = 0.96567 = 2.49774 (daily total)
// - then spreadsheet =               = 0.97    = 2.42281 (2014-08-27)
// - then spreadsheet =               = 1.00683 = 2.43936 (2014-09-03)
// - then manual      =               = 1.135   = 2.76867 (2015-01-21 - full month report from openHAB vs Meridian)

// default/startup values
float scaleFactor_CT1 = 2.76867;  // Grid
float scaleFactor_CT2 = 2.76867;  // Solar (Enasolar output)
float scaleFactor_CT3 = 2.76867;  // HWC (3kW)
float scaleFactor_CT4 = 2.76867;  // UFHP

// phaseCal is used to alter the phase of the voltage waveform relative to the
// current waveform.  The algorithm interpolates between the most recent pair
// of voltage samples according to the value of phaseCal. 
//
//    With phaseCal = 1, the most recent sample is used.  
//    With phaseCal = 0, the previous sample is used
//    With phaseCal = 0.5, the mid-point (average) value in used
//
// NB. Any tool which determines the optimal value of phaseCal must have a similar 
// scheme for taking sample values as does this sketch!
// http://openenergymonitor.org/emon/node/3792#comment-18683
const float phaseCal_CT1 = 0.2;
const float phaseCal_CT2 = 0.4;
const float phaseCal_CT3 = 0.6;
const float phaseCal_CT4 = 0.8;

int datalogCountInMainsCycles;
const int maxDatalogCountInMainsCycles = DATALOG_PERIOD_IN_SECONDS * CYCLES_PER_SECOND;
float normalisation_CT1;
float normalisation_CT2;
float normalisation_CT3;
float normalisation_CT4;

boolean LED_pulseInProgress = false;
unsigned long LED_onAt;

void setup() {  
  // ensure the watchdog is disabled
  wdt_disable();

  // initialise the serial interface  
  Serial.begin(SERIAL_BAUD);
  
  // dump the sketch details
  Serial.println("-------------------------------------");
  Serial.println("Sketch ID:        emontx_pvrouter.ino");
  Serial.println("-------------------------------------");
  
  // setup indicator LED
  pinMode(ledPin, OUTPUT);
       
  // initialise the SPI bus
  SPI.begin();

  // initialise the ethernet module with our MAC and IP addresses
  Serial.print("Initialising ethernet connection...");
  Ethernet.begin(mac, ip);
  Serial.println("done");
  
  // attempt to connect to our MQTT broker
  mqttConnect();

  // ensure the trigger is OFF
  updateTrigger(TRIGGER_OFF);
  
  // when using integer maths, calibration values that have supplied in floating point 
  // form need to be rescaled.  
  phaseCal_int_CT1 = phaseCal_CT1 * 256; // for integer maths
  phaseCal_int_CT2 = phaseCal_CT2 * 256; // for integer maths
  phaseCal_int_CT3 = phaseCal_CT3 * 256; // for integer maths
  phaseCal_int_CT4 = phaseCal_CT4 * 256; // for integer maths

  // define operating limits for the LP filter which identifies DC offset in the voltage 
  // sample stream.  By limiting the output range, the filter always should start up 
  // correctly.
  DCoffset_V_long = 512L * 256; // nominal mid-point value of ADC @ x256 scale  
  DCoffset_V_min = (long)(512L - 100) * 256; // mid-point of ADC minus a working margin
  DCoffset_V_max = (long)(512L + 100) * 256; // mid-point of ADC plus a working margin

  // in this sketch, the ADC is free-running with a cycle time of ~104uS.
  Serial.println("ADC mode:       free-running");
  
  // set up the ADC to be free-running 
  ADCSRA  = (1<<ADPS0)+(1<<ADPS1)+(1<<ADPS2);  // Set the ADC's clock to system clock / 128
  ADCSRA |= (1 << ADEN);                       // Enable the ADC 
  
  ADCSRA |= (1<<ADATE);  // set the Auto Trigger Enable bit in the ADCSRA register.  Because 
                         // bits ADTS0-2 have not been set (i.e. they are all zero), the 
                         // ADC's trigger source is set to "free running mode".
                         
  ADCSRA |=(1<<ADIE);    // set the ADC interrupt enable bit. When this bit is written 
                         // to one and the I-bit in SREG is set, the 
                         // ADC Conversion Complete Interrupt is activated. 

  ADCSRA |= (1<<ADSC);   // start ADC manually first time 
  sei();                 // Enable Global Interrupts  

  // update all our config parameters, using the default/startup values  
  updateConfigParameters();
  
  Serial.print("Anti-creep limit (Joules / mains cycle) = ");
  Serial.println(ANTI_CREEP_LIMIT);  
  Serial.println("----");    

  // enable the watchdog timer - 8s timeout
  wdt_enable(WDTO_8S);
  wdt_reset();
}

// An Interrupt Service Routine is now defined in which the ADC is instructed to perform 
// a conversion of the voltage signal and each of the signals for current.  A "data ready" 
// flag is set after each voltage conversion has been completed, it being the last one in
// the sequence.  
//   Samples for current are taken first because the phase of the waveform for current is 
// generally slightly advanced relative to the waveform for voltage.  The data ready flag 
// is cleared within loop().

// This Interrupt Service Routine is for use when the ADC is in the free-running mode.
// It is executed whenever an ADC conversion has finished, approx every 104 us.  In 
// free-running mode, the ADC has already started its next conversion by the time that
// the ISR is executed.  The ISR therefore needs to "look ahead". 
//   At the end of conversion Type N, conversion Type N+1 will start automatically.  The ISR 
// which runs at this point therefore needs to capture the results of conversion Type N , 
// and set up the conditions for conversion Type N+2, and so on.  
// 
ISR(ADC_vect) {                                         
  static unsigned char sample_index = 0;
  
  switch (sample_index) {
    case 0:
      sampleV = ADC; 
      ADMUX = 0x40 + currentSensor_CT2; // set up the next-but-one conversion
      sample_index++; // advance the control flag             
      dataReady = true; 
      break;
    case 1:
      sampleI_CT1 = ADC; 
      ADMUX = 0x40 + currentSensor_CT3; // for the next-but-one conversion
      sample_index++; // advance the control flag                
      break;
    case 2:
      sampleI_CT2 = ADC; 
      ADMUX = 0x40 + currentSensor_CT4; // for the next-but-one conversion
      sample_index++; // advance the control flag                
      break;
    case 3:
      sampleI_CT3 = ADC; 
      ADMUX = 0x40 + voltageSensor; // for the next-but-one conversion
      sample_index++; // advance the control flag                 
      break;
    case 4:
      sampleI_CT4 = ADC; 
      ADMUX = 0x40 + currentSensor_CT1; // for the next-but-one conversion
      sample_index = 0; // reset the control flag                
      break;
    default:
      sample_index = 0;                 // to prevent lockup (should never get here)      
  }  
}


// When using interrupt-based logic, the main processor waits in loop() until the 
// dataReady flag has been set by the ADC.  Once this flag has been set, the main
// processor clears the flag and proceeds with all the processing for one pair of 
// V & I samples.  It then returns to loop() to wait for the next pair to become 
// available.
//   If the next pair of samples become available before the processing of the 
// previous pair has been completed, data could be lost.  This situation can be 
// avoided by prior use of the WORKLOAD_CHECK mode.  Using this facility, the amount
// of spare processing capacity per loop can be determined.  
//
void loop() { 
  // reset the watchdog timer
  wdt_reset();

  // flag is set after every pair of ADC conversions
  if (dataReady) {
    // reset the flag
    dataReady = false;
    // executed once for each pair of V&I samples
    allGeneralProcessing();
  }
}

// MQTT call back to allow the PV router to be configured
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // assume payload will be at most 16 chars
  char message[16];
  strncpy(message, (char*)payload, length);
  
  // tokenise the topic
  char * topictoken;
  topictoken = strtok(topic, "/");

  // junk the first two tokens - i.e. /emontx/pvrouter/+
  topictoken = strtok(NULL, "/");
  topictoken = strtok(NULL, "/");

  // listen for control messages
  if (strcmp(topictoken, "scalefactor") == 0) {
    scaleFactor_CT1 = atof(message);
    scaleFactor_CT2 = scaleFactor_CT1;
    scaleFactor_CT3 = scaleFactor_CT1;
    scaleFactor_CT4 = scaleFactor_CT1;
    // reconfigure the various scale/power factors
    updateConfigParameters();
  } else if (strcmp(topictoken, "pvrmode") == 0) {
    // update the output mode
    if (strncmp(message, "antiflicker", 11) == 0) {
      outputMode = ANTI_FLICKER;
    } else if (strncmp(message, "normal", 6) == 0) {
      outputMode = NORMAL;
    } else {
      Serial.print("Ignoring invalid output mode: ");
      Serial.println(message);
    }
    // reconfigure the various scale/power factors
    updateConfigParameters();
  } else if (strcmp(topictoken, "pvrstatus") == 0) {
    // update the trigger state to enable/disable PV routing
    triggerEnabled = (strncmp(message, "1", 1) == 0) || (strncmp(message, "on", 2) == 0);
    if (triggerEnabled) { 
      Serial.println("Trigger enabled");
    } else { 
      Serial.println("Trigger disabled"); 
    }
  } else {
    Serial.print("Unsupported config option: ");
    Serial.println(topictoken);
  }
}

boolean mqttConnect() {
  Serial.print("Attempting to connect to MQTT broker as ");
  Serial.print(mqttUsername);
  Serial.print("...");
  
  // attempt to connect to the broker and setup our subscriptions etc
  boolean success = mqtt.connect(mqttClientId, mqttUsername, mqttPassword, mqttTopicLwt, mqttLwtQos, mqttLwtRetain, "0"); 
  if (success) {
    Serial.println("success");
    // subscribe to our config topics
    mqtt.subscribe(mqttTopicPvrConfig);
    // publish retained LWT so anything listening knows we are alive
    byte data[] = { '1' };
    mqtt.publish(mqttTopicLwt, data, 1, mqttLwtRetain);
  } else {
    Serial.println("failed");
  }
  return success;
}

// This routine is called to process each pair of V & I samples.  Note that when using 
// interrupt-based code, it is not necessary to delay the processing of each pair of 
// samples as was done in Mk2a builds.  This is because there is no longer a strict 
// alignment between the obtaining of each sample by the ADC and the processing that can 
// be done by the main processor while the ADC conversion is in progress.  
//   When interrupts are used, the main processor and the ADC work autonomously, their
// operation being only linked via the dataReady flag.  As soon as data is made available
// by the ADC, the main processor can start to work on it immediately.  
//
void allGeneralProcessing() {
  static boolean triggerNeedsToBeArmed = false;  // once per mains cycle (+ve half)
  static int samplesDuringThisCycle;             // for normalising the power in each mains cycle
  static long sumP_CT1;                          // for per-cycle summation of 'real power' 
  static long sumP_CT2;
  static long sumP_CT3;
  static long sumP_CT4;
  static enum polarities polarityOfLastSampleV;  // for zero-crossing detection
  static long cumVdeltasThisCycle_long;          // for the LPF which determines DC offset (voltage)
  static long lastSampleVminusDC_long;           // for the phaseCal algorithm
  static byte whenToSendCount = 0;
  
  // remove DC offset from the raw voltage sample by subtracting the accurate value 
  // as determined by a LP filter.
  long sampleVminusDC_long = ((long)sampleV<<8) - DCoffset_V_long; 

  // determine polarity, to aid the logical flow
  enum polarities polarityNow;   
  if (sampleVminusDC_long > 0) { 
    polarityNow = POSITIVE; 
  }else { 
    polarityNow = NEGATIVE; 
  }

  if (polarityNow == POSITIVE) 
  {                           
    if (beyondStartUpPhase)
    {  
      if (polarityOfLastSampleV != POSITIVE)
      {
        // This is the start of a new +ve half cycle (just after the zero-crossing point)
        cycleCount++;

        // the trigger is armed once during each +ve half-cycle 
        triggerNeedsToBeArmed = true;
        
        // Calculate the real power and energy during the last whole mains cycle.
        //
        // sumP contains the sum of many individual calculations of instantaneous power.  In  
        // order to obtain the average power during the relevant period, sumP must first be 
        // divided by the number of samples that have contributed to its value.
        //
        // The next stage would normally be to apply a calibration factor so that real power 
        // can be expressed in Watts.  That's fine for floating point maths, but it's not such
        // a good idea when integer maths is being used.  To keep the numbers large, and also 
        // to save time, calibration of power is omitted at this stage.  realPower_long is 
        // therefore (1/powerCal) times larger than the actual power in Watts.
        //
        long realPower_long_CT1 = sumP_CT1 / samplesDuringThisCycle; // proportional to Watts
        long realPower_long_CT2 = sumP_CT2 / samplesDuringThisCycle; // proportional to Watts
        long realPower_long_CT3 = sumP_CT3 / samplesDuringThisCycle; // proportional to Watts
        long realPower_long_CT4 = sumP_CT4 / samplesDuringThisCycle; // proportional to Watts
   
        // Next, the energy content of this power rating needs to be determined.  Energy is 
        // power multiplied by time, so the next step is normally to multiply by the time over 
        // which the power was measured.
        //   Instanstaneous power is calculated once every mains cycle, so that's every fiftieth 
        // of a second.  When integer maths is being used, this routine power-to-energy conversion 
        // seems an unnecessary workload.  As all sampling periods are of similar duration (20mS), 
        // it is more efficient simply to add all the power samples together, and note that their 
        // sum is actually 50 times greater than it would otherwise be.
        //   Although the numerical value itself does not change, a new name is helpful so as  
        // to avoid any confusion.  The 'energy' variable below is 50 * (1/powerCal) times larger 
        // than the actual energy in Joules.
        //
        long realEnergy_long_CT1 = realPower_long_CT1; 
        long realEnergy_long_CT2 = realPower_long_CT2; 
        long realEnergy_long_CT3 = realPower_long_CT3; 
        long realEnergy_long_CT4 = realPower_long_CT4; 
        
        // Add this latest contribution to the energy bucket
        energyInBucket_long_CT1 += realEnergy_long_CT1;
        energyInBucket_long_CT2 += realEnergy_long_CT2;
        energyInBucket_long_CT3 += realEnergy_long_CT3;
        energyInBucket_long_CT4 += realEnergy_long_CT4;
        
        // Apply max and min limits to bucket's level.  This is to ensure correct operation
        // when conditions change, i.e. when import changes to export, and vici versa.
        // copy the value from CT1 to our grid accumulator - which we use for PV diversion
        energyInBucket_long += (realEnergy_long_CT1 * -1);
        if (energyInBucket_long > capacityOfEnergyBucket_long) { 
          energyInBucket_long = capacityOfEnergyBucket_long; 
        } else if (energyInBucket_long < 0) {
          energyInBucket_long = 0; 
        }  
        
        datalogCountInMainsCycles++;
        
        if (datalogCountInMainsCycles >= maxDatalogCountInMainsCycles)
        {         
          tx_data.power_CT1 = energyInBucket_long_CT1 * normalisation_CT1;
          tx_data.power_CT2 = energyInBucket_long_CT2 * normalisation_CT2;
          tx_data.power_CT3 = energyInBucket_long_CT3 * normalisation_CT3;
          tx_data.power_CT4 = energyInBucket_long_CT4 * normalisation_CT4;          
          
          send_data();
          
          datalogCountInMainsCycles = 0;
          energyInBucket_long_CT1 = 0;         
          energyInBucket_long_CT2 = 0;         
          energyInBucket_long_CT3 = 0;         
          energyInBucket_long_CT4 = 0;            
        }

        // clear the per-cycle accumulators for use in this new mains cycle.  
        samplesDuringThisCycle = 0;
        sumP_CT1 = 0;
        sumP_CT2 = 0;
        sumP_CT3 = 0;
        sumP_CT4 = 0;

      } // end of processing that is specific to the first Vsample in each +ve half cycle   

      // still processing samples where the voltage is POSITIVE ...
      if (triggerNeedsToBeArmed == true) {
        // check to see whether the trigger device can now be reliably armed
        // much easier than checking the voltage level
        if (samplesDuringThisCycle == 3) {
          // check if the trigger is enabled
          if (triggerEnabled) {
            // only update the trigger if we are outside the hysteresis range
            if (energyInBucket_long < lowerEnergyThreshold_long) {
              updateTrigger(TRIGGER_OFF);
            } else if (energyInBucket_long > upperEnergyThreshold_long) {
              updateTrigger(TRIGGER_ON);
            }
          } else {
            // if the trigger is not enabled then ensure it is OFF
            updateTrigger(TRIGGER_OFF);
          }     

          // clear the flag
          triggerNeedsToBeArmed = false;  
        }
      } 
    }
    else
    {  
      // wait until the DC-blocking filters have had time to settle
      if (millis() > startUpPeriod * 1000) {
        beyondStartUpPhase = true;
        sumP_CT1 = 0;
        sumP_CT2 = 0;
        sumP_CT3 = 0;
        sumP_CT4 = 0;
        samplesDuringThisCycle = 0;
        Serial.println ("Go!");
      }
    }
  } // end of processing that is specific to samples where the voltage is positive
  
  else // the polarity of this sample is negative
  {     
    if (polarityOfLastSampleV != NEGATIVE)
    {
      // This is the start of a new -ve half cycle (just after the zero-crossing point)
      //
      // This is a convenient point to update the Low Pass Filter for DC-offset removal,
      // which needs to be done right from the start.
      long previousOffset = DCoffset_V_long;
      DCoffset_V_long = previousOffset + (cumVdeltasThisCycle_long>>6); // faster than * 0.01
      cumVdeltasThisCycle_long = 0;
      
      // To ensure that the LPF will always start up correctly when 240V AC is available, its
      // output value needs to be prevented from drifting beyond the likely range of the 
      // voltage signal.  This avoids the need to use a HPF as was done for initial Mk2 builds.
      if (DCoffset_V_long < DCoffset_V_min) {
        DCoffset_V_long = DCoffset_V_min; 
      } else if (DCoffset_V_long > DCoffset_V_max) {
        DCoffset_V_long = DCoffset_V_max; 
      }
        
      // Turn off the LED if it has been on for long enough
      check_LED_status();
    } // end of processing that is specific to the first Vsample in each -ve half cycle
  } // end of processing that is specific to samples where the voltage is positive
  
  // Processing for EVERY pair of samples. Most of this code is not used during the 
  // start-up period, but it does no harm to leave it in place.  Accumulated values 
  // are cleared when beyondStartUpPhase is set to true.
  //
  // remove most of the DC offset from the current sample (the precise value does not matter)
  long sampleIminusDC_long_CT1 = ((long)(sampleI_CT1-DCoffset_I))<<8;
  long sampleIminusDC_long_CT2 = ((long)(sampleI_CT2-DCoffset_I))<<8;
  long sampleIminusDC_long_CT3 = ((long)(sampleI_CT3-DCoffset_I))<<8;
  long sampleIminusDC_long_CT4 = ((long)(sampleI_CT4-DCoffset_I))<<8;
  
  // phase-shift the voltage waveform so that it aligns with the current when a 
  // resistive load is used
  long  phaseShiftedSampleVminusDC_long_CT1 = lastSampleVminusDC_long
         + (((sampleVminusDC_long - lastSampleVminusDC_long)*phaseCal_int_CT1)>>8);  
  long  phaseShiftedSampleVminusDC_long_CT2 = lastSampleVminusDC_long
         + (((sampleVminusDC_long - lastSampleVminusDC_long)*phaseCal_int_CT2)>>8);  
  long  phaseShiftedSampleVminusDC_long_CT3 = lastSampleVminusDC_long
         + (((sampleVminusDC_long - lastSampleVminusDC_long)*phaseCal_int_CT3)>>8);  
  long  phaseShiftedSampleVminusDC_long_CT4 = lastSampleVminusDC_long
         + (((sampleVminusDC_long - lastSampleVminusDC_long)*phaseCal_int_CT4)>>8);  
  
  // calculate the "real power" in this sample pair and add to the accumulated sum
  long filtV_div4_CT1 = phaseShiftedSampleVminusDC_long_CT1>>2;  // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4_CT1 = sampleIminusDC_long_CT1>>2; // reduce to 16-bits (now x64, or 2^6)
  long instP_CT1 = filtV_div4_CT1 * filtI_div4_CT1;  // 32-bits (now x4096, or 2^12)
  instP_CT1 = instP_CT1>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)       
  sumP_CT1 +=instP_CT1; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  
  long filtV_div4_CT2 = phaseShiftedSampleVminusDC_long_CT2>>2;  // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4_CT2 = sampleIminusDC_long_CT2>>2; // reduce to 16-bits (now x64, or 2^6)
  long instP_CT2 = filtV_div4_CT2 * filtI_div4_CT2;  // 32-bits (now x4096, or 2^12)
  instP_CT2 = instP_CT2>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)       
  sumP_CT2 +=instP_CT2; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  
  long filtV_div4_CT3 = phaseShiftedSampleVminusDC_long_CT3>>2;  // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4_CT3 = sampleIminusDC_long_CT3>>2; // reduce to 16-bits (now x64, or 2^6)
  long instP_CT3 = filtV_div4_CT3 * filtI_div4_CT3;  // 32-bits (now x4096, or 2^12)
  instP_CT3 = instP_CT3>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)       
  sumP_CT3 +=instP_CT3; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  
  long filtV_div4_CT4 = phaseShiftedSampleVminusDC_long_CT4>>2;  // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4_CT4 = sampleIminusDC_long_CT4>>2; // reduce to 16-bits (now x64, or 2^6)
  long instP_CT4 = filtV_div4_CT4 * filtI_div4_CT4;  // 32-bits (now x4096, or 2^12)
  instP_CT4 = instP_CT4>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)       
  sumP_CT4 +=instP_CT4; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  
  samplesDuringThisCycle++;
  
  // store items for use during next loop
  cumVdeltasThisCycle_long += sampleVminusDC_long; // for use with LP filter
  lastSampleVminusDC_long = sampleVminusDC_long;  // required for phaseCal algorithm
  
  polarityOfLastSampleV = polarityNow;  // for identification of half cycle boundaries
}
// end of loop()

void updateConfigParameters() {
  Serial.println("Updating config parameters:");
  Serial.print("  scaleFactor_CT1             =  "); Serial.println(scaleFactor_CT1,4);
  Serial.print("  scaleFactor_CT2             =  "); Serial.println(scaleFactor_CT2,4);
  Serial.print("  scaleFactor_CT3             =  "); Serial.println(scaleFactor_CT3,4);
  Serial.print("  scaleFactor_CT4             =  "); Serial.println(scaleFactor_CT4,4);
  
  // calculate the power calibration factors
  float powerCal_CT1 = (235.1*(3.3/1023))*(60.606*(3.3/1023))*scaleFactor_CT1;
  float powerCal_CT2 = (235.1*(3.3/1023))*(60.606*(3.3/1023))*scaleFactor_CT2;
  float powerCal_CT3 = (235.1*(3.3/1023))*(60.606*(3.3/1023))*scaleFactor_CT3;
  float powerCal_CT4 = (235.1*(3.3/1023))*(60.606*(3.3/1023))*scaleFactor_CT4;

  // for the flow of energy at the 'grid' connection point (CT1) 
  capacityOfEnergyBucket_long = (long)SWEETZONE_IN_JOULES * CYCLES_PER_SECOND * (1/powerCal_CT1);
  
  // calculate the normalisation factor to actually apply to our readings
  normalisation_CT1 = powerCal_CT1 / maxDatalogCountInMainsCycles;
  normalisation_CT2 = powerCal_CT2 / maxDatalogCountInMainsCycles;
  normalisation_CT3 = powerCal_CT3 / maxDatalogCountInMainsCycles;
  normalisation_CT4 = powerCal_CT4 / maxDatalogCountInMainsCycles;
  
  // calculate the energy bucket thresholds for use by the PV diverter
  Serial.print("Output mode:    ");
  if (outputMode == ANTI_FLICKER)
  {
    Serial.println("anti-flicker");
    Serial.print("  offsetOfEnergyThresholds    = ");
    Serial.println(offsetOfEnergyThresholdsInAFmode); 

    // settings for anti-flicker mode
    lowerEnergyThreshold_long = 
      capacityOfEnergyBucket_long * (0.5 - offsetOfEnergyThresholdsInAFmode); 
    upperEnergyThreshold_long = 
      capacityOfEnergyBucket_long * (0.5 + offsetOfEnergyThresholdsInAFmode);   
  }
  else
  { 
    Serial.println("normal"); 

    // settings for normal mode
    lowerEnergyThreshold_long = 
      capacityOfEnergyBucket_long * 0.5; 
    upperEnergyThreshold_long = 
      capacityOfEnergyBucket_long * 0.5;   
  }
  
  // display relevant settings for selected output mode
  Serial.print("  capacityOfEnergyBucket_long = ");
  Serial.println(capacityOfEnergyBucket_long);
  Serial.print("  lowerEnergyThreshold_long   = ");
  Serial.println(lowerEnergyThreshold_long);
  Serial.print("  upperEnergyThreshold_long   = ");
  Serial.println(upperEnergyThreshold_long);
}

void send_data() {
  // check our DHCP lease is still ok
  Ethernet.maintain();

  // process any MQTT messages - will return false if not connected
  // so attempt to reconnect if required
  if (mqtt.loop() || mqttConnect()) {  
    publishReading("grid", tx_data.power_CT1);
    publishReading("solar", tx_data.power_CT2);
    publishReading("ufhp", tx_data.power_CT3);
    publishReading("hwc", tx_data.power_CT4);
    publishReading("house", tx_data.power_CT1 + tx_data.power_CT2);
  
    Serial.print("CT1 = ");
    Serial.print(tx_data.power_CT1);
    Serial.print(", CT2 = ");
    Serial.print(tx_data.power_CT2);
    Serial.print(", CT3 = ");
    Serial.print(tx_data.power_CT3);
    Serial.print(", CT4 = ");
    Serial.print(tx_data.power_CT4);
    Serial.println();
  
    digitalWrite(ledPin, LED_ON);
    LED_pulseInProgress = true;
    LED_onAt = cycleCount;
  }
}

void check_LED_status() {
  if (LED_pulseInProgress == true) {
    // pulse duration = 40 ms
    if (cycleCount > (LED_onAt + 2)) {
      digitalWrite(ledPin, LED_OFF); 
      LED_pulseInProgress = false; 
    }
  }
}  

void updateTrigger(int triggerValue) {
  // only publish an update if the trigger state has changed
  if (triggerValue != triggerCurrent) {
    publishData(mqttTopicPvrTrigger, triggerValue);
    triggerCurrent = triggerValue;
  }
}

void publishReading(char * name, int reading) {
  // build the MQTT topic
  char topic[32];
  snprintf(topic, 32, "%s/%s", mqttTopicDataLog, name);
  
  publishData(topic, reading);
}

void publishData(char * topic, int data) {
  // build the MQTT payload
  char payload[16];
  snprintf(payload, 16, "%i", data);

  // publish to the MQTT broker 
  mqtt.publish(topic, payload);
}
