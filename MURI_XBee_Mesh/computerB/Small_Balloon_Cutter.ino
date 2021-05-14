//============================================================================================================================================
// MURI Resistor Cutter Box B (for Small Balloon)
// Written by Steele Mitchell and PJ Collins - mitc0596 & coll0792 Spring 2020
// XBee mesh written by Paul Wehling - wehli007 Fall 2020

// Last updated: 5/14/21
// Changes made: thorough comments
//============================================================================================================================================
//
// Part of the Double Ballooning System. This is a small resistor cutaway payload set just below the large balloon.
// Will superheat a resistor to cut the tether when instructed, or can operate autonomously according to its state machine.
// Documentation: MURI > BOLT Campaign > Double-Resistor Cutter > Double-Resistor Cutter System Overview:
//                https://docs.google.com/document/d/1AktxL2UisbX6VzWkeL9jIVlEu8cipdC7h6uNuxK32Yo/edit#heading=h.7wkmqyx5nh8u
//
//=============================================================================================================================================

/*  Arduino Uno w/ PCB Shield pin connections:
     ----------------------------------------------
    | Component                    | Pins used     |    
    | ---------------------------------------------|
    | UBlox Neo m8n                | 0,1           |
    | LEDs                         | 7,16          | 
    | Xbee                         | 8,9           |
    | H-Drivers                    | 2,3,10,11     |
    | Latching Relay               | 12,13         |
    | Thermistor                   | A0            | 
     ----------------------------------------------
*/

#define SERIAL_BUFFER_SIZE 32

// Libraries
#include <SPI.h>
#include <SoftwareSerial.h>
#include <UbloxGPS.h>
#include <LatchRelay.h> 
#include <Arduino.h>
#include <RelayXBee.h>

 
// Pin Definitions
#define UBLOX_RX 0
#define UBLOX_TX 1
#define LED 7
#define LED2 16
#define Xbee_RX 9
#define Xbee_TX 8
#define CUTTER_PIN1 11 
#define CUTTER_PIN2 10 // this pin needs to be connected and initialized, but will be inactive
#define CUTTER_PIN3 2 
#define CUTTER_PIN4 3 // this pin needs to be connected and initialized, but will be inactive
#define HEAT_ON 12
#define HEAT_OFF 13
#define THERMISTOR_A A0
// #define VOLT_PIN A5 // for voltage reading - not set up yet

// Radio Parameters
#define xbeeChannel 'C'                             //Shorthand for a communication channel preset (See RelayXBee library for specifics)
#define CDU_RX_SIZE 30                              //Number of bytes in CDU packet
#define CDU_TX_SIZE 7                               //Number of bytes in a gondola packet
#define waitForReconnection 30                      //Amount of time (secs) of radio inactivity before the cutter starts listening for a new connection
#define xbeeID "MURI"                               //Initialization ID for all mesh members


// Constants
#define PA_TO_ATM 1/101325              // PSI to ATM conversion ratio
#define SEA_LEVEL_PSI 14.7              // average sea level pressure in PSI
#define M2MS 60000                      // milliseconds per minute
#define SIZE 10                         // size of arrays that store values
#define D2R PI/180                      // degrees to radians conversion
#define R2D 180/PI                      // radians to degrees conversion
#define SECONDS_PER_HOUR 3600           // seconds per hour
#define FPM_PER_MPH 88                  // feet per minute per mile per hour
#define FEET_PER_METER 3.28084          // feet per meter


// Intervals
#define MASTER_TIMER 180*M2MS       // 180 standard
#define ASCENT_TIMER 150*M2MS       // 150 standard
#define SA_TIMER 20*M2MS            // 20 standard
#define FLOAT_TIMER 30*M2MS         // 30 standard
#define SLOW_DESCENT_TIMER 40*M2MS  // 40 standard
#define INITIALIZATION_TIME 25*M2MS // 25 standard 
#define DEFAULT_TIME 30*M2MS        // 30 standard
#define LED_INTERVAL 1000               // LEDs run on a 1 second loop to indicate lack of connection
#define UPDATE_INTERVAL 1000            // update all data and the state machine every 1 second
#define CUT_INTERVAL 2*M2MS              // ensure the cutting mechanism is on for 2 minutes


// Fix statuses
#define NOFIX 0x00
#define FIX 0x01


// Boundaries
#define ALTITUDE_FLOOR 5000 // standard 5000
#define ALTITUDE_CEILING 100000 // standard 100000
#define SA_FLOOR 50000 // standard 50000
#define SLOW_DESCENT_FLOOR 80000 // standard 80000
////change lat and long boundaries before every flight!!!////
#define EASTERN_BOUNDARY -92.8
#define WESTERN_BOUNDARY -94.8
#define SOUTHERN_BOUNDARY 43.6
#define NORTHERN_BOUNDARY 44.4
/////////////////////////////////////////////////////////////
#define MIN_TEMP -60                    // minimum acceptable internal temperature
#define MAX_TEMP 90                     // maximum acceptable interal temperature
#define LOW_TEMP -5                     // activation temp for heating pads
#define HIGH_TEMP 5                     // deactivation temp for heating pads
//#define MINIMUM_VOLTAGE 0             // minimum acceptable voltage available

// Velocity Boundaries
#define MAX_SA_RATE 375                 // maximum velocity (ft/min) that corresponds to a slow ascent state
#define MAX_FLOAT_RATE 100              // maximum velocity that corresponds to a float state, or minimum for a slow ascent state
#define MIN_FLOAT_RATE -100             // minimum velocity that corresponds to a float state, or maximum for a slow descent state
#define MIN_SD_RATE -600                // minimum velocity that corresponds to a slow desent state

// States
#define INITIALIZATION 0x00
#define ASCENT 0x01
#define SLOW_ASCENT 0x02
#define FLOAT 0x03
#define SLOW_DESCENT 0x04
#define DESCENT 0x05
#define TEMP_FAILURE 0x06
#define BATTERY_FAILURE 0x07
#define OUT_OF_BOUNDS 0x08
#define PAST_TIMER 0x09
#define ERROR_STATE 0x10

//Thermistor
#define C2K 273.15
#define ADC_MAX 1024                                                    // The maximum adc value given to the thermistor, should be 8196 for a teensy and 1024 for an Arduino
#define CONST_A 0.001125308852122
#define CONST_B 0.000234711863267                                       // A, B, and C are constants used for a 10k resistor and 10k thermistor for the steinhart-hart equation
#define CONST_C 0.000000085663516                                       // NOTE: These values change when the thermistor and/or resistor change value, so if that happens, more research needs to be done on those constants
#define CONST_R 10000        
float t1 = -127.00;                                                    //Temperature initialization values

// Time Stamps
unsigned long updateStamp = 0, restartStamp =0;
unsigned long cutStampA = 0,  cutStampB = 0;  
unsigned long LEDOnStamp = 0, LED2OnStamp = 0;
unsigned long LEDOffStamp = 0, LED2OffStamp = 0;
unsigned long slowBuffer = 0;
unsigned long testStamp = 0;

// State Machine
struct DetData{ // proposed struct filled out by Determination function
  float alt;
  float latitude;
  float longitude;
  float AR;
  float pressure;
  float Time; 
  uint8_t Usage; // 00 means using timer (error on all others), 01 means using GPS, 05 using linear progression
} detData;

uint8_t ascentCounter = 0, SAcounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0;
uint8_t tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0, initCounter = 1;
unsigned long ascentStamp = 0, SAstamp = 0, floatStamp = 0, SDstamp = 0, descentStamp = 0, defaultStamp = 0, defaultStamp2, defaultStampCutA = 0;

uint8_t currentState = INITIALIZATION; // state we are in, starts as initialization
uint8_t stateSuggest; // state recommended by Control function
uint8_t cutReasonB = 0x22; // 0x22 indicates no cut
uint8_t cutStatusB = 0x01; // 1 for false (no cut), 2 for true (cut)
bool cutterOnB = false;    // variable to activate resistor cut

// GPS Variables
UbloxGPS gps(&Serial);
float alt[SIZE];                  // altitude in feet, also there exists a queue library we can use instead
unsigned long timeStamp[SIZE];    // time stamp array that can be used with alt array to return a velocity
float latitude[SIZE];
float longitude[SIZE];
bool fixFlag[SIZE];
uint8_t fixStatus[10];
float ascentRate;
float groundSpeed;
float heading;
uint8_t sats;
bool LEDOn = false, LED2On = false;

struct GPSData{ // proposed struct filled out by compareGPS
  float alt;
  float latitude;
  float longitude;
  float AR;
} GPSdata;

// active heating variables
float sensTemp;
bool coldSensor = false;
LatchRelay sensorHeatRelay(HEAT_ON,HEAT_OFF);        //Declare latching relay objects and related logging variables
String sensorHeat_Status = "";
uint8_t heatStatus = 0x00;

// Xbee comms variables and packets
SoftwareSerial xbeeSerial(Xbee_RX, Xbee_TX);        //Initializes the xbee serial over non-Serial pins
RelayXBee xbee = RelayXBee(&xbeeSerial,xbeeID);     //Creates XBee Object
byte cutterBIdentifier = 0x02;                      //Holds the gondola's individual ID, set during xbeeStartup()
byte connectedG = 0x00;                             //ID of the gondola the cutter is connected to
unsigned long downtimeG = 0;                                //Holds the millis time (secs) of last successful transmission. If goes over 30 secs since last, cutter resets name and listens for gondola
struct data{                                 // FOR OUTGOING DATA
  uint8_t startByte;
  uint8_t cutterTag;
  uint8_t hrs = 0, mins = 0, secs = 0;
  float latitude;
  float longitude;
  float Altitude;
  float t1;
  bool heatStatus;
  uint8_t currentState;
  uint8_t suggestedState;
  bool autonomous;
  uint8_t cutStatus;
  uint8_t cutReason;
  uint16_t checksum;
  uint8_t stopByte;
}dataPacket;                                 // shortcut to create data object dataPacket

struct input{                                // FOR INCOMING DATA
  uint8_t startByte;
  uint8_t cutterTag;
  uint8_t fixStatus;
  uint8_t command;
  uint16_t checksum;
  uint8_t stopByte;
}inputPacket;

// Autonomous operation variables
unsigned long timeOut;
bool autonomousNow = false;   // autonomous mode initialized as off
uint8_t counterZero = 0; 
uint8_t counterOne = 0;
uint8_t prevCommand;
int once = 1;

void setup() {
  Serial.begin(9600);   // initialize serial monitor

  xbeeStartup(); // initialize xbee serial communication

  initGPS();            // initialze GPS

  initRelays();        //Initialize Relays

  pinMode(LED,OUTPUT); // initialize LEDs
  pinMode(LED2,OUTPUT);

  pinMode(CUTTER_PIN1,OUTPUT); // Initialize H-drivers (2 and 4 must be initialized but will not be used)
  pinMode(CUTTER_PIN2,OUTPUT);
  pinMode(CUTTER_PIN3,OUTPUT);
  pinMode(CUTTER_PIN4,OUTPUT);

}

void loop() {

  gps.update(); // update GPS reading

  if(millis() - updateStamp > UPDATE_INTERVAL) {   // does the following every update interval
    updateStamp = millis(); // marks latest update

    updateTemperatures(); // update temperatures
 
    actHeat();          //Controls active heating

    updateTelemetry();  // update GPS data (for decision making)

    Determination();
    Control();
    State(); // these three update the state machine
    
    Serial.print(F("State: "));
    printState();

    Serial.print(F("Lat & long: "));
    Serial.print(latitude[0]);
    Serial.print(F(", "));
    Serial.println(longitude[0]);
    Serial.print(F("Temp: "));
    Serial.println(t1);
     
    sendData();         // send current data to main gondola

    updateXbee();       // gets data from Gondola
    
     if( cutterBIdentifier == 0x02){ 
      autonomousNow = true; // autonomous mode activates - cutter will make its own decisions
      Serial.println(F("Autonomous Mode ON"));
    }
  }

  // cut balloon if the master timer expires
  if(millis() > MASTER_TIMER) {
    requestCut();
    cutReasonB = 0x70;
  }

  if(millis() - cutStampB > CUT_INTERVAL && cutterOnB) cutResistorOffB(); // shuts the resistor off after the cut interval so it doesn't keep burning

  fixGPSLEDSchema(); // these update the LED patterns
  fixXbeeLEDSchema();
  
}
