//============================================================================================================================================
// MURI Resistor Cutter Box A
// Written by Steele Mitchell and PJ Collins - mitc0596 & coll0792 Spring 2020
// XBee mesh written by Paul Wehling - wehli007 Fall 2020
//============================================================================================================================================
//
// Can run autonomously or communicate with a main computer. Will cut when instructed, or on its own if no communication.
//
//=============================================================================================================================================

/*  Arduino Uno w/ PCB Shield pin connections:
     ----------------------------------------------
    | Component                    | Pins used     |    
    | ---------------------------------------------|
    | UBlox Neo m8n                | 0,1           |
    | LED                          | 7             | 
    | Xbee                         | 8,9           |
    | H-Driver                     | 10,11         |
    | Latching Relay               | 12,13         |
    | Thermistors                  | A0, A1        | 
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
#define CUTTER_PIN4 3 // inactive
#define HEAT_ON 12
#define HEAT_OFF 13
#define AKSHAY_PIN A5
#define THERMISTOR_A A0
#define THERMISTOR_B A1 

// Radio Parameters
#define xbeeChannel 'C'                             //Shorthand for a communication channel preset (See RelayXBee library for specifics)
#define CDU_RX_SIZE 36                              //Number of bytes in CDU packet
#define CDU_TX_SIZE 7                               //Number of bytes in a gondola packet
#define waitForReconnection 30                      //Amount of time (secs) of radio inactivity before the cutter starts listening for a new connection
#define xbeeID "MURI"                               //Initialization ID for all mesh members


// Intervals
// #define FIX_INTERVAL 5000               // GPS with a fix—will flash for 5 seconds
// #define NOFIX_INTERVAL 2000             // GPS with no fix—will flash for 2 seconds
//#define TEST_INTERVAL 30*60000          // Interval to test cutters - uncomment for thermal vac test!
#define LED_INTERVAL 1000               // LEDs run on a 1 second loop to indicate lack of connection
#define UPDATE_INTERVAL 1000            // update all data and the state machine every 1 second
#define CUT_INTERVAL 120000              // ensure the cutting mechanism is on for 2 minutes
#define MASTER_INTERVAL 210             // master timer that cuts balloon after 3hr, 30min
//#define PRESSURE_TIMER_INTERVAL 50      // timer that'll cut the balloon 50 minutes after pressure reads 70k feet
#define ASCENT_INTERVAL 180             // timer that cuts balloon A 3 hours after ASCENT state initializes
#define SLOW_DESCENT_INTERVAL 60        // timer that cuts both balloons (as a backup) an hour after SLOW_DESCENT state initializes
#define SLOW_DESCENT_BUFFER 10          // timer that ensures slow descent state is maintained for 10 minutes
//#define HEAT_INTERVAL 5

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

// Fix statuses
#define NOFIX 0x00
#define FIX 0x01


// Boundaries
///////CHANGE BEFORE EACH FLIGHT////////
#define EASTERN_BOUNDARY -93.06           // longitudes
#define WESTERN_BOUNDARY -94.281
#define NORTHERN_BOUNDARY 44.29            // latitudes
#define SOUTHERN_BOUNDARY 43.94
#define SLOW_DESCENT_CEILING 100000     // max altitude stack can reach before balloon is cut and stack enters slow descent state
#define SLOW_DESCENT_FLOOR 80000        // min altitude for the slow descent state
#define INIT_ALTITUDE 2000              // altitude at which the state machine begins
#define RECOVERY_ALTITUDE 5000          // altitude at which the recovery state intializes on descent
#define MIN_TEMP -60                    // minimum acceptable internal temperature
#define MAX_TEMP 90                     // maximum acceptable interal temperature
#define LOW_TEMP -5                     // activation temp for heating pads
#define HIGH_TEMP 5                     // deactivation temp for heating pads
#define MINIMUM_VOLTAGE 0             // minimum acceptable voltage available

// Velocity Boundaries
#define MAX_SA_RATE 375                 // maximum velocity (ft/min) that corresponds to a slow ascent state
#define MAX_FLOAT_RATE 100              // maximum velocity that corresponds to a float state, or minimum for a slow ascent state
#define MIN_FLOAT_RATE -100             // minimum velocity that corresponds to a float state, or maximum for a slow descent state
#define MIN_SD_RATE -600                // minimum velocity that corresponds to a slow desent state

// #define PRESSURE_TIMER_ALTITUDE 70000   // altitude at which the pressure timer begins

//Thermistor
#define C2K 273.15
#define ADC_MAX 1024                                                    // The maximum adc value given to the thermistor, should be 8196 for a teensy and 1024 for an Arduino
#define CONST_A 0.001125308852122
#define CONST_B 0.000234711863267                                       // A, B, and C are constants used for a 10k resistor and 10k thermistor for the steinhart-hart equation
#define CONST_C 0.000000085663516                                       // NOTE: These values change when the thermistor and/or resistor change value, so if that happens, more research needs to be done on those constants
#define CONST_R 10000        
float t1 = -127.00;                                                    //Temperature initialization values
float t2 = -127.00;

// Time Stamps
unsigned long updateStamp = 0, restartStamp =0;
unsigned long cutStampA = 0,  cutStampB = 0;  
unsigned long LEDOnStamp = 0, LED2OnStamp = 0;
unsigned long LEDOffStamp = 0, LED2OffStamp = 0;
unsigned long slowBuffer = 0;
unsigned long testStamp = 0;

// State Machine
uint8_t state; 
bool stateSwitched;
bool maxAltReached = false;
uint8_t cutStatusB = 0x01; // 1 for false, 2 for true
bool cutterOnB = false;
uint8_t cutReasonB;
String stateString;
uint8_t prevHit = 1;
uint8_t thisHit = 1;

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

// active heating variables
float sensTemp;
bool coldSensor = false;
LatchRelay sensorHeatRelay(HEAT_ON,HEAT_OFF);        //Declare latching relay objects and related logging variables
String sensorHeat_Status = "";
uint8_t heatStatus = 0x00;
//int m = 1;
//unsigned long heatStamp = 0;

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
  float volts;
  float t1;
  float t2;
  uint8_t cutStatus;
  bool heatStatus;
  uint8_t currentState;
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
bool autonomousNow = false;
uint8_t counterZero = 0; 
uint8_t counterOne = 0;
uint8_t prevCommand;
int once = 1;

void setup() {
  Serial.begin(9600);   // initialize serial monitor
  
  xbeeStartup(); // initialize xbee serial communication
    
  initGPS();            // initialze GPS

  initRelays();        //Initialize Relays

  pinMode(LED,OUTPUT);
  pinMode(LED2,OUTPUT);

  pinMode(CUTTER_PIN1,OUTPUT);
  pinMode(CUTTER_PIN2,OUTPUT);
  pinMode(CUTTER_PIN3,OUTPUT);
  pinMode(CUTTER_PIN4,OUTPUT);

}

void loop() {

  gps.update();

  if(millis() - updateStamp > UPDATE_INTERVAL) {   
    updateStamp = millis();

    updateTemperatures();
 
    actHeat();          //Controls active heating

    updateTelemetry();  // update GPS data

    stateMachine();     // update the state machine
    Serial.print(F("State: "));
    Serial.println(stateString);

    Serial.print(F("Lat & long: "));
    Serial.print(latitude[0]);
    Serial.print(F(", "));
    Serial.println(longitude[0]);
    Serial.print(F("Temps and Volts: "));
    Serial.print(t1);
    Serial.print(F(", "));
    Serial.print(t2);
    Serial.print(F(", "));
    Serial.println(2*analogRead(AKSHAY_PIN));

    // Add this for thermal vac test
//     if (millis() - testStamp > 5*M2MS && once == 1){
//       testStamp = millis();
//       once++;
//       cutResistorOnA();
//     }
     
    sendData();         // send current data to main

    //readInstruction();  // read commands from main, cuts if instructed

    updateXbee();
    timeOut = millis()-(downtimeG*1000);
     if( timeOut > 1*M2MS){ // change to desired max disconnect time
      autonomousNow = true;
      Serial.println(F("Autonomous Mode ON"));
    }
  }

  // cut balloon if the master timer expires
  if(millis() > MASTER_INTERVAL*M2MS) {
    cutResistorOnB();
    cutReasonB = F("master timer expired");
  }

  if(millis() - cutStampB > CUT_INTERVAL && cutterOnB) cutResistorOffB();

  fixGPSLEDSchema();
  fixXbeeLEDSchema();
  
}
