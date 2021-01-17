/////////////////////////////////////////////////////////////////////////////////////////////////
///////////*************************** MAIN DEFINITIONS ****************************/////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

/**************DATA BACKUP*******************/
File flightData;
String ext = ".bin";
String fileN;
String fileName1;
unsigned long timeSD;     // Time stamp - used for determining when to save SD card
int appendPos;            // File position marker - used for saving and re-opening SD file
unsigned int fileNum;
unsigned int address;
int sdCount;
const int chipSelect = BUILTIN_SDCARD;

/**************GPS SENSORS*******************/
//static NMEAGPS  uBloxEX;   //uBlox GPS instantiate NMEAGPS object
//static gps_fix  uBloxEXFix; //gps_fix object

int32_t lat1 = 0;     //Latitude
int32_t lon = 0;      //Longitude
int32_t alt = 0;      //Altitude
byte stat = 0;        //Status
uint8_t numSats = 0;  //Number of Satellites in View
uint8_t utcHour = 0;  //UTC Time - Hour
uint8_t utcMin = 0;   //UTC Time - Minutes
uint8_t utcSec = 0;   //UTC Time - Seconds

/******************RADIO VARIABLES AND PARAMETERS*********************/

#include <RelayXBee.h>
#define xbeeSerial Serial5
#define tooManyContacts 100                         //If system has recorded more than this amount of contacts, the memory of known contacts is reset to prevent a memory leak if there's a bug
#define xbeeChannel 'C'                             //Shorthand for a communication channel preset (See RelayXBee library for specifics)
#define CDU_RX_SIZE 35                            //Number of bytes in CDU packet
#define CDU_TX_SIZE 7                               //Number of bytes in a gondola packet
#define keepRadioEvents false                       //If true, radioEvents builds throughout flight. Otherwise, it is cleared at the start of every updateXbee() call
#define waitForReconnection 35                      //Amount of time (secs) of radio inactivity before the gondola starts searching for a lost cutter. Should be more than the equivelant value
                                                      //on a cutter box
#define xbeeID "MURI"                               //Initialization ID for all mesh members

byte gondolaIdentifier = 0x00;                      //Holds the gondola's individual ID, set during xbeeStartup()
RelayXBee xbee = RelayXBee(&xbeeSerial,xbeeID);     //Creates XBee Object

byte *knownIDs;                                     //Dynamically allocated array holding ID names of all XBee contacts on network
                                                      //See documentation for key
byte connectedA = 0x00;                             //ID of the cutter A the gondola thinks it should be connected to
byte connectedB = 0x00;                             //ID of the cutter B the gondola thinks it should be connected to
short contactsHeld = 0;                             //The number of known XBees on the communication channel
short downtimeA=0,downtimeB=0;                      //Holds the millis time (secs) of last successful transmission. If goes over 35 secs since last, gondola searches for new box to connect to
String radioEvents;                                 //If significant events or warnings occur during running of this code, it is put here. Do with it what you like.
bool searching = false;                             //True if new cutter contacts should be connected into gondola network
unsigned long pingcool = 0;

byte recoveredDataA[CDU_RX_SIZE];                   //Holds data collected from cutter unit A. Replaces cdu1Packet_rx
byte recoveredDataB[CDU_RX_SIZE];                   //Holds data collected from cutter unit B. Replaces cdu2Packet_rx

/******************DATA PACKETS AND SENSORS VARIABLES*********************/
ADC *adc = new ADC();

byte id_erau[2] = {0xA0, 0xB1};//Identifier for GPS + ERAU packet.
uint16_t type = 0;             //Identifier for type of turbulence packet.
uint16_t sps_start = 0;        //Identifier for SPS30 data packet.
uint16_t erau_checksum = 0;
uint16_t checksum;  
uint8_t cutReasonA = 0x22, cutReasonB = 0x22;

uint16_t sps_packet_number = 0;// Used to test whether packets are being skipped - may be removed later
uint16_t cu_packet_number = 0; // Used to test whether packets are being skipped - may be removed later

//ARRAY SIZE CONSTANTS
#define GPS_SIZE 48             // Number of bytes in GPS + ERAU packet
#define SPS_SIZE 81             // Number of bytes in SPS30 packet
#define INSTRUMENT_SIZE 67      // Number of bytes in instrument packet
#define GONDOLA_SIZE 50         // Number of bytes in gondola packet
#define RAW_SIZE 73             // Number of bytes in raw packet
#define CDU_RX_SIZE 36          // Number of bytes in CDU packet
#define CDU_TX_SIZE 7           // Number of bytes to send to CDU


//CODES FOR BLUETOOTH
byte CUT_A_command = 0x15;      // Gondola to cutter A - CUT command
byte CUT_B_command = 0x25;      // Gondola to cutter B - CUT command
#define SIZE 10 // size of arrays that store values
uint8_t fixStatus[SIZE]; // gps fix status
byte cdu1Packet_tx[CDU_TX_SIZE] = {0x42, 0x41, fixStatus[0], CUT_A_command, 0, 0, 0x53};
byte cdu2Packet_tx[CDU_TX_SIZE] = {0x42, 0x42, fixStatus[0], CUT_B_command, 0, 0, 0x53};
byte MTcdu1Packet_tx[CDU_TX_SIZE] = {0x42, 0x41, fixStatus[0], 0x30, 0, 0, 0x53};
byte MTcdu2Packet_tx[CDU_TX_SIZE] = {0x42, 0x42, fixStatus[0], 0x30, 0, 0, 0x53};
byte AlternateMTcdu1Packet_tx[CDU_TX_SIZE] = {0x42, 0x41, fixStatus[0], 0x31, 0, 0, 0x53};
byte AlternateMTcdu2Packet_tx[CDU_TX_SIZE] = {0x42, 0x42, fixStatus[0], 0x31, 0, 0, 0x53};

byte erauPacket[GPS_SIZE];      //Scientific data packet byte array.
byte umnPacket[SPS_SIZE];       //UMN - SPS30 packet
byte cuPacket[RAW_SIZE];        //CU - turbulence packet
//byte cdu1Packet_rx[CDU_RX_SIZE];//Cutting Descent Unit 1 - Cutter A 
//byte cdu2Packet_rx[CDU_RX_SIZE];//Cutting Descent Unit 2 - Cutter B
unsigned int packet_number;     //Packet number/counter.
unsigned long time_packet;      //Time stamp used to mark ERAU packets - close to UTC time stamp
unsigned long umn_time_packet;  //Time stamp used to relate SPS measurement to absolute time
unsigned long cu_time_packet;   //Time stamp used to relate CU measurement to absolute time
unsigned char write_counter;    //Array indexer for writing data.
unsigned char read_counter;     //Array indexer for reading data.

unsigned long fix_timestamp = 0;
unsigned long cdu_timer;

//CDU VARIABLES DEFINITON
byte cdu1_status = 0;
byte cdu2_status = 0; // Status of CDUs
int16_t cdu1_temp_uno = 0;
int16_t cdu1_temp_bat = 0;
int16_t cdu2_temp_uno = 0;
int16_t cdu2_temp_bat = 0; //CDU temperatures

unsigned long cdu1_watchdog;    // timer used to reset bluetooth 1
unsigned long cdu2_watchdog;    // timer used to reset bluetooth 2
uint16_t cdu1_start;            //Identifier for CDU 1 packet
uint16_t cdu2_start;            //Identifier for CDU 2 packet

//ANALOG PINS DEFINITION
#define TEMP_EXT_H A9           //External Temperature - High Range.
#define TEMP_EXT_L A8           //External Temperature - Low Range.
#define TEMP_INT A7             //Internal Temperature.
#define VOLTAGE A6              //Voltage Monitor (VBat).

int tempExt_l = 0;              //External Temperature - Low Range.
int tempExt_h = 0;              //External Temperature - High Range.
int tempInt = 0;                //Internal Temperature.
int voltage = 0;                //Voltage Monitor (VBat).

//9DoF PINS DEFINITION
#define LSM9DS1_SCK 13
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI 11
#define LSM9DS1_XGCS 15         //set pin for accelerometer/gyro chip select (XGCS)
#define LSM9DS1_MCS 14          //set pin for magnetometer chip select(MCS)

//9DoF VARIABLES DEFINITION
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);
sensors_event_t a, m, g, temp;
int accel_x = 0;
int accel_y = 0;
int accel_z = 0;

int gyro_x = 0;
int gyro_y = 0;
int gyro_z = 0;

#define STARTUP_TIME_LIMIT 18000000  // How much time passes before data stops coming through UART 6
#define FIX_3D_LED         6      // GPIO pin for pre-launch 3D fix verification



/////////////////////////////////////////////////////////////////////////////////////////////////
///////////*************************** CDU DEFINITIONS ****************************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

#define MAX_LAT_CHANGE 0.05 // In degrees (because the gps function returns this in degrees)
#define MAX_LON_CHANGE 0.02 // In degrees this is smaller because the latitude in sweeden is smaller per degree
#define MAX_ALT_CHANGE 300 // I think this is in meters, I believe thats what the gps outputs but can easily be changed to feet
#define SIZE 10 // size of arrays that store values
#define FIX 0x01
#define NOFIX 0x00
#define D2R PI/180
#define R2D 180/PI
#define SECONDS_PER_HOUR 3600
#define FPM_PER_MPH 88

float latitude[SIZE], longitude[SIZE], Altitude[SIZE];
bool AWorking, BWorking, CWorking;
unsigned long timeStamp[SIZE];
float dt = 0;
uint8_t sats;
float ascentRate;
float ascentRatePrev = 0;
float groundSpeed;
float heading;

float latA, latB, latC, latAPrev, latBPrev, latCPrev;
float lonA, lonB, lonC, lonAPrev, lonBPrev, lonCPrev;
float altA, altB, altC, altAPrev, altBPrev, altCPrev;

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////********************** STATE MACHINE DEFINITIONS ***********************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

  #define M2MS 2*60000
#define ALTITUDE_FLOOR 5000
#define ALTITUDE_CEILING 100000
#define SA_FLOOR 50000
#define SLOW_DESCENT_FLOOR 80000
////change boundaries before every flight!!!////
#define EASTERN_BOUNDARY -92.49
#define WESTERN_BOUNDARY -95.07
#define SOUTHERN_BOUNDARY 43.74
#define NORTHERN_BOUNDARY 45.57
///////////////////////////////////////////////
#define MASTER_TIMER 180*M2MS
#define ASCENT_TIMER 150*M2MS
#define SA_TIMER 30*M2MS
#define FLOAT_TIMER 30*M2MS
#define SLOW_DESCENT_TIMER 40*M2MS
#define INITIALIZATION_TIME 25*M2MS
#define DEFAULT_TIME 30*M2MS

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

// have compareGPS output the following GPS average of all systems, saved as struct
struct GPSData{ // proposed struct filled out by compareGPS
  float alt;
  float latitude;
  float longitude;
  float AR;
} GPSdata;

struct DetData{ // proposed struct filled out by Determination
  // lat, long = 0 if GPS data is bad (might want to use something other than 0)
  // pressure = 0 if GPS data is good
  
  float alt;
  float latitude;
  float longitude;
  float AR;
  float pressure;
  float Time; 
  uint8_t Usage; // 00 means using timer (error on all others), 01 means using 1 GPS, 02 using 2 GPS, 03 all 3 GPS, 04 using pressure, 05 using linear progression
} detData;
uint8_t SDcounter = 0;
uint8_t ascentCounter = 0, SAcounter = 0, floatCounter = 0, descentCounter = 0;
uint8_t tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0;
unsigned long ascentStamp = 0, SAstamp = 0, floatStamp = 0, defaultStamp = 0, defaultStamp2, defaultStampCutA = 0, xbeeStamp = 0;

bool tenGoodHits = false;
uint8_t tenGoodHitsCounter = 0;

uint8_t stateSuggest; // state recommended by control
uint8_t currentState = INITIALIZATION; // state we are in, starts as initialization

unsigned long updateStamp, SDstamp, descentStamp, timerStampCutA;
int which = 1;
int initCounter = 1;
int initCounter2 = 1;
int timeCounter = 1;

// float state ascent rate fixes
float ARprev = 0;
int FARcounter = 0;

int cwsa = 0, hwsa = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////
