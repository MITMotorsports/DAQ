/*  This is the DAQ system for MIT Motorsports.
 *  Questions: Contact Ben Claman, Laura Gustafson, or Justin Carus
 */


/*  Defining global variables for the sensors.
 *  
 */

/*  Hook Up Guide
 *   - Arduino - Component
 *   
 *  XBee
 *   - VCC - 1
 *   - GND - 10
 *   - Tx - 3
 *   - Rx - 2
 *  
 *  GPS
 *   - 
 *   - 
 *   - 
 *   - 
 *   - 
 *   
 *  IMU
 *   - A4 (SDA) - SDA
 *   - A5 (SCL) - SCL
 *   - 5 - CS
 *   
 *  RTC
 *   - 
 *   - 
 *  
 *  SDC
 *   - 13 (SCK) - SCK
 *   - 12 (MISO) - MISO
 *   - 11 (MOSI) - MOSI
 *   - 10 - CS
 *   
 *  TEM
 *   - 3 - Sense Line
 */

// General
#include "SimpleTimer.h"
SimpleTimer timer;

void setup(){
  Serial.begin(9600);
  //setupRTC();
  setupSDC();
  setupTEM();
  setupGPS();
}

void loop(){
  timer.run();
}

unsigned long s;
unsigned long start;
bool log(String key, String val){
  s = millis()/1000 + start;
  Serial.println(val);
  String d = key + ',' + s + ',' + val;
  //Serial.println(d);
  writeSDC(d);
  return true;
}

// GPS //////////////////////////////////////////////////////////////////
#include <SoftwareSerial.h>
SoftwareSerial gpsSerial(10, 11); // RX, TX (TX not used)
const int sentenceSize = 80;
char sentence[sentenceSize];

void setupGPS()
{
  gpsSerial.begin(9600);
  timer.setInterval(500, gpsUpdate);
}

void gpsUpdate()
{
  static int i = 0;
  if (gpsSerial.available())
  {
    char ch = gpsSerial.read();
    if (ch != '\n' && i < sentenceSize)
    {
      sentence[i] = ch;
      i++;
    }
    else
    {
     sentence[i] = '\0';
     i = 0;
     displayGPS();
    }
  }
}

void displayGPS()
{
  char field[20];
  getField(field, 0);
  if (strcmp(field, "$GPRMC") == 0)
  {
    getField(field, 3);  // number
    log("lat", field);
    getField(field, 4); // N/S
    
    getField(field, 5);  // number
    log("lon", field);
    getField(field, 6);  // E/W
  }
}

void getField(char* buffer, int index)
{
  int sentencePos = 0;
  int fieldPos = 0;
  int commaCount = 0;
  while (sentencePos < sentenceSize)
  {
    if (sentence[sentencePos] == ',')
    {
      commaCount ++;
      sentencePos ++;
    }
    if (commaCount == index)
    {
      buffer[fieldPos] = sentence[sentencePos];
      fieldPos ++;
    }
    sentencePos ++;
  }
  buffer[fieldPos] = '\0';
}

/*
// IMU //////////////////////////////////////////////////////////////////
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include "SparkFunLSM9DS1.h"

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

void imuSetup() 
{
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    log("err", "Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
  } else {
    timer.setInterval(20, updateIMU);
  }
}

void updateIMU()
{
  //printGyro();  // Print "G: gx, gy, gz"
  printAccel(); // Print "A: ax, ay, az"
  //printMag();   // Print "M: mx, my, mz"
  
  // Print the heading and orientation for fun!
  // Call print attitude. The LSM9DS1's magnetometer x and y
  // axes are opposite to the accelerometer, so my and mx are
  // substituted for each other.
  //printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
  //Serial.println();
  
  //delay(PRINT_SPEED);
}

void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  imu.readGyro();
  
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  //Serial.print(imu.calcGyro(imu.gx), 2);
  //Serial.print(", ");
  //Serial.print(imu.calcGyro(imu.gy), 2);
  //Serial.print(", ");
  //Serial.print(imu.calcGyro(imu.gz), 2);
  //Serial.println(" deg/s");
  //log("imu", String(imu.calcGyro(imu.gx), 2) + "," + 
  //           String(imu.calcGyro(imu.gy), 2) + "," + 
  //           String(imu.calcGyro(imu.gz), 2));
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu.readAccel();
  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  //Serial.print(imu.calcAccel(imu.ax), 2);
  //Serial.print(", ");
  //Serial.print(imu.calcAccel(imu.ay), 2);
  //Serial.print(", ");
  //Serial.print(imu.calcAccel(imu.az), 2);
  //Serial.println(" g");
  log("imu", String(imu.calcAccel(imu.ax), 2) + "," + 
             String(imu.calcAccel(imu.ay), 2) + "," + 
             String(imu.calcAccel(imu.az), 2));
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  imu.readMag();
  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(
float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}
*/

// RTC //////////////////////////////////////////////////////////////////
#include <Wire.h>
/*
// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>
#include "RTClib.h"

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

RTC_DS1307 rtc;
DateTime starttime = new DateTime(0);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setupRTC() {
  if (! rtc.begin()) {
    log("err","Couldn't find RTC");
  }

  if (rtc.isrunning()) {
    starttime = rtc.now();
    start = starttime.unixtime();
  } else{
    log("wrn", "RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    start = 0;
    
  }
}
*/

// EEPROM workaround

#include <EEPROM.h>
#define EEPROMADDR 0
int filenum = 0;

// SDC //////////////////////////////////////////////////////////////////
#include <SPI.h>
#include <SD.h>

#define chipSelect 10
File dataFile;
void setupSDC()
{


filenum = EEPROM.read(EEPROMADDR);
filenum = (filenum + 1) % 256;
String filename = String(filenum) + ".csv";
Serial.println("Recording to " + filename);
EEPROM.write(EEPROMADDR, filenum);



  
  if (!SD.begin(chipSelect)) {
    log("err", "Card failed, or not present");
  }
                                 
  /*dataFile = SD.open(String(starttime.year()) + '.' + 
                                 starttime.month() + '.' + 
                                 starttime.day() + ' ' + 
                                 starttime.hour() + '.' + 
                                 starttime.minute() + '.' + 
                                 starttime.second() + ".csv", FILE_WRITE);*/
  dataFile = SD.open(filename, FILE_WRITE);
  Serial.println("Output to " + filename);
}

void writeSDC(String str)
{
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  if (dataFile) {
    dataFile.println(str);
    dataFile.flush();
  }
  // if the file isn't open, pop up an error:
  else {
    log("err", "error writing to file");
  }
}


// SDC //////////////////////////////////////////////////////////////////
#include "OneWire.h"
#include "DallasTemperature.h"

// Data wire is plugged into pin 3 on the Arduino
#define ONE_WIRE_BUS 3

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

DeviceAddress addrs[8];
 
// Records all temperatures. The function takes about 835 milliseconds to run.
void updateTEM(){
  sensors.requestTemperatures();
  String dataString;
  for(int i = 0; i < sensors.getDeviceCount(); i++){
    dataString += sensors.getTempC(addrs[i]) + ',';
  }
  log("tem", dataString);
}

void setupTEM() {  

  // Start up the library
  sensors.begin();
  int numDevices = sensors.getDeviceCount();
  log("nfo",String("found ") + numDevices + " temperature sensors");
  int i = 0;
  DeviceAddress addr;
  while(oneWire.search(addr)){
    memcpy(addrs[i], addr, 1);
    sensors.setResolution(addr, 12);
    i++;
  }
  timer.setInterval(1000, updateTEM);
}

/*
// Used to find the device addresses of all sensors on the bus
void discoverOneWireDevices(void) {
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  
  Serial.print("Looking for 1-Wire devices...\n\r");
  while(oneWire.search(addr)) {
    Serial.print("\n\rFound \'1-Wire\' device with address:\n\r");
    for( i = 0; i < 8; i++) {
      Serial.print("0x");
      if (addr[i] < 16) {
        Serial.print('0');
      }
      Serial.print(addr[i], HEX);
      if (i < 7) {
        Serial.print(", ");
      }
    }
    if ( OneWire::crc8( addr, 7) != addr[7]) {
        Serial.print("CRC is not valid!\n");
        return;
    }
  }
  Serial.print("\n\r\n\rThat's it.\r\n");
  oneWire.reset_search();
  return;
}
*/
