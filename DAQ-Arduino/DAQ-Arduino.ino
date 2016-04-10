/*  This is the DAQ system for MIT Motorsports.
 *  Questions: Contact Ben Claman, Laura Gustafson, or Justin Carrus
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
 *   - RX0 - 5
 *   - TX0 - 6
 *   - 
 *   
 *  IMU
 *   - A4 (SDA) - SDA
 *   - A5 (SCL) - SCL
 *   
 *  TEM
 *   - 3 - Sense Line
 */

/////////////
// General //
/////////////
#include "./SimpleTimer.h"
#include <SoftwareSerial.h>
SimpleTimer timer;
SoftwareSerial XBee(5, 6);

void setup(){
	XBee.begin(9600);
	
	//XBee.begin(115200);
  setupTEM();
  //setupGPS();
  setupIMU();
  setupCAN();
}

void loop(){
  timer.run();
}

unsigned long s;
bool log(String key, String val){
  s = millis();
  String d = String(s) + ',' + key + ',' + val;
  XBee.println(d);
  return true;
}


/////////
// CAN //
/////////
void setupCAN(){
	Serial.begin(115200);
	timer.setInterval(50, canUpdate);
}

void canUpdate(){
	if (Serial.available() > 0){
    XBee.print(millis());
    XBee.print(',');
		while(Serial.peek() != '\n'){
			XBee.print((char) Serial.read());
		}
   Serial.read();
	}
}

/////////
// GPS //
/////////
#include <SoftwareSerial.h>
SoftwareSerial gpsSerial(10, 11);
const int sentenceSize = 80;
char sentence[sentenceSize];

void setupGPS()
{
  gpsSerial.begin(9600);
  timer.setInterval(50, gpsUpdate);
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


/////////
// IMU //
/////////
#include <Wire.h>
#include <SPI.h>
#include "./SparkFunLSM9DS1.h"
LSM9DS1 imu;
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

// Sketch Output Settings //
#define PRINT_CALCULATED
//#define PRINT_RAW

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

void setupIMU() 
{
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  // [enabled] turns the acclerometer on or off.
  imu.settings.accel.enabled = true; // Enable accelerometer
  // [enableX], [enableY], and [enableZ] can turn on or off
  // select axes of the acclerometer.
  imu.settings.accel.enableX = true; // Enable X
  imu.settings.accel.enableY = true; // Enable Y
  imu.settings.accel.enableZ = true; // Enable Z
  // [scale] sets the full-scale range of the accelerometer.
  // accel scale can be 2, 4, 8, or 16
  imu.settings.accel.scale = 2; // Set accel scale to +/-8g.
  // [sampleRate] sets the output data rate (ODR) of the
  // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
  // DISABLED! Otherwise accel sample rate = gyro sample rate.
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz
  imu.settings.accel.sampleRate = 2; // Set accel to 10Hz.
  // [bandwidth] sets the anti-aliasing filter bandwidth.
  // Accel cutoff freqeuncy can be any value between -1 - 3. 
  // -1 = bandwidth determined by sample rate
  // 0 = 408 Hz   2 = 105 Hz
  // 1 = 211 Hz   3 = 50 Hz
  imu.settings.accel.bandwidth = -1; // BW = 408Hz
  // [highResEnable] enables or disables high resolution 
  // mode for the acclerometer.
  imu.settings.accel.highResEnable = false; // Disable HR
  // [highResBandwidth] sets the LP cutoff frequency of
  // the accelerometer if it's in high-res mode.
  // can be any value between 0-3
  // LP cutoff is set to a factor of sample rate
  // 0 = ODR/50    2 = ODR/9
  // 1 = ODR/100   3 = ODR/400
  imu.settings.accel.highResBandwidth = 0;  

  // [enabled] turns the magnetometer on or off.
  imu.settings.mag.enabled = true; // Enable magnetometer
  // [scale] sets the full-scale range of the magnetometer
  // mag scale can be 4, 8, 12, or 16
  imu.settings.mag.scale = 12; // Set mag scale to +/-12 Gs
  // [sampleRate] sets the output data rate (ODR) of the
  // magnetometer.
  // mag data rate can be 0-7:
  // 0 = 0.625 Hz  4 = 10 Hz
  // 1 = 1.25 Hz   5 = 20 Hz
  // 2 = 2.5 Hz    6 = 40 Hz
  // 3 = 5 Hz      7 = 80 Hz
  imu.settings.mag.sampleRate = 5; // Set OD rate to 20Hz
  // [tempCompensationEnable] enables or disables 
  // temperature compensation of the magnetometer.
  imu.settings.mag.tempCompensationEnable = false;
  // [XYPerformance] sets the x and y-axis performance of the
  // magnetometer to either:
  // 0 = Low power mode      2 = high performance
  // 1 = medium performance  3 = ultra-high performance
  imu.settings.mag.XYPerformance = 3; // Ultra-high perform.
  // [ZPerformance] does the same thing, but only for the z
  imu.settings.mag.ZPerformance = 3; // Ultra-high perform.
  // [lowPowerEnable] enables or disables low power mode in
  // the magnetometer.
  imu.settings.mag.lowPowerEnable = false;
  // [operatingMode] sets the operating mode of the
  // magnetometer. operatingMode can be 0-2:
  // 0 = continuous conversion
  // 1 = single-conversion
  // 2 = power down
  imu.settings.mag.operatingMode = 0; // Continuous mode

  // [enabled] turns the gyro on or off.
  imu.settings.gyro.enabled = true;  // Enable the gyro
  // [scale] sets the full-scale range of the gyroscope.
  // scale can be set to either 245, 500, or 2000
  imu.settings.gyro.scale = 245; // Set scale to +/-245dps
  // [sampleRate] sets the output data rate (ODR) of the gyro
  // sampleRate can be set between 1-6
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952
  imu.settings.gyro.sampleRate = 3; // 59.5Hz ODR
  // [bandwidth] can set the cutoff frequency of the gyro.
  // Allowed values: 0-3. Actual value of cutoff frequency
  // depends on the sample rate. (Datasheet section 7.12)
  imu.settings.gyro.bandwidth = 0;
  // [lowPowerEnable] turns low-power mode on or off.
  imu.settings.gyro.lowPowerEnable = false; // LP mode off
  // [HPFEnable] enables or disables the high-pass filter
  imu.settings.gyro.HPFEnable = true; // HPF disabled
  // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
  // Allowable values are 0-9. Value depends on ODR.
  // (Datasheet section 7.14)
  imu.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
  // [flipX], [flipY], and [flipZ] are booleans that can
  // automatically switch the positive/negative orientation
  // of the three gyro axes.
  imu.settings.gyro.flipX = false; // Don't flip X
  imu.settings.gyro.flipY = false; // Don't flip Y
  imu.settings.gyro.flipZ = false; // Don't flip Z
  
  if (!imu.begin())
  {
    log("err", "Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
  } else {
		imu.calibrate();
		imu.calibrateMag();
    timer.setInterval(20, updateIMU);
  }
}

void updateIMU()
{

  imu.readGyro();
  log("gyr", (String(imu.gx) + ' ' + imu.gy + ' ' + imu.gz));
  imu.readAccel();
  log("acc", (String(imu.ax) + ' ' + imu.ay + ' ' + imu.az));
  imu.readMag();
  log("mag", (String(imu.mx) + ' ' + imu.my + ' ' + imu.mz));
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


/////////
// TEM //
/////////
#include "./OneWire.h"
#include "./DallasTemperature.h"

// Data wire is plugged into pin 3 on the Arduino
#define ONE_WIRE_BUS 3

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

//DeviceAddress addrs[8];

//DeviceAddress T1 = {0x28, 0x1F, 0x58, 0x29, 0x07, 0x00, 0x00, 0x53};


float T1C;
float T2C;
float T3C;
float T4C;
DeviceAddress T1 = { 0x28, 0x90, 0xAB, 0x29, 0x07, 0x00, 0x00, 0x75 };
DeviceAddress T2 = { 0x28, 0x32, 0x28, 0x29, 0x07, 0x00, 0x00, 0xC2 };
DeviceAddress T3 = { 0x28, 0x0E, 0xF4, 0x29, 0x07, 0x00, 0x00, 0x2D };
DeviceAddress T4 = { 0x28, 0xCB, 0x6A, 0x29, 0x07, 0x00, 0x00, 0x4C };
 
// Records all temperatures. The function takes about 835 milliseconds to run.
void updateTEM(){
  String dataString;
  T1C = sensors.getTempC(T1);
  T2C = sensors.getTempC(T2);
  T3C = sensors.getTempC(T3);
  T4C = sensors.getTempC(T4);
  dataString = String(T1C) + String(' ') + 
               String(T2C) + String(' ') + 
               String(T3C) + String(' ') + 
               String(T4C);
  /*
  for(int i = 0; i < sensors.getDeviceCount(); i++){
    dataString += sensors.getTempC(addrs[i]) + ',';
  }*/
  log("tem", dataString);
  sensors.requestTemperatures();
}

void setupTEM() {  

  // Start up the library
  sensors.begin();
  sensors.setResolution(T1, 12);
  sensors.setResolution(T2, 12);
  sensors.setResolution(T3, 12);
  sensors.setResolution(T4, 12);
  sensors.setWaitForConversion(false);
  /*
  int numDevices = sensors.getDeviceCount();
  log("nfo",String("found ") + numDevices + " temperature sensors");
  int i = 0;
  DeviceAddress addr;
  while(oneWire.search(addr)){
    memcpy(addrs[i], addr, 1);
    sensors.setResolution(addr, 12);
    i++;
  }*/
  sensors.requestTemperatures();
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
