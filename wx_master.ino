 //This is the master program for the arduino wx stations
//First written September 22, 2015 by Chris Phillips
//Added BMP 180 on Sept. 22, 2015
//Added AM2315 on Sept. 23, 2015
//Added SHT15 in November 2015
//Removed AM2315 in December 2015
//Adjusted for arduino pro mini 3.3v February 1st, 2016
//Enabled program to run even when sensors are missing on May 27, 2016

/* Analog Pins used:
 * A0 - Wind Direction
 * A1 - Air Quality  
 * 
 * Digital Pins used:
 * D2 - Anemometer
 * D3 - Rain Tipping Bucket
 * D4 - Air Quality Fan
 * D6 - SHT1X Clock
 * D7 - SHT1x Data
*/

//Including libraries
#include<SD.h>  //For storing files on an sd card
#include<SPI.h>  //For SPI communication
#include<SFE_BMP180.h>  //For using BMP 180 pressure sensor
#include <SHT1x.h> //For use with SHT1X temp and humidity sensor. Comment out when not in use
#include<Wire.h>  //Used for I2C communication

//creation of necessary objects
SFE_BMP180 pressure;
SHT1x sht1x(7,6);
//SHT1x sht1x(10, 11); //sht1x(dataPin, clockPin)

//Defning i2c address of clock chip
#define DS3231_I2C_ADDRESS 0x68

//Timing Controls
unsigned long const dt = 15; //Measurement time in seconds (minimum is 5)

//Initialization of variables
char output_filename[13]; //Name of file to store data in. Max 8 chars plus extension.
volatile unsigned int wind_count; //For determining wind speed
volatile byte rain_count; //For determining rainfall
unsigned long raintime = 0; //used to debounce rainfall

//Time Keeping Variables - Used for timestamps
byte second;
byte minute;
byte hour;
byte dayOfWeek;
byte dayOfMonth;
byte month;
byte year;

//Creating file identifiers
File file_lun; //Output file identifier

//#####################################
//--------------SETUP------------------
//#####################################
void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600); //Opening serial port at 9600 baud
  pressure.begin();

  //Setting pins for input
  pinMode(2, INPUT); //Pin for wind speed
  pinMode(3, INPUT); //Pin for Rain

  //go to wind_increment function on Rising edge of pin 2
  attachInterrupt(digitalPinToInterrupt(2), wind_increment, FALLING);
  //go to rain increment function on Rising edge of pin 3
  attachInterrupt(digitalPinToInterrupt(3), rain_increment, RISING);

  //Setting digital pins needed for output
  //Air Quality Fan
  //pinMode(4, OUTPUT);

  //Initializing SD Card
  SD.begin(10); //Pin to activate for SD card

  //Creating filename
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  sprintf(output_filename, "20%u%02u%02u.txt",year,month,dayOfMonth);
  
  //BMP180 startup commands
  //Opening file to create header
  file_lun = SD.open(output_filename, FILE_WRITE);
  file_lun.println("Begin Record");
  file_lun.print("Yr");
  file_lun.print("\t");
  file_lun.print("Month");
  file_lun.print("\t");
  file_lun.print("Day");
  file_lun.print("\t");
  file_lun.print("Hr");
  file_lun.print("\t");
  file_lun.print("Min");
  file_lun.print("\t");
  file_lun.print("Sec");
  file_lun.print("\t");
  file_lun.print("P mb");
  file_lun.print("\t");
  file_lun.print("BMP T 'C");
  file_lun.print("\t");
  
  file_lun.print("SHT15 T 'C");
  file_lun.print("\t");
  file_lun.print("RH");
  file_lun.print("\t");
  
  file_lun.print("Rain (tips)"); //Rain for measurement period
  file_lun.print("\t");
  file_lun.print("Wspd m/s");
  file_lun.print("\t");
  file_lun.println("Wdir V"); //Must be converted from volts in post-processing
  //file_lun.print("\t");
  //file_lun.println("Air Q. V");
  file_lun.close();
  
}

//##############################################
//------------------MAIN LOOP-------------------
//##############################################
void loop() {
  // put your main code here, to run repeatedly:
  /*//Printing data to screen (comment out when not hooked to pc)
  file_lun = SD.open(output_filename, FILE_READ);
  while (file_lun.available()) {
   Serial.write(file_lun.read());
  }
  file_lun.close();
  //End printing to screen */

  //Local Variables
  unsigned long time_start;

  //Time stamp
  //Reading Time
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  
  //Opening File
  sprintf(output_filename, "20%u%02u%02u.txt",year,month,dayOfMonth);
  file_lun = SD.open(output_filename, FILE_WRITE);

  time_start = millis();
  file_lun.print(year);
  file_lun.print("\t");
  file_lun.print(month);
  file_lun.print("\t");
  file_lun.print(dayOfMonth);
  file_lun.print("\t");
  file_lun.print(hour);
  file_lun.print("\t");
  file_lun.print(minute);
  file_lun.print("\t");
  file_lun.print(second);
  file_lun.print("\t");
  
  //BMP 180 pressure sensor
  file_lun.print(bmpget_pressure());
  file_lun.print("\t");
  file_lun.print(bmpget_temperature());
  file_lun.print("\t");


  //SHT15 thermometer and humidity sensor
  file_lun.print(sht1x.readTemperatureC());
  file_lun.print("\t");
  file_lun.print(sht1x.readHumidity());
  file_lun.print("\t");
  

  //Rain bucket
  file_lun.print(rain_count); //rain in mm for period
  rain_count = 0; //Resetting wind count
  file_lun.print("\t");
  
  
  //Wind speed and wind direction
  file_lun.print(1.00584*(float(wind_count)/float(dt))); // wind speed in m/s
  wind_count = 0; //Reseting wind_count
  file_lun.print("\t");
  file_lun.print(analogRead(0)); //0.0049 converts analogRead value to a voltage, for 5V arduinos
  file_lun.println("\t");                    //0.0032 is for 3.3v arduinos


  //Air Quality
  /*
  digitalWrite(4, HIGH);
  delay(30000);
  file_lun.println(analogRead(1)*0.0049);
  digitalWrite(4, LOW);
  */
  file_lun.close();

delay(dt*1000.0-(millis()-time_start)); //delay takes milliseconds so need convert from sec -> msec
}


//##############################################
//WIND AND RAIN
//##############################################

//wind_increment procedure
void wind_increment() {
  wind_count++;
}

//rain_increment procedure
void rain_increment() {
  if ((millis()-raintime)>500) {
    rain_count++;
    raintime = millis();
  }
}

//##############################################
//TIMEKEEPING ROUTINES
//##############################################

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

void readDS3231time(byte *second,
byte *minute,
byte *hour,
byte *dayOfWeek,
byte *dayOfMonth,
byte *month,
byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}


//##############################################
//BMP ROUTINES
//##############################################

double bmpget_pressure()
//Returns -9999 on failure 
//Adapted from SFE_BMP180 library examples
{
  char status;
  double T, P;
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);
    // Retrieve the completed temperature measurement:
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.
      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);
        // Retrieve the completed pressure measurement:
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else return(-9999);
      }
      else return(-9999);
    }
    else return(-9999);
  }
  else return(-9999);
}
double bmpget_temperature()
//Returns -9999 on failure
//Adapted from SFE_BMP180 library examples
{
  char status;
  double T;
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);
    // Getting temperature
    // Function returns 1 if successful, 0 otherwise.
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      return(T);
    }
    else return(-9999);
  }
  else return(-9999);
}

