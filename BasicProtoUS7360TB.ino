/*
  Bridge Sensor with GSM, SDCard, thermistor, US and RTC.
  Sketch used by MSI Sensors platform.
  Created 04JUL16
  Modified 04AUG16
*/
#include <ARTF_SDCard.h>
#include <SdFat.h>

#include <math.h>
#include <Time.h>
#include <Adafruit_FONA.h>
#include <LowPower.h>

#include <SPI.h>


// RTC Dependency
#include <Wire.h>
#include <RTClib.h>

// Analog Pins
#define THERMISTOR_PIN A1
#define ULTRASONIC_PIN A2


//GSM pins
#define FONA_RX 10
#define FONA_TX 9
#define FONA_RST 5
//#define FONA_RI 7
#define FONA_PWR 11

// this is a large buffer for replies
char replybuffer[255];


//Thermistor power pin.
#define THERM_PIN        6


// Settings
#define SENSOR_NUM                "2B"
//Number of readings per text message.
#define SEND_DATA_AFTER_X_READINGS              3
#define TAKE_READING_AFTER_X_HOUR             1
//Number of readings per text message.
#define SEND_X_SMS_AT_START             5

//wait until GSM find the network, please enter in seconds
#define WAIT_FOR_NETWORK    180
//Number of thermistor readings to be averaged.
#define NUM_THERM_READINGS         3
//Delay in miliseconds between temperature readings.
#define THERM_READING_DELAY        200
//Number of distance readings to be averaged.
#define NUM_DISTANCE_READINGS      3
//Delay in milliseconds between distance readings.
#define DISTANCE_READING_DELAY     200
//Character type between data points.
#define DATA_DELIM                 ':'
#define BACKUP_FILENAME            "backup.csv"
#define UNSENT_FILENAME            "unsent.csv"
#define PHONE_NUMBER_SMS                "+93728898056"
#define DISTANCE_INCREMENT         5


#define SD_CS_PIN                   10
ARTF_SDCard sd(SD_CS_PIN);

//Fona library requirment
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);


// Custom Datatypes
typedef struct {
  int distance;
  int temperature;
  String sensorNum;
  //int vbat;
  DateTime timestamp;
} SensorReadingSMS;

// Global Variables
int numCachedReadings = 0;
int readingHour = 0;
bool readingFlag = false;

SensorReadingSMS sensorReadingsSMS[SEND_DATA_AFTER_X_READINGS];

//Real time clock requirement
RTC_PCF8523  rtc;


void setup()
{
  // SC card CS pin defined.
  pinMode(SD_CS_PIN, OUTPUT);
  //Thermistor pin defined.
  pinMode(THERM_PIN, OUTPUT);
  //pinMode(FONA_RST, OUTPUT);
  // PinMode settings for HC-SR04 ultrasonic sensor. Block these if using Maxbotix US sensor.

  pinMode(FONA_PWR, OUTPUT);

  //Turn off GSM. Soon after we connect power in the Basic Proto, the power flow will start to GSM module and will be started.
  //Which destabalize the normal flow and procedure, this line of codes will turn the GSM off
  digitalWrite(FONA_PWR, HIGH);
  delay(2000);
  digitalWrite(FONA_PWR, LOW);
  delay(2000);

  for (int i = 1; i <= SEND_X_SMS_AT_START; i++) {
    //Turn on GSM.
    digitalWrite(FONA_PWR, HIGH);
    delay(2000);
    digitalWrite(FONA_PWR, LOW);
    delay(15000);



    Serial.begin(115200);
    Serial.println(F("FONA SMS caller ID test"));
    Serial.println(F("Initializing....(May take 3 seconds)"));

    fonaSerial->begin(4800);
    if (! fona.begin(*fonaSerial)) {
      Serial.println(F("Couldn't find FONA"));

      while (1);
    }
    Serial.println(F("FONA is OK"));


    uint16_t vbat;
    fona.getBattPercent(&vbat);

    DateTime   unixTime = rtc.now();
    int readingHour = unixTime.hour();
    int readingMinute = unixTime.minute();

    int readingYear   = unixTime.year();
    int readingMonth = unixTime.month();
    int readingDay = unixTime.day();

    //concatinating Test SMS time and date
    String readingTime = String(readingHour) + ":" + String(readingMinute);
    String readingDate = String(readingDay) + ":" + String(readingMonth) + ":" + String(readingYear);

    //Take temperature and distance reading
    int temperature = round(takeThermReading());
    int distance = round(takeDistanceReading());


    //formating contents for SMS
    String testMessageSMS =  String(i) + String(":Sensor:") + String(SENSOR_NUM) + String(" BV: ")  + String(vbat) + String("%") + String("Time:") + String(readingTime) + String("Date:") + String(readingDate) + String("T:") + temperature + String("D:") + distance;

    char messageSMS[121];
    strncpy(messageSMS, testMessageSMS.c_str(), sizeof(messageSMS));
    messageSMS[sizeof(messageSMS) - 1] = 0;


    if (waitForNetwork(WAIT_FOR_NETWORK)) {
      fona.sendSMS(PHONE_NUMBER_SMS, messageSMS);
    }


    //Turn off GSM.
    digitalWrite(FONA_PWR, HIGH);
    delay(2000);
    digitalWrite(FONA_PWR, LOW);
    delay(2000);

    sleep(15);

  }

}


void loop()
{



  Wire.begin();
  rtc.begin();

  DateTime   unixTime = rtc.now();
  int currentHour = unixTime.hour();

  //This will compare current hour and previous reading hour
  //If there were exact one hour difference or as we have identified in TAKE_READING_AFTER_X_HOUR
  //it will take reading and will save to array
  //readingFlag is set to false so it could take reading for the very first time
  if (readingFlag == false || currentHour - readingHour == TAKE_READING_AFTER_X_HOUR) {
    //It reacords reading hour such as 3,4,5
    readingHour = unixTime.hour();

    //if reading hour is 23, it will reset the reading hour varible to -1
    //if we don't do such it will subtract 0-23 = -23. this will not meet our requirment and it will not send SMS
    if (readingHour == 23) {
      readingHour = -1;
    }

    ////Take temperature and distance reading
    double temperature = takeThermReading();
    double distance = takeDistanceReading();

    //rounding temperature and distance reading
    int roundedTemperature = round(temperature);
    int roundedDistance = round(distance);


    // Cache distance and time in global array variable
    sensorReadingsSMS[numCachedReadings].sensorNum = SENSOR_NUM;
    //sensorReadingsEmail[numCachedReadings].vbat = 0 ;
    sensorReadingsSMS[numCachedReadings].timestamp = unixTime.unixtime();
    sensorReadingsSMS[numCachedReadings].distance = roundedDistance;
    sensorReadingsSMS[numCachedReadings].temperature = roundedTemperature;


    readingFlag = true;
    //increment array index
    numCachedReadings += 1;
  }



  //Look if number of readings in the Cache array is equal to the specefied number, in our case 3
  if (numCachedReadings == SEND_DATA_AFTER_X_READINGS)
  {
    //Turn on GSM.
    digitalWrite(FONA_PWR, HIGH);
    delay(2000);
    digitalWrite(FONA_PWR, LOW);
    delay(15000);


    fonaSerial->begin(4800);
    if (! fona.begin(*fonaSerial)) {
      Serial.println(F("Couldn't find FONA"));
      while (1);
    }


    uint16_t vbat;
    fona.getBattPercent(&vbat);


    String textMessageSMS = String(sensorReadingsSMS[0].sensorNum) + " " +
                            String(vbat) + " " +
                            String(sensorReadingsSMS[0].timestamp.unixtime()) + " " +
                            String(sensorReadingsSMS[0].distance) + " " +
                            String(sensorReadingsSMS[0].temperature);

    for (int i = 1; i < numCachedReadings; ++i)
    {
      //textMessageEmail += String(DATA_DELIM) + String(sensorReadingsEmail[i].sensorNum) + " " + String(vbat) + " " + String(sensorReadingsEmail[i].timestamp.unixtime()) + " " + String(sensorReadingsEmail[i].distance) + " " + String(sensorReadingsEmail[i].temperature);
      textMessageSMS += String(DATA_DELIM) + String(sensorReadingsSMS[i].sensorNum) + " " + String(vbat) + " " + String(sensorReadingsSMS[i].timestamp.unixtime()) + " " + String(sensorReadingsSMS[i].distance) + " " + String(sensorReadingsSMS[i].temperature);
    }


    char messageSMS[121];
    strncpy(messageSMS, textMessageSMS.c_str(), sizeof(messageSMS));
    messageSMS[sizeof(messageSMS) - 1] = 0;

    sd.begin();


    if (waitForNetwork(WAIT_FOR_NETWORK)) {
      //Sending SMS and writing to the log files on SD card
      if (fona.sendSMS(PHONE_NUMBER_SMS, messageSMS)) {
        sd.writeFile(UNSENT_FILENAME, messageSMS);
      } else {
        sd.writeFile(BACKUP_FILENAME, messageSMS);
      }
    } else {
      sd.writeFile(UNSENT_FILENAME, messageSMS);
    }
    //Turn off GSM.
    digitalWrite(FONA_PWR, HIGH);
    delay(2000);
    digitalWrite(FONA_PWR, LOW);
    delay(2000);

    // Reset number of cached readings
    numCachedReadings = 0;


  }
  sleep(37);
}
//
//
double takeThermReading()
{
  // 2. Turn on thermistor.
  // ----------------------
  digitalWrite(THERM_PIN, HIGH);

  // 3. Take 5 thermistor readings. (one every 20ms)
  // -----------------------------------------------
  int thermReadings[NUM_THERM_READINGS];
  for (int i = 0; i < NUM_THERM_READINGS; ++i)
  {
    thermReadings[i] = analogRead(THERMISTOR_PIN);
    delay(THERM_READING_DELAY);
  }

  // 4. Turn off thermistor.
  // -----------------------
  digitalWrite(THERM_PIN, LOW);
  delay(500);

  // 5. Average 5 thermistor readings.
  // ---------------------------------
  double sumTherm = 0;
  for (int i = 0; i < NUM_THERM_READINGS; ++i)
  {
    sumTherm += thermReadings[i];
  }
  double avgTherm = sumTherm / NUM_THERM_READINGS;
  avgTherm = 1023 / avgTherm - 1;
  double R = 10000 / avgTherm;


  // 6. Convert average thermistor reading into temperature.
  // -------------------------------------------------------

  // Steinhart-Hart, modified:
  double avgTemperature = ( 3950.0 / (log( R / (10000.0 * exp( -3950.0 / 298.13 ) ) ) ) ) - 273.13;

  return avgTemperature;
}

double takeDistanceReading()
{

  // Calibration time
  delay(3000);


  // 8. Take 3 distance readings. (One every 200ms)
  // ----------------------------------------------
  int distanceReadings[NUM_DISTANCE_READINGS];
  for (int i = 0; i < NUM_DISTANCE_READINGS; ++i)
  {

    //Only for US sensor
    int reading = analogRead(ULTRASONIC_PIN);
    distanceReadings[i] = reading * DISTANCE_INCREMENT;
    delay(DISTANCE_READING_DELAY);
  }


  // 9. Turn off ultrasonic US sensor (MOSFET).
  // ---------------------------------------
  //digitalWrite(MOSFET_US_PIN, LOW);
  delay(500);


  // 10. Average 3 distance measurements.
  // ------------------------------------
  double sumDistance = 0.0;
  for (int i = 0; i < NUM_DISTANCE_READINGS; ++i)
  {
    sumDistance += distanceReadings[i];
  }

  //averaging and converting to CM
  double avgDistance = (sumDistance / NUM_DISTANCE_READINGS) / 10;

  if (avgDistance < 0 )
  {
    avgDistance = 0;
  }
  return avgDistance;
}

//Search until SIM finds the network in the specific time. if not found in specific time it will jump out of the while loop
bool waitForNetwork(int secondsWait)
{
  //Counts number of seconds the microcontroller is trying to find the network
  int waitCount = 0;
  //Network status, Status 1 means newtwork found.
  uint8_t n = fona.getNetworkStatus();

  while (n != 1) {
    n = fona.getNetworkStatus();
    if (waitCount >= secondsWait)
    {
      //No network found in the defined time the MC will jump to loop
      break;
    }
    else {
      //add five seconds with network wait time
      waitCount += 5;
    }
    //wait five seconds for the network
    delay(5000);
  }

  if (n == 1) {
    delay(15000);
    return true;
  } else {
    return false;
  }
}
void sleep(int sleepCycle) {
  for (int i = 0; i < sleepCycle; ++i)
  {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    digitalWrite(FONA_PWR, HIGH);  //keeps the GSM off
  }
}

