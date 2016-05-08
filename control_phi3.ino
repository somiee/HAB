#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>

SoftwareSerial mySerial(3, 2);

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *left = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *right = AFMS.getStepper(200, 2);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO false
#define pi 3.14159265358979323846

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

boolean correctDir = false;
float oldLat; //need to set this initially
float oldLong; //need to set this initially
float goalLat = 40.112428;
float goalLong = -88.228301;
float goalLat1r = (goalLat*pi) / 180;
float goalLong1r = (goalLong*pi) / 180;
float psi;
float maxAltitude = 200;
bool armed = false; //set this to false before launch!!!!!!!
int algorithmDelay = 30; //seconds

void setup(){
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  
  GPS.sendCommand(PGCMD_ANTENNA);
  
  useInterrupt(true);
  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  #ifdef UDR0
  if (GPSECHO)
  if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
  #endif
}
void useInterrupt(boolean v) {
  if (v) {
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function above
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  usingInterrupt = true;
  } else {
  // do not call the interrupt function COMPA anymore
  TIMSK0 &= ~_BV(OCIE0A);
  usingInterrupt = false;
  }
}

uint32_t timer = millis();
void loop(){
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
    if (c) Serial.print(c);
  }
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
  // a tricky thing here is if we print the NMEA sentence, or data
  // 
  if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    return; // we can fail to parse a sentence in which case we should just wait for another
  }
  
  if (timer > millis()) timer = millis();
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
  timer = millis(); // reset the timer

    if (GPS.fix) {
      if(!armed){
        checkArmed();
      }else{      
        algorithm();
      }
    }
  }
}

void checkArmed(){
  float currentAltitude = GPS.altitude;
  if(currentAltitude > maxAltitude ){
    maxAltitude = currentAltitude;
  }else if(maxAltitude - currentAltitude > 2000){
    armed = true;
  }
}

void algorithm(){
  int timeCount = 0;
  oldLat = GPS.latitudeDegrees;
  oldLong = GPS.longitudeDegrees;
  Serial.print("old lat/lon: ");
  Serial.print(oldLat,6);
  Serial.print("/ ");
  Serial.println(oldLong,6);
  while(timeCount < algorithmDelay){
    timeCount++;
    delay(1000);
  }
  if (GPS.newNMEAreceived()) {
  // a tricky thing here is if we print the NMEA sentence, or data
  // 
  if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    return; // we can fail to parse a sentence in which case we should just wait for another
  }
  float heading = getHeading();
  float theAngle = getAngle();
  float psi = theAngle - heading;
  Serial.print("new lat/lon: ");
  Serial.print(oldLat,6);
  Serial.print("/ ");
  Serial.println(oldLong,6);
  Serial.print("heading: ");
  Serial.println(heading);
  Serial.print("angle: ");
  Serial.println(theAngle);
  //turn motor
  psi = psi + 360.0;
  psi = psi / 360.0;
  if (psi > 1) {
    psi = psi - 1;
  }
  psi = psi * 360.0;
  Serial.print("psi: ");
  Serial.println(psi);
  if(psi >= 180 && psi <= 350){
    for(int i = 0; i < 8; i++){
     left->step(200, FORWARD, DOUBLE);
    }
    left->release();
    //Serial.println("Turning right motor 8 times");
  }else if(psi >= 10 && psi < 180){
    for(int i = 0; i < 8; i++){
     right->step(200, FORWARD, DOUBLE);
    }
    right->release();
   //Serial.println("Turning left motor 8 times");
  }
  
}

float getHeading(){
  float gpslatitudeinitial = oldLat;
  float gpslongitudeinitial = oldLong;
  //Serial.print("Initial pt:");
  //Serial.println(gpslatitudeinitial); Serial.println(",");
  //Serial.println(gpslongitudeinitial);
  float gpslatitudefin = GPS.latitudeDegrees;
  float gpslongitudefin = GPS.longitudeDegrees;
  //Serial.print("Final pt:");
  //Serial.println(GPS.latitudeDegrees); Serial.println(",");
  //Serial.println(gpslongitudefin);
  float gpslatitudeinitial1 = (gpslatitudeinitial * pi) / 180.0;
  float gpslongitudeinitial1 = (gpslongitudeinitial * pi) / 180.0;
  //convert to radians
  float gpslatitudefin1 = (gpslatitudefin * pi) / 180.0;
  float gpslongitudefin1 = (gpslongitudefin * pi) / 180.0;
  float diffLon = (gpslongitudefin1 - gpslongitudeinitial1);

  float yh = sin(diffLon) * cos(gpslatitudefin1);
  float xh = cos(gpslatitudeinitial1) * sin(gpslatitudefin1) - sin(gpslatitudeinitial1)
             * cos(gpslatitudefin1) * cos(diffLon);

  float heading = atan2(yh, xh);
  float heading1 = (heading * 180.0) / pi;

  oldLat = gpslatitudefin;
  oldLong = gpslongitudefin;
  heading1 = heading1 + 360.0;
  heading1 = heading1 / 360.0;
  if (heading1 > 1) {
    heading1 = heading1 - 1;
  }
  heading1 = heading1 * 360.0;
  return heading1;
}

float getAngle(){
  //convert to radians
  float lat2r = (oldLat * pi) / 180.0;
  float long2r = (oldLong * pi) / 180.0;

  float dLon = (goalLong1r - long2r);

  float y = sin(dLon) * cos(goalLat1r);
  float x = cos(lat2r) * sin(goalLat1r) - sin(lat2r)
            * cos(goalLat1r) * cos(dLon);

  float brng = atan2(y, x);
  brng = (brng * 180.0) / pi;
  //brng = (int(brng) + 360) % 360; //angle with respect to north degrees
  brng = brng + 360.0;
  brng = brng / 360.0;
  if (brng > 1) {
    brng = brng - 1;
  }
  brng = brng * 360.0;
  return brng;
}
