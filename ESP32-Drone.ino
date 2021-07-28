#include <IBusBM.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <MS5x.h>
#include <ESP32Servo.h> // 
#include "ESC.h" // RC_ESP library
#include "RunningMedian.h"



#define ESC_PIN0 (13) // connected to ESC control wire
#define ESC_PIN1 (12) // connected to ESC control wire
#define ESC_PIN2 (14) // connected to ESC control wire
#define ESC_PIN3 (27) // connected to ESC control wire
#define LED_BUILTIN (2) // not defaulted properly for ESP32s/you must define it
#define MIN_SPEED 1000 // speed just slow enough to turn motor off
#define SPIN_SPEED 1055 // speed just slow enough to turn motor off
#define MAX_SPEED 2000 // speed where my motor drew 3.6 amps at 12v.
#define ARM_SPEED 500 // arm value
#define GYRO0 32
#define GYRO1 33
#define GYRO2 34
#define GYRO3 35

IBusBM IBus;    // IBus object
ESC myESC0 (ESC_PIN0, MIN_SPEED, MAX_SPEED, ARM_SPEED); // ESC_Name (PIN, Minimum Value, Maximum Value, Arm Value)
ESC myESC1 (ESC_PIN1, MIN_SPEED, MAX_SPEED, ARM_SPEED); 
ESC myESC2 (ESC_PIN2, MIN_SPEED, MAX_SPEED, ARM_SPEED); 
ESC myESC3 (ESC_PIN3, MIN_SPEED, MAX_SPEED, ARM_SPEED); 
Adafruit_MPU6050 mpu;
MS5x barometer(&Wire);
RunningMedian samplesX = RunningMedian(20);
RunningMedian samplesY = RunningMedian(20);
RunningMedian samplesALT = RunningMedian(30);

long int val; // variable to read the value from the IBus
unsigned long prevTime = 0;
unsigned long prevTime2 = 0;
double prevPressure=0; // The value of the pressure the last time the sensor was polled
double prevTemperature=0; // The value of the temperature the last time the sensor was polled
double seaLevelPressure = 0;
float gyro_x_cal, gyro_y_cal;
float gyro_x, gyro_y, gyro_z;
double pressure = 0;
double temperature = 0;
double altitude = 0;
  int firstrun = 1;
  int battery_voltage;
  float holdAltitude;
  int holdThrottle;


void setup() {
  Serial.begin(115200);     // debug info
  while (!Serial)
    delay(10); // will pause etc until serial console opens
    
  Serial.println("Start IBus2PWM_ESP32");
  IBus.begin(Serial2,1);    // iBUS object connected to serial2 RX2 pin and use timer 1

  delay(1000);
  pinMode(ESC_PIN0, OUTPUT);
  pinMode(ESC_PIN1, OUTPUT);
  pinMode(ESC_PIN2, OUTPUT);
  pinMode(ESC_PIN3, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // set led to on to indicate arming
  
//calibrate ESCs by setting them to high till it clicks twice, then to lowest until it clicks three times, trips when powered on and SWC is in the middle
  if (IBus.readChannel(8) == 1500){
    myESC0.speed(MAX_SPEED);
    myESC1.speed(MAX_SPEED);
    myESC2.speed(MAX_SPEED);
    myESC3.speed(MAX_SPEED);
    delay(6000);
    myESC0.speed(MIN_SPEED);
    myESC1.speed(MIN_SPEED);
    myESC2.speed(MIN_SPEED);
    myESC3.speed(MIN_SPEED);
    delay(5000); 
  }
  Serial.println("Arming ESCs"); 
  myESC0.arm(); // Send the Arm command to ESC
  myESC1.arm(); 
  myESC2.arm(); 
  myESC3.arm(); 
  delay(5000); 
  digitalWrite(LED_BUILTIN, LOW); // led off to indicate arming completed

  // the following loop turns on the motor slowly, so get ready
  Serial.println("Spinning up Motors"); 
  for (int i=0; i<60; i++){ // run speed from 1045 to 1060
    myESC0.speed(1000+i); // spin motor up a little (13)
    myESC1.speed(1000+i); // spin motor up a little (12)
    myESC2.speed(1000+i); // spin motor up a little (14)
    myESC3.speed(1000+i); // spin motor up a little (27)
    delay(10);
  }
  Serial.println("Setting motors to minimum"); 
  myESC0.speed(1000); // spin motor down (13)
  myESC1.speed(1000); // spin motor down (12)
  myESC2.speed(1000); // spin motor down (14)
  myESC3.speed(1000); // spin motor down (27)
  
  pinMode(GYRO0, OUTPUT);
  pinMode(GYRO1, OUTPUT);
  pinMode(GYRO2, OUTPUT);
  pinMode(GYRO3, OUTPUT);

  for (int i = 0; i < 4; i++) {
    if (i == 0) {
      digitalWrite(GYRO0, LOW);
      digitalWrite(GYRO1, HIGH);
      digitalWrite(GYRO2, HIGH);
      digitalWrite(GYRO3, HIGH);
    }
    if (i == 1) {
      digitalWrite(GYRO0, HIGH);
      digitalWrite(GYRO1, LOW);
      digitalWrite(GYRO2, HIGH);
      digitalWrite(GYRO3, HIGH);
    }
    if (i == 2) {
      digitalWrite(GYRO0, HIGH);
      digitalWrite(GYRO1, HIGH);
      digitalWrite(GYRO2, LOW);
      digitalWrite(GYRO3, HIGH);
    }
    if (i == 3) {
      digitalWrite(GYRO0, HIGH);
      digitalWrite(GYRO1, HIGH);
      digitalWrite(GYRO2, HIGH);
      digitalWrite(GYRO3, LOW);
    }

    Serial.print("Initializing MPU6050 #");
    Serial.println(i);
    // Try to initialize!
    if (!mpu.begin()) {
      Serial.print("Error with MPU6050 #");
      Serial.print(i);
      Serial.println(", check wiring!");
      while (1) {
        delay(10);
      }
    }
    Serial.print("MPU6050 #");
    Serial.print(i);
    Serial.println(" Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    Serial.print("MPU6050 #");
    Serial.print(i);
    Serial.print(" - Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
      case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
      case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
      case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
      case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    Serial.print("MPU6050 #");
    Serial.print(i);
    Serial.print(" - Gyro range set to: ");
    switch (mpu.getGyroRange()) {
      case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
      case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
      case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
      case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    Serial.print("MPU6050 #");
    Serial.print(i);
    Serial.print(" - Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
      case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
      case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
      case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
      case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
      case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
      case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
      case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }

    Serial.println("");
    delay(100);

  }
  
//setup altimider
  while(barometer.connect()>0) { // barometer.connect starts wire and attempts to connect to sensor
    Serial.println(F("Error connecting..."));
    delay(500);
  }
  Serial.println(F("Connected to Sensor"));
  barometer.setDelay(1000); // barometer will wait 1000 ms before taking new temperature and pressure readings
  
}


void loop() {
  float xm = samplesX.getMedian();
  float ym = samplesY.getMedian();
  float am = samplesALT.getMedian();
  unsigned long timer = millis()/1000;
  int x, y;
  int quadCopterFL = 1000;
  int quadCopterFR = 1000;
  int quadCopterRL = 1000;
  int quadCopterRR = 1000;
  int quadCopterFLmotor = 1000;
  int quadCopterFRmotor = 1000;
  int quadCopterRLmotor = 1000;
  int quadCopterRRmotor = 1000;
  int userInputLUD = 0;
  int userInputLLR = 0;
  int userInputRUD = 0;
  int userInputRLR = 0;
  int userInputSWA = 0;
  int userInputSWB = 0;
  int userInputSWD = 0;
  float gyrotemp;
  int altFEET;

  unsigned long currentMillis = millis(); 
  /* 
  //run everything inside here every second
  if ((unsigned long)(currentMillis - prevTime) >= 500) {
   prevTime = currentMillis;
  }
*/
  //calculate battery level from drop down
  battery_voltage = (analogRead(4) + 65) * 1.2317;
  battery_voltage = battery_voltage * 0.92 + (analogRead(4) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(LED_BUILTIN, HIGH);


  readGyro(&x, &y);
  samplesX.add(x);
  samplesY.add(y);
  samplesALT.add(altitude);

  
  altFEET = (int)am/.3048;
  userInputLUD = map(IBus.readChannel(2), 1000, 2000, 1000, 2000); //left stick / up/down
  userInputLLR = map(IBus.readChannel(3), 1000, 2000, -500, 500); //left stick / left/right
  userInputRUD = map(IBus.readChannel(1), 1000, 2000, -500, 500); //right stick / up/down
  userInputRLR = map(IBus.readChannel(0), 1000, 2000, -500, 500); //right stick / left/right
  userInputSWD = IBus.readChannel(9);
  userInputSWA = IBus.readChannel(6);
  userInputSWB = IBus.readChannel(7);

  if (userInputSWD == 1000){  //if SWD is up, then hold altitude
    if (firstrun == 1) {holdAltitude = altFEET; holdThrottle = userInputLUD; firstrun = 0;}  //first time on? set throttle and alt to current
    if (firstrun == 0) { //is on, not new

  //run everything inside here every half second
  if ((unsigned long)(currentMillis - prevTime) >= 500) {
      prevTime = currentMillis;
      if (altFEET < holdAltitude) {holdThrottle += 10;}  // getting lower, add some speed
      if (altFEET > holdAltitude) {holdThrottle -= 10;}  //getting higher, reduce some speed      
  }
    }
    userInputLUD = holdThrottle; //set the user left stick(throttle) to whatever the altimiter wants
  }  
  if (userInputSWD == 2000){firstrun = 1;} //if SWD is down, then reset hold altittude
 
  if (userInputLUD > 1000 && userInputLUD < SPIN_SPEED){userInputLUD = 1000;}   
  if (userInputSWA == 1000 && userInputLUD < SPIN_SPEED){userInputLUD = SPIN_SPEED+1;}  //safety switch, if SWA is up, then don't let the motors turn off completely

  //algorithms for movement
  if ((userInputLUD >= SPIN_SPEED)) { 
    quadCopterFL = userInputLUD + userInputRLR - userInputRUD + userInputLLR;
    quadCopterFR = userInputLUD - userInputRLR - userInputRUD - userInputLLR;
    quadCopterRL = userInputLUD + userInputRLR + userInputRUD - userInputLLR;
    quadCopterRR = userInputLUD - userInputRLR + userInputRUD + userInputLLR;
  }
  if ((userInputRUD > -5 && userInputRUD < 5) && (userInputRLR > -5 && userInputRLR < 5) && (userInputSWB == 1000) && (userInputLUD > SPIN_SPEED+1)) {  //if user not moving right stick, SWB is up, and it's not idling, let auto-level work, rotate will still work
    quadCopterFL = userInputLUD + ((int)(xm - 1000)/2) + userInputLLR;
    quadCopterFR = userInputLUD - ((int)(xm - 1000)/2) - userInputLLR;
    quadCopterRL = userInputLUD + ((int)(ym - 1000)/2) - userInputLLR;
    quadCopterRR = userInputLUD - ((int)(ym - 1000)/2) + userInputLLR;
  }

/*
  if (battery_voltage < 1240 && battery_voltage > 800){  //Is the battery connected?
    quadCopterFL += quadCopterFL * ((1240 - battery_voltage)/(float)3500);   //Compensate the ESC pulse for voltage drop.
    quadCopterFR += quadCopterFR * ((1240 - battery_voltage)/(float)3500);
    quadCopterRL += quadCopterRL * ((1240 - battery_voltage)/(float)3500);
    quadCopterRR += quadCopterRR * ((1240 - battery_voltage)/(float)3500);
  } 
*/
  if (userInputLUD >= SPIN_SPEED && quadCopterFL < SPIN_SPEED){quadCopterFL = SPIN_SPEED;}  //don't let the motors drop below spinning speed
  if (quadCopterFL > 2000){quadCopterFL = 2000;}                          //don't let the motor go faster than max
  if (userInputLUD >= SPIN_SPEED && quadCopterFR < SPIN_SPEED){quadCopterFR = SPIN_SPEED;}
  if (quadCopterFR > 2000){quadCopterFR = 2000;}
  if (userInputLUD >= SPIN_SPEED && quadCopterRL < SPIN_SPEED){quadCopterRL = SPIN_SPEED;}
  if (quadCopterRL > 2000){quadCopterRL = 2000;}
  if (userInputLUD >= SPIN_SPEED && quadCopterRR < SPIN_SPEED){quadCopterRR = SPIN_SPEED;} 
  if (quadCopterRR > 2000){quadCopterRR = 2000;}
  
  quadCopterFLmotor = map(quadCopterFL, 1000, 2000, MIN_SPEED, MAX_SPEED); // scale input to valid speed range
  quadCopterFRmotor = map(quadCopterFR, 1000, 2000, MIN_SPEED, MAX_SPEED); 
  quadCopterRLmotor = map(quadCopterRL, 1000, 2000, MIN_SPEED, MAX_SPEED); 
  quadCopterRRmotor = map(quadCopterRR, 1000, 2000, MIN_SPEED, MAX_SPEED); 

  Serial.print(" FL power level = "); 
  Serial.print(quadCopterFLmotor);
  Serial.print(",FR power level = "); 
  Serial.print(quadCopterFRmotor);
  Serial.print(",RL power level = "); 
  Serial.print(quadCopterRLmotor);
  Serial.print(",RR power level = "); 
  Serial.println(quadCopterRRmotor);

  myESC0.speed(quadCopterFLmotor); // sets the ESC speed
  myESC1.speed(quadCopterFRmotor); 
  myESC2.speed(quadCopterRLmotor); 
  myESC3.speed(quadCopterRRmotor); 

}


void readGyro(int *gyroDataXaverage, int *gyroDataYaverage) {
  int gyroDataX[3];
  int gyroDataY[3];
  float gyroDataTemp[3];
  memset(gyroDataX, 0, sizeof(gyroDataX));
  memset(gyroDataY, 0, sizeof(gyroDataY));

  for (int i = 0; i < 5; i++) {  //cycle through each MPU6050 sensor 0-3, and the pressure sensor at 4
    if (i == 0) {
      digitalWrite(GYRO0, LOW);  //set every pin but 32 to HIGH
      digitalWrite(GYRO1, HIGH);
      digitalWrite(GYRO2, HIGH);
      digitalWrite(GYRO3, HIGH);
    }
    if (i == 1) {
      digitalWrite(GYRO0, HIGH);  //set every pin but 33 to HIGH
      digitalWrite(GYRO1, LOW);
      digitalWrite(GYRO2, HIGH);
      digitalWrite(GYRO3, HIGH);
    }
    if (i == 2) {
      digitalWrite(GYRO0, HIGH);  //set every pin but 34 to HIGH
      digitalWrite(GYRO1, HIGH);
      digitalWrite(GYRO2, LOW);
      digitalWrite(GYRO3, HIGH);
    }
    if (i == 3) {
      digitalWrite(GYRO0, HIGH);  //set every pin but 35 to HIGH
      digitalWrite(GYRO1, HIGH);
      digitalWrite(GYRO2, HIGH);
      digitalWrite(GYRO3, LOW);
    }
    if (i == 4) {
      barometer.checkUpdates();
      if (barometer.isReady()) { 
        temperature = barometer.GetTemp(); // Returns temperature in C
        pressure = barometer.GetPres(); // Returns pressure in Pascals
        if (seaLevelPressure == 0) seaLevelPressure = barometer.getSeaLevel(86.55); //Calculate predicted seaLevel pressure based off a known altitude in meters
        altitude = barometer.getAltitude(true);
      }   
    }

    if ((i >= 0) && (i <= 3)) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroDataX[i] = a.acceleration.x * 100;
    gyroDataY[i] = a.acceleration.y * 100;
    }
    delay(2);
  }

  int Xmax = 0;
  int Xmin = 65535;
  int Xsum = 0;
  int Xtemp;
  int Ymax = 0;
  int Ymin = 65535;
  int Ysum = 0;
  int Ytemp;
  for (int i = 0; i < 4; i++) {  
    Xtemp = gyroDataX[i] + 1000;
    Ytemp = gyroDataY[i] + 1000;
    if (Xtemp < Xmin) Xmin = Xtemp;
    if (Ytemp < Ymin) Ymin = Ytemp;
    if (Xtemp > Xmax) Xmax = Xtemp;
    if (Ytemp > Ymax) Ymax = Ytemp;
    Xsum += Xtemp;
    Ysum += Ytemp;
  }
  //Now we have the sum of 4 readings, and the max & min readings, Olympic Average
  Xsum -= Xmin;
  Ysum -= Ymin;
  Xsum -= Xmax; //subtract the min and max from the sum, now only 2 are left
  Ysum -= Ymax; //subtract the min and max from the sum, now only 2 are left
  Xtemp = (Xsum / 2); 
  Ytemp = (Ysum / 2);

  delay(4);
  *gyroDataXaverage = Xtemp;
  *gyroDataYaverage = Ytemp;
}