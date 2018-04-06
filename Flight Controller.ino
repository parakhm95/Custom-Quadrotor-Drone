#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>
#include "RF24.h"
#include <Servo.h>
#include "HMC5883L.h"



int acc_axis[4], gyro_axis[4], temperature;
long acc_x, acc_y, acc_z, acc_total_vector;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll,angle_yaw;
int calib, startupt;
float ypr[3];
int16_t mx, my, mz;
float heading,headingReq,headingPrev;
int constants[5];
unsigned long commlastupdate = 0;

float yawReq = 0,
      yawNow=0,
      yawPrev=0, 
      yawErrNow = 0, 
      yawErrIgt = 0, 
      yawErrPrev = 0, 
      yawKp = 0, 
      yawKi = 0, 
      yawKd = 0;

float pitchReq = -0.19, 
      pitchErrNow = 0, 
      pitchErrIgt = 0, 
      pitchErrPrev = 0, 
      pitchKp = 0, 
      pitchKi = 0, 
      pitchKd = 0;

float rollReq = -1.41, 
      rollErrNow = 0, 
      rollErrIgt = 0, 
      rollErrPrev = 0, 
      rollKp = 0, 
      rollKi = 0, 
      rollKd = 0;


int throttle = 0;
int pitch = 0, roll = 0, yaw = 0;
unsigned long tcount = 0;
int startup = 0;
int power1, power2, power3, power4;
int throttle_flag = 0;
float teemer;


RF24 resradio(9, 10);
HMC5883L mag;

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;


byte addresses[][6] = {"Hawan", "Pawan"};



void setup(){

  motor1.attach(4);
  motor2.attach(5);
  motor3.attach(6);
  motor4.attach(7);
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(5000);
  digitalWrite(8, LOW);

  Wire.begin();
  TWBR = 12;
  setupgyro();
  mag.initialize();
  for(calib = 0; calib < 4000 ; calib++){
      readIMU();
      gyro_axis_cal[1] += gyro_axis[1];                                       //Ad roll value to gyro_roll_cal.
      gyro_axis_cal[2] += gyro_axis[2];                                       //Ad pitch value to gyro_pitch_cal.
      gyro_axis_cal[3] += gyro_axis[3];
      delay(3);
    }

    gyro_axis_cal[1] /= 4000;                                                 //Divide the roll total by 2000.
    gyro_axis_cal[2] /= 4000;                                                 //Divide the pitch total by 2000.
    gyro_axis_cal[3] /= 4000;
  readMag();
  headingReq=heading;
  headingPrev=heading;
  Serial.begin(250000);
  resradio.begin();
  resradio.setPALevel(RF24_PA_MAX);
  resradio.setPayloadSize(10);
  resradio.openWritingPipe(addresses[0]);
  resradio.openReadingPipe(1, addresses[1]);
  resradio.setAutoAck(false);
  resradio.setChannel(124);
  resradio.startListening();
}



unsigned long temmer;

void loop(){
  digitalWrite(13,HIGH);
  readMag();
  readIMU();
  fly();
//  Serial.print(gyro_pitch);
//  Serial.print("  ");
//  Serial.print(gyro_roll);
//  Serial.print("  ");
  Serial.print(angle_pitch);
  Serial.print("  ");
  Serial.println(angle_roll);
  while(micros()-temmer<10000){}
  temmer=micros();
}

void setupgyro(){

    Wire.beginTransmission(0x68);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(0x68);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(0x68);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(0x68);                                      //Start communication with the address found during search
    Wire.write(0x1B);                                                          //Start reading @ register 0x1B
    Wire.endTransmission();                                                    //End the transmission
    Wire.requestFrom(0x68, 1);                                         //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                               //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){                                                   //Check if the value is 0x08
      digitalWrite(12,HIGH);                                                   //Turn on the warning led
      while(1)delay(10);                                                       //Stay in this loop for ever
    }

    Wire.beginTransmission(0x68);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x04);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro    

}

void readMag(){
  mag.getHeading(&mx, &my, &mz);
  heading = atan2(my, mx);
    if(heading < 0){
      heading += 2 * M_PI;
    }
}

void readIMU(){
    Wire.beginTransmission(0x68);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(0x68,14);

    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_axis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_x variable.
    acc_axis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_y variable.
    acc_axis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_z variable.
    temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read()<<8|Wire.read();

    if(calib==4000){
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];
    

    gyro_roll = gyro_axis[1];
    gyro_pitch = gyro_axis[2];
    gyro_yaw = gyro_axis[3];

    acc_x = -acc_axis[2];
    acc_y = -acc_axis[1];
    acc_z = acc_axis[3];

    
    
    angle_pitch += gyro_pitch * 0.0001526;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    angle_roll += gyro_roll * 0.0001526; 
    angle_yaw += gyro_yaw * 0.0001526;//Calculate the traveled roll angle and add this to the angle_roll variable.


    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    angle_pitch -= angle_roll * sin(gyro_yaw * 0.000002664);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
    angle_roll += angle_pitch * sin(gyro_yaw * 0.000002664);                  //If the IMU has yawed transfer the pitch angle to the roll angel.
  
    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
    
    if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
      angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
    }
    if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
      angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
    }
    
    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
    angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
    angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.

    if(startupt==0){
    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;
    startupt=2;
    }
    
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

    headingPrev=heading;
}
}

void motorWrite(int power1, int power2, int power3, int power4) {
    motor1.writeMicroseconds(power1);
    motor2.writeMicroseconds(power2);
    motor3.writeMicroseconds(power3);
    motor4.writeMicroseconds(power4);
}

void motorCal() {
    power1 = throttle + yaw + pitch + roll;
    power2 = throttle - yaw + pitch - roll;
    power3 = throttle + yaw - pitch - roll;
    power4 = throttle - yaw - pitch + roll;
    
    power1 = constrain(power1, 1000, 2000);
    power2 = constrain(power2, 1000, 2000);
    power3 = constrain(power3, 1000, 2000);
    power4 = constrain(power4, 1000, 2000);
}

void motorUpdate() {
    motorCal();
    motorWrite(power1, power2, power3, power4);
}


void PIDcal(void) {
    if (1) {
    yawReq -= (constants[4] - 512) * 0.0006;
    pitchErrNow = pitchReq - ((float(constants[1])) / 1023 *70) + 35 - (angle_pitch);
    rollErrNow = rollReq + ((float(constants[2])) / 1023 * 70) - 35 - (angle_roll);

    yawNow = angle_yaw; 
   if(yawNow - yawPrev > 280)
   {
    yawReq += 360;
   }
   else if(yawNow - yawPrev < -280)
   {
    yawReq -= 360;
   }
   yawErrNow = yawReq - yawNow;
    yawErrIgt += yawErrNow;
    rollErrIgt += rollErrNow;
    pitchErrIgt += pitchErrNow;

        yaw = (yawKp * yawErrNow) + (yawKd * (yawErrNow - yawErrPrev)) + (yawErrIgt * yawKi);
        pitch = (pitchKp * pitchErrNow) + (pitchKd * (-angle_pitch + pitchErrPrev)) + (pitchKi * (pitchErrIgt));
        roll = (rollKp * rollErrNow) + (rollKd * (-angle_roll + rollErrPrev)) + (rollKi * (rollErrIgt));

        yawPrev = yawNow;
        yawErrPrev = yawErrNow;
        pitchErrPrev = (angle_pitch);
        rollErrPrev = (angle_roll);
    }
}

void commread(void) {
    if (resradio.available()) {
        resradio.read(&constants, sizeof(constants));
        commlastupdate = millis();
        throttle = map(constants[0], 0, 1023, 1000, 1700);
//    Serial.println(throttle);
        yawKp = 4;
        yawKi = 0.00525;
        yawKd = 0;
        pitchKp = 2.4;
        pitchKi = 0.0105;
        pitchKd = 62.4;
        rollKp = 2.4;
        rollKi = 0.0105;
        rollKd =  62.4;
    }

    if (millis() - commlastupdate > 4000) {
        throttledown();
    }
}

void throttledown(void) {
    //Serial.println("Getting throttle down!!");
  constants[1] = 512;
  constants[2] = 512;
  constants[4] = 512;
    for (; throttle > 1000 ; throttle -= 3) {
        digitalWrite(8, HIGH);
        PIDcal();
    motorUpdate();
        delay(100);
        digitalWrite(8, LOW);
    }
    throttle = 1000;
}

void fly() {
    if (millis() > 15000 && startup == 0) {
        yawReq = angle_yaw;
        startup = 1;
        digitalWrite(8, HIGH);
    }

    if (millis() < 15000) {
        yaw = 0;
        yawErrIgt = 0;
    }
    if (resradio.available() && throttle_flag == 0) {
        resradio.read(&constants, sizeof(constants));
        throttle = map(constants[0], 0, 1023, 1000, 1600);
        if (throttle < 1050) {
            throttle_flag = 1;
        }
    }
    else if (throttle_flag == 1) {
        commread();
        if (throttle > 1080) {
            PIDcal();
            motorUpdate();
        }
        else {
      yaw =0;
      pitch = 0;
      roll = 0;
            motorWrite(throttle, throttle, throttle, throttle);
        }
    }
}
