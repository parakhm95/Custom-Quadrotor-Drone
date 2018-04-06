#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];


volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

#include <SPI.h>
#include "RF24.h"
#include <Servo.h>

RF24 resradio(9, 10);

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;


byte addresses[][6] = {"Hawan", "Pawan"};

void setup() {
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

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(250000);
  resradio.begin();
  resradio.setPALevel(RF24_PA_MAX);
  resradio.setPayloadSize(10);
  resradio.openWritingPipe(addresses[0]);
  resradio.openReadingPipe(1, addresses[1]);
  resradio.setAutoAck(false);
  resradio.setChannel(124);
  resradio.startListening();

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("\nSend any character to begin DMP programming and demo: "));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(125);
  mpu.setYGyroOffset(-42);
  mpu.setZGyroOffset(-12);
  mpu.setZAccelOffset(1235);

  if (devStatus == 0) {
	Serial.println(F("Enabling DMP..."));
	mpu.setDMPEnabled(true);

	Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
	attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
	mpuIntStatus = mpu.getIntStatus();

	Serial.println(F("DMP ready! Waiting for first interrupt..."));
	dmpReady = true;
	packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
	Serial.print(F("DMP Initialization failed (code "));
	Serial.print(devStatus);
	Serial.println(F(")"));
  }
  pinMode(LED_PIN, OUTPUT);
}


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

float pitchReq = -0.06, 
	  pitchErrNow = 0, 
	  pitchErrIgt = 0, 
	  pitchErrPrev = 0, 
	  pitchKp = 0, 
	  pitchKi = 0, 
	  pitchKd = 0;

float rollReq = -1.10, 
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


void motorWrite(int power1, int power2, int power3, int power4) {
	motor1.writeMicroseconds(power1);
	motor2.writeMicroseconds(power2);
	motor3.writeMicroseconds(power3);
	motor4.writeMicroseconds(power4);
}

void motorCal() {
	power1 = throttle - yaw - pitch + roll;
	power2 = throttle + yaw - pitch - roll;
	power3 = throttle - yaw + pitch - roll;
	power4 = throttle + yaw + pitch + roll;
	
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
	if (millis() - tcount > 3) {
    yawReq -= (constants[4] - 512) * 0.0006;
    pitchErrNow = pitchReq + ((float(constants[1])) / 1023 *30) - 15 - (ypr[1] * 180 / M_PI);
    rollErrNow = rollReq + ((float(constants[2])) / 1023 * 30) - 15 - (ypr[2] * 180 / M_PI);

		yawNow = ypr[0] * 180 / M_PI; 
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
		pitch = (pitchKp * pitchErrNow) + pitchKd * (-(ypr[1] * 180 / M_PI) + pitchErrPrev) + pitchKi * (pitchErrIgt);
		roll = (rollKp * rollErrNow) + rollKd * (-(ypr[2] * 180 / M_PI) + rollErrPrev) + rollKi * (rollErrIgt);

		yawPrev = yawNow;
		yawErrPrev = yawErrNow;
		pitchErrPrev = (ypr[1] * 180 / M_PI);
		rollErrPrev = (ypr[2] * 180 / M_PI);
		tcount = millis();
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
		yawReq = ypr[0] * 180 / M_PI;
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

float teemer;

void loop() {
  teemer=micros();
	if (!dmpReady) return;
	while (!mpuInterrupt && fifoCount < packetSize) {
    commread();
    if (throttle_flag == 1){
		motorUpdate();
    }
	}
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	fifoCount = mpu.getFIFOCount();

	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		mpu.resetFIFO();
	}
	else if (mpuIntStatus & 0x02) {
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		mpu.getFIFOBytes(fifoBuffer, packetSize);

		fifoCount -= packetSize;
		
		#ifdef OUTPUT_READABLE_YAWPITCHROLL
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//			Serial.print("ypr\t");
//			Serial.print(ypr[0] * 180 / M_PI);
//			Serial.print("\t");
//			Serial.print(ypr[1] * 180 / M_PI);
//			Serial.print("\t");
//			Serial.println(ypr[2] * 180 / M_PI);
		#endif
		blinkState = !blinkState;
		digitalWrite(LED_PIN, blinkState);
	}
	fly();
}
