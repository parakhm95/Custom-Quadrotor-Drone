int yawcal = -89,
    pitchcal = 136,
    rollcal = 96;

int arr[5] = {0,0,0,0,0};
float arr1[2] = {0,0};

#include <SPI.h>
#include "RF24.h"


RF24 transradio(9,10);

byte addresses[][6] = {"Hawan","Pawan"};

void setup() {
  Serial.begin(115200);
  digitalWrite(A0, INPUT_PULLUP);
  digitalWrite(A1, INPUT_PULLUP);
  pinMode(10,OUTPUT);
  //transradio.setAutoAck(false);
 
  transradio.begin();
  transradio.setPALevel(RF24_PA_MAX);
  transradio.setPayloadSize(10);
  //transradio.printDetails();
  transradio.openWritingPipe(addresses[1]);
  transradio.openReadingPipe(1,addresses[0]);
  transradio.setAutoAck(false);
  transradio.setRetries(15,15);
  transradio.setDataRate(RF24_1MBPS);
  transradio.setChannel(124);

}

/*
A0-RightSwitch
A1-LeftSwitch
A2-RightPot
A3-Pitch - 64 - 377 - 700
A4-Throttle - 0 - 655
A5-Yaw - 235 - 601 - 983
A6-Roll - 20 - 415 - 773
A7-LeftPot
*/

void loop() {
  arr[0]=map(analogRead(A4),0,700,0,1023);
  arr[1]=constrain(analogRead(A3)+pitchcal,0,1023);
  arr[2]=constrain(analogRead(A6)+rollcal,0,1023);
  arr[3]=analogRead(A0);
  arr[4]=constrain(analogRead(A5)+yawcal,0,1023);

 transradio.stopListening();
 delay(2);


if(!transradio.write(&arr,sizeof(arr),1)){
  //Serial.println("Sending Failed");
  Serial.print(arr[0]);
   Serial.print(",");
   Serial.print(arr[1]);
   Serial.print(",");
   Serial.print(arr[2]);
   Serial.print(",");
   Serial.print(arr[3]);
   Serial.print(",");
   Serial.print(arr[4]);
   Serial.print(",");
   Serial.println("");
   digitalWrite(10,LOW);
    }
  else
  {
   //Serial.println("Sending Success");
   Serial.print("$");
   Serial.print(arr[0]);
   Serial.print(" ");
   Serial.print(arr[1]);
   Serial.print(" ");
   Serial.print(arr[2]);
   Serial.print(" ");
   Serial.print(arr[3]);
   Serial.print(" ");
//   Serial.print(arr[4]);
//   Serial.print("");
   Serial.println(";");
   digitalWrite(10,HIGH);
  }
  // put your main code here, to run repeatedly:

}
