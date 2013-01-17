/*
 Robot motor control
 
 This sketch is used to control a Pololu md08a Mototr controller from an arduino mega
 
 Authour: Alex Angell
 Date: October 2011
 Version: 1.0
 
 */
#include "pins_arduino.h"

 
#define COUNTS_REV 48 //Number of counts per revolution
#define WHEEL_D 42    //Diameter of wheel in mm
#define WHEEL_SPACE 88 //space between wheel centres in mm
#define Pi 3.14159265  //Mathmatical constant Pi
const float WHEEL_C = Pi * WHEEL_D;  //Circumfrence of wheel in mm
const float COUNT_MM = WHEEL_C/COUNTS_REV;
const int MAXSPEED = 50;            //Maximum speed limit for motors in 
const int MINSPEED = -MAXSPEED;

const int MAXDIST = 100;
const int MINDIST = -MAXDIST;

//struct 

//Define Pin numbers
const int PWMA = 13;
const int AIN1 = 22;
const int AIN2 = 23;
const int PWMB = 12;
const int BIN1 = 24;
const int BIN2 = 25;
const int MotStby = 26;
const int PinEncoderA_A = 2;
const int PinEncoderA_B = 3;
const int PinEncoderB_A = 18;
const int PinEncoderB_B = 19;

//Encoder Variables
volatile long int countA = 0;
volatile long int countB = 0;
volatile char EncoderA_Aprev = 0;
volatile char EncoderA_A = 0;
volatile char EncoderB_Aprev = 0;
volatile char EncoderB_A = 0;

//Control Variables
class PID{
  public:
    PID();
    
    
  private:
    float Kp;
    float Ki;
    float Kd;
    int dT;
};

  

//Rotation Variables
float rpmA=0;
float rpmB = 0;

//Position Variables
float posKp=10;//Position control gain
float posKi=0;
float posKd=0;
int targetPosA=0;
int targetPosB=0;
int posErrorA=0;
int prev_posErrorA=0;
int posErrorB=0;
int prev_posErrorB=0;
float posIntegralA=0;
float posIntegralB=0;

//Speed Variables
volatile float SpeedA = 0;    // currnet speed of mottor A
volatile float SpeedB = 0;    // currnet speed of mottor B
int targetSpeedA = 0;       //Speed motor A is to get too
int targetSpeedB = 0;       //Speed motor B is to get too
long int currentTime = 0;
long int prevTime=0;
long int prevCountA=0;

int path[10][2] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};

//Serial Variables
int incomingByte = 0;
char incomingData[128];
int availableChars = 0;

void setup()  { 
  
  randomSeed(1);
  Serial.begin(115200);
  // declare pin 13 to be an output:
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PinEncoderA_A, INPUT);
  pinMode(PinEncoderA_B, INPUT);
  pinMode(PinEncoderB_A, INPUT);
  pinMode(PinEncoderB_B, INPUT);
  EncoderA_A=digitalRead(PinEncoderA_A);
  EncoderB_A=digitalRead(PinEncoderB_A);
  attachInterrupt(0,encoderA,CHANGE);
  attachInterrupt(1,encoderA,CHANGE);
  attachInterrupt(5,encoderB,CHANGE);
  attachInterrupt(4,encoderB,CHANGE);
  
  digitalWrite(MotStby,LOW); //Make sure motor controller is off
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,LOW);
  digitalWrite(MotStby,HIGH); //Turn on the motor controller

  
  targetPosA=5*((Pi*89)/COUNT_MM); //28cm
  targetPosB=-targetPosA;
  prevCountA=countA;
  interrupts();
} 

void loop()  { 
  
    if (targetPosA==countA){
      //targetPosA = random(MINDIST,MAXDIST+1);
    }  
//    else if (targetSpeedA>SpeedA){
//      SpeedA++;
//    }
//    else if (targetSpeedA<SpeedA){
//      SpeedA--;
//    }
    
    if (targetPosB==countB){
      //targetPosB = random(MINDIST,MAXDIST+1);
    }  
//    else if (targetPosB>countB){
//      targetPosB++;
//    }
//    else if (targetPosA<countA){
//      targetPosB--;
//    }
//    
//    setMotor('A',SpeedA);
//    setMotor('B',SpeedB);

    prevTime=currentTime;
    currentTime=millis();
    
    rpmA = ((float)(countA-prevCountA)/((float)(currentTime-prevTime)/1250));
//    if((countA-prevCountA)!=0){
//      Serial.print("Count ");
//      Serial.println(countA-prevCountA,DEC);
//      Serial.print("Time ");
//      Serial.println(((float)((float)(currentTime-prevTime))/1250),8);
//      Serial.print("RPM");
//      Serial.println(rpmA,2);
//    }
    prevCountA=countA;
    
    posErrorA = targetPosA-countA;
    posIntegralA += (posErrorA*((float)(currentTime-prevTime)/1000));
    targetSpeedA = (posKp*posErrorA) + (posKi*posIntegralA);
    setMotor('A',targetSpeedA);
     
    posErrorB = targetPosB-countB;
    posIntegralB += (posErrorB*((float)(currentTime-prevTime)/1000));
    targetSpeedB = (posKp*posErrorB) + (posKi*posIntegralB);
    setMotor('B',targetSpeedB);  
    
    CheckSerial();
    delay(50);    
    //Serial.println(rpmA,4);
    //Serial.print("    ");
    //Serial.println(countB);
  
  /*
  for (int i=0; i==targetSpeed; i++){ 
    Speed = Speed + (i*fadeDir);
    analogWrite(PWMA, Speed);
    delay(500);
  }*/
  
}

void CheckSerial(){

  if (Serial.available() > 0) {
    delay(5);
    availableChars = Serial.available();
    // read the incoming data:
    for(int k=0;k!=availableChars+1;k++){ 	
      incomingData[k] = Serial.read();
    };
    incomingData[availableChars+1]=10;
    for(int k=0;k<availableChars;k++){ 	
       Serial.println(incomingData[k]);
    };
    if(incomingData[0]=='M'||incomingData[0]=='m'){
      incomingData[0]=' ';
      if(incomingData[1]=='A'||incomingData[1]=='a'){ 
        incomingData[1]=' ';
        targetPosA = atoi(incomingData);
        Serial.print("Motor A : ");
        Serial.println(targetPosA);
      }
      else if (incomingData[1]=='B'||incomingData[1]=='b'){ 
        incomingData[1]=' ';
        targetPosB = atoi(incomingData);
        Serial.print("Motor B : ");
        Serial.println(targetPosB);
      }
      else{
        targetPosA = atoi(incomingData);
        targetPosB =targetPosA;
        Serial.print("Motor A & B : ");
        Serial.println(targetPosB);
      };
    }
    else if (incomingData[0]=='K'||incomingData[0]=='k'){ 
      incomingData[0]=' ';
      if (incomingData[1]=='P'||incomingData[1]=='p'){
        incomingData[1]=' ';
        posKp = atoi(incomingData);
        Serial.print("Kp = ");
        Serial.println(posKp);
      }
      else if (incomingData[1]=='I'||incomingData[1]=='i'){
        incomingData[1]=' ';
        posKi = atoi(incomingData);
        Serial.print("Ki = ");
        Serial.println(posKi);
      }
      else{
        Serial.print("Kp = ");
        Serial.println(posKp);
        Serial.print("Ki = ");
        Serial.println(posKi);
        Serial.print("Kd = ");
        Serial.println(posKd);  
      }  
    }
    else{
      targetPosA = atoi(incomingData);
      targetPosB =targetPosA;
      Serial.print("Motor A & B : ");
      Serial.println(targetPosB);
    };
    Serial.flush();
  }  
}

void encoderA(){
  //Encoder A pin A attached to arduino pin 2, microprocessor PE4
  //Encoder A pin A attached to arduino pin 2, microprocessor PE5
  EncoderA_Aprev=EncoderA_A;  //Set previous state of Encoder A pin A
  EncoderA_A=bitRead(PINE,4);  //Read current state of encoder A pin A
  //XOR previous state of Encoder A pin A with current state of Encoder A pin B to calculate direction of rotation
  if(EncoderA_Aprev^(bitRead(PINE,5))){  
   countA--; 
  }
  else{
   countA++;
  } 
}

void encoderB(){
  EncoderB_Aprev=EncoderB_A;
  EncoderB_A=bitRead(PIND,3);
  if(EncoderB_Aprev^bitRead(PIND,2)){
   countB++;
  }
  else{
   countB--;
  }  
}
int asciiToDec(char asciiArray []){
  int answer=0;  //decimal answer to return
  int length =0; //length of string
  int unit = 0;
  
  for(int i=0;i!=129;i++){
     if(asciiArray[i]=='\n') length=i;
  }
  
  for (int j=length-1;j!=-1;j--){
    if(asciiArray[j]>=48 && asciiArray[j]<=57){
      asciiArray[j] -= 48; //conver ascii number representation to decimal
    }
    else{
      if(!(j==0 && asciiArray[j]=='-')) asciiArray[j] = 0;
    }
    if(!(j==0 && asciiArray[j]=='-')) answer += asciiArray[j]*(10);
    unit++;
  }
  if(asciiArray[0]=='-') answer = -answer;
  return answer; 
}

void setMotor(char motor, int Speed){
 
  
  
  if(Speed >255){
    Speed=255;
  }else if(Speed <-255) {
    Speed = -255;
  }
  if(Speed >MAXSPEED){
    Speed=MAXSPEED;
  }else if(Speed <MINSPEED) {
    Speed = MINSPEED;
  }  

  if(motor == 'A'){
  if (Speed == 0){
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,LOW);  
  }
  else if (Speed>0){
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
  }
  else if (Speed<0){
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);  
  }
  analogWrite(PWMA, abs(Speed));
 }
 else if(motor == 'B'){
  if (Speed == 0){
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,LOW);  
  }
  else if (Speed>0){
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
  }
  else if (Speed<0){
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);  
  }
  analogWrite(PWMB, abs(Speed));
 }
}
