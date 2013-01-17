/*
 Robot control
 
 This sketch is used to control a Mobile Robot using an arduino mega ADK
 
 Authour: Alex Angell
 Date: 17 January 2013
 Version: 1.1
 
 */
#include "PIns_arduino.h"
 
#define COUNTS_REV 48 //Number of counts per revolution
#define WHEEL_D 42    //Diameter of wheel in mm
#define WHEEL_SPACE 88 //space between wheel centres in mm
//#define PI 3.14159265  //Mathmatical constant Pi

const float WHEEL_C = PI * WHEEL_D;  //Circumfrence of wheel in mm
const float COUNT_MM = WHEEL_C/COUNTS_REV;

const int MAXSPEED = 255;            //Maximum speed limit for motors in 
const int MINSPEED = -MAXSPEED;

const int MAXPWM = 100;            //Maximum drive limit for motors in %
const int MINPWM = -MAXPWM;

const int MAXDIST = 100;            //Maximum distance command 
const int MINDIST = -MAXDIST;

//Control Variables

//PID Class
class PID{
  public:
    PID(){
      Kp = 1;
      Ki = 0;
      Kd = 0;
    };

    float input;
    void Update(float in){
      input = in;
      error = ref-input;
      integral += (error*((float)(dT)/1000));
      output = (Kp*error) + (Ki*integral); 
    };
    
  private:
    float Kp;
    float Ki;
    float Kd;
    float error;
    float output;
    float integral;
    float ref;
    int dT; //Micro Seconds
    int prevTime;
};

struct gains{
  float Kp;
  float Ki;
  float Kd;
};

gains pos = {20,0,0};
gains Speed = {10,0,0}; 



//Define Pin numbers
const char mcPWMA = 12;
const char mcAIN1 = 22;
const char mcAIN2 = 23;
const char mcPWMB = 11;
const char mcBIN1 = 24;
const char mcBIN2 = 25;
const char mcMotStby = 26;
const char PinEncoderA_A = 2;
const char PinEncoderA_B = 3;
const char PinEncoderB_A = 18;
const char PinEncoderB_B = 19;

//Shift register pin numbers
//Pin connected to ST_CP of 74HC595
char latchPin = 8;
//Pin connected to SH_CP of 74HC595
char clockPin = 10;
////Pin connected to DS of 74HC595
char dataPin = 9;


//Encoder Variables
volatile long int countA = 0;
volatile long int countB = 0;
volatile char EncoderA_Aprev = 0;
volatile char EncoderA_A = 0;
volatile char EncoderB_Aprev = 0;
volatile char EncoderB_A = 0;  

//Rotation Variables
float rpmA=0;
float rpmB = 0;

//Position Variables
volatile float theta;
volatile float x;
volatile float y;

int targetPosA=0;
int targetPosB=0;
int posErrorA=0;
int prev_posErrorA=0;
int posErrorB=0;
int prev_posErrorB=0;
float posIntegralA=0;
float posIntegralB=0;

//Speed Variables
volatile float SpeedA = 0;    // current speed of mottor A
volatile float SpeedB = 0;    // current speed of mottor B
int targetSpeedA = 0;       //Speed motor A is to get too
int targetSpeedB = 0;       //Speed motor B is to get too
long int currentTime = 0;
long int prevTime=0;
int loopTime = 0;
long int prevCountA=0;
long int prevCountB=0;

int path[10][2] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};  //Array to store coordinates of a path

//Serial Variables
int incomingByte = 0;
char incomingData[128];
int availableChars = 0;

  int v =0;
  int r =0;

//Setup Function
void setup()  { 
  randomSeed(1);
  Serial.begin(9600);
  analogReference(DEFAULT);
  
  // declare pin 13 to be an output:
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  pinMode(48, OUTPUT);
  pinMode(mcPWMA, OUTPUT);
  pinMode(mcAIN1, OUTPUT);
  pinMode(mcAIN2, OUTPUT);
  pinMode(mcPWMB, OUTPUT);
  pinMode(mcBIN1, OUTPUT);
  pinMode(mcBIN2, OUTPUT);
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
  
  digitalWrite(mcMotStby,LOW); //Make sure motor controller is off
  digitalWrite(mcAIN1,LOW);
  digitalWrite(mcAIN2,LOW);
  digitalWrite(mcBIN1,LOW);
  digitalWrite(mcBIN2,LOW);
  digitalWrite(mcMotStby,HIGH); //Turn on the motor controller
  
  //Setup shift register pins
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  
  //Setup analog pins
  pinMode(A0, INPUT);
  
  
  //targetPosA=5*((Pi*89)/COUNT_MM); //28cm
  targetPosA = 0;
  targetPosB=-targetPosA;
  prevCountA=countA;
  
  tone(48, 1000, 500);
  tone(48, 2000,500);
} 

//Main Loop
void loop()  { 
    loopTime = loopTimer();
        
    CheckSerial();
    BarGraph( (int)(Range()) );
    deadRecon();
  
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
//  }

/*    if(Range()<15){  
      if(random(0,2)==1){
        Clockwise();
      }
      else{
        AntiClockwise();
      }
      delay(random(250,500));
    }
    else{
      setMotor('A',-100);
      setMotor('B',-100);
    }*/
    
    rpmA = ((float)(countA-prevCountA)/((float)(loopTime)/1250));
    rpmB = ((float)(countB-prevCountB)/((float)(loopTime)/1250));
//    if((countA-prevCountA)!=0){
//      Serial.print("Count ");
//      Serial.println(countA-prevCountA,DEC);
//      Serial.print("Time ");
//      Serial.println(((float)((float)(currentTime-prevTime))/1250),8);
//      Serial.print("RPM");
//      Serial.println(rpmA,2);
//    }
    prevCountA=countA;
    
    delay(100);    
  
  /*
  for (int i=0; i==targetSpeed; i++){ 
    Speed = Speed + (i*fadeDir);
    analogWrite(PWMA, Speed);
    delay(500);
  }*/
  
}

//Display value to Bargraph
//Bar graph makes use of shift register
void BarGraph(int numberToDisplay ){
    //Bargraph
    constrain(numberToDisplay, 3, 30);
    numberToDisplay /= 7;    
    if(numberToDisplay>4) numberToDisplay = 4;
    
    switch(numberToDisplay){
      case 0:
        numberToDisplay = 0;
        break;
      case 1:
        numberToDisplay = 128;
        break;
      case 2:
        numberToDisplay = 192;
        break;
      case 3:
        numberToDisplay = 224;
         break; 
      case 4:
        numberToDisplay = 240;
        break;
    }
    //Write number to bar graph
    //Pull shift register latch low to set all outputs low (All LEDs off)   
    digitalWrite(latchPin, LOW);
    // shift out the bits:
    shiftOut(dataPin, clockPin, MSBFIRST, numberToDisplay);  
    //take the latch pin high so the LEDs will light up:
    digitalWrite(latchPin, HIGH);
}

//Time since last function call
int loopTimer(){
    prevTime=currentTime;
    currentTime=micros();
    return (currentTime-prevTime);
}

//Perform dead reconing based on distance traveled since last function call
void deadRecon(){
  static float drCountA;
  static float drCountB;
  long int tempA;
  long int tempB;
  float S;
  tempA = countA;
  tempB = countB;
  drCountA = tempA - drCountA;
  drCountB = tempB - drCountB;
  
  drCountA *= COUNT_MM;
  drCountB *= COUNT_MM;
  
  S = (float)(drCountB/2)+(float)(drCountA/2);
  theta += (float)((float)(drCountB-drCountA)/WHEEL_SPACE);
  x += S*cos(theta);
  y += S*sin(theta);
  
  drCountA = tempA;
  drCountB = tempB;

}

//Perform ranging using IR range finder
//Return: distance in mm?
float Range(){
  float dist = 0;
  
  dist = (2914 / (analogRead(0) + 5)) - 1;
  return dist;
  
}

//PID position control of motor A
void PIDA(){
  posErrorA = targetPosA-countA;
  posIntegralA += (posErrorA*((float)(currentTime-prevTime)/1000));
  targetSpeedA = (pos.Kp*posErrorA) + (pos.Ki*posIntegralA);
  setMotor('A',targetSpeedA);  
}

//PID position control of motor A
void PIDB(){
  posErrorB = targetPosB-countB;
  posIntegralB += (posErrorB*((float)(currentTime-prevTime)/1000));
  targetSpeedB = (pos.Kp*posErrorB) + (pos.Ki*posIntegralB);
  setMotor('B',targetSpeedB);
}

//Function called on each interupt from motor A encoders
//Increments count on CW rotation
//Deccrements count on CCW rotation
void encoderA(){
  //Encoder A pin A attached to arduino pin 2, microprocessor PE4
  //Encoder A pin B attached to arduino pin 2, microprocessor PE5
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

//Function called on each interupt from motor B encoders
void encoderB(){
  //Encoder B pin A attached to arduino pin 2, microprocessor PD3
  //Encoder B pin B attached to arduino pin 2, microprocessor PD2
  EncoderB_Aprev=EncoderB_A;
  EncoderB_A=bitRead(PIND,3);
  if(EncoderB_Aprev^bitRead(PIND,2)){
   countB++;
  }
  else{
   countB--;
  }  
}

//Set speed of selected motor
void setMotorSpeed(char motor, int Speed){
  //Speed currently dirrectly mapped to duty
  //Change to use speed control in future
  
  setMotor(motor, Speed);
  
}

//Set the duty cycle of selected motor
void setMotor(char motor, int Duty){
 
  if(Duty >MAXSPEED){
    Duty=MAXSPEED;
  }
  else if(Duty <MINSPEED) {
    Duty = MINSPEED;
  };  
  
  if(Duty >255){
    Duty=255;
  }
  else if(Duty <-255) {
    Duty = -255;
  };

  if(motor == 'A'){
    if (Duty == 0){
      digitalWrite(mcAIN1,HIGH);
      digitalWrite(mcAIN2,HIGH);  
    }
    else if (Duty>0){
      digitalWrite(mcAIN1,LOW);
      digitalWrite(mcAIN2,HIGH);
    }
    else if (Duty<0){
      digitalWrite(mcAIN1,HIGH);
      digitalWrite(mcAIN2,LOW);  
    };
    analogWrite(mcPWMA, abs(Duty));
  }
  else if(motor == 'B'){
    if (Duty == 0){
      digitalWrite(mcBIN1,HIGH);
      digitalWrite(mcBIN2,HIGH);  
    }
    else if (Duty>0){
      digitalWrite(mcBIN1,HIGH);
      digitalWrite(mcBIN2,LOW);
    }
    else if (Duty<0){
      digitalWrite(mcBIN1,LOW);
      digitalWrite(mcBIN2,HIGH);  
    };
    analogWrite(mcPWMB, abs(Duty));
  };
}

//Actions
void Forward(){
  setMotor('A', 255);
  setMotor('B', 255);  
}

void Stop(){
  setMotor('A', 0);
  setMotor('B', 0);  
}

void Backward(){
  setMotor('A', -255);
  setMotor('B', -255);  
}

void ForwardLeft(){
  setMotor('A', 128);
  setMotor('B', 255);  
}

void ForwardRight(){
  setMotor('A', 255);
  setMotor('B', 128);  
}

void Clockwise(){
  setMotor('A', 255);
  setMotor('B', -255);  
}

void AntiClockwise(){
  setMotor('A', -255);
  setMotor('B', 255);  
}

void Random(){
  setMotor('A', 255);
  setMotor('B', -255);  
}

//Move robot at set velocity and angular veocity 
void Move(int velocity, double angVelocity){
 //(rate robot turns) +ve = anti clockwise as specified by the right-hand-rule
  
  setMotor('A', 0);
  setMotor('B', 0);

  int A = 0;
  int B = 0;
  A = (double)velocity - ((double)(angVelocity*WHEEL_SPACE)/2);
  B = (double)velocity + (double)((angVelocity*WHEEL_SPACE)/2);

  setMotorSpeed('A', A);
  setMotorSpeed('B', B);
    
}

//Check the serial port for data from pc
void CheckSerial(){

  int temp;

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
    if(incomingData[0]=='A'||incomingData[0]=='a'){ //Analog
      Serial.println(analogRead(0));
      Serial.println(Range());
    }
    else if (incomingData[0]=='C'||incomingData[0]=='c'){ //Count, Display curent wheel encoder counts
      Serial.print("Count A = ");
      Serial.println(countA);
      Serial.print("Count B = ");
      Serial.println(countB); 
    }
    else if(incomingData[0]=='M'||incomingData[0]=='m'){  //Motor, Set motor positions
      incomingData[0]=' ';
      if(incomingData[1]=='A'||incomingData[1]=='a'){ //Motor A
        incomingData[1]=' ';
        targetPosA = atoi(incomingData);
        Serial.print("Motor A : ");
        Serial.println(targetPosA);
      }
      else if (incomingData[1]=='B'||incomingData[1]=='b'){ //Motor B
        incomingData[1]=' ';
        targetPosB = atoi(incomingData);
        Serial.print("Motor B : ");
        Serial.println(targetPosB);
      }
      else{  //Motors A&B
        targetPosA = atoi(incomingData);
        targetPosB =targetPosA;
        Serial.print("Motor A & B : ");
        Serial.println(targetPosB);
      };
    }
    else if (incomingData[0]=='V'||incomingData[0]=='v'){ 
        incomingData[0]=' ';
        v = atoi(incomingData);  
        Move(v, (double)r/10);
        Serial.print("Velocity = ");
        Serial.println(v);         
    }
    else if (incomingData[0]=='R'||incomingData[0]=='r'){ 
        incomingData[0]=' ';
        r = atoi(incomingData);  
        Move(v, (double)r/10);
        Serial.print("Rotation = ");
        Serial.println(r);         
    }
    else if (incomingData[0]=='K'||incomingData[0]=='k'){ 
      incomingData[0]=' ';
      if (incomingData[1]=='P'||incomingData[1]=='p'){
        incomingData[1]=' ';
        pos.Kp = atoi(incomingData);
        Serial.print("Kp = ");
        Serial.println(pos.Kp);
      }
      else if (incomingData[1]=='I'||incomingData[1]=='i'){
        incomingData[1]=' ';
        pos.Ki = atoi(incomingData);
        Serial.print("Ki = ");
        Serial.println(pos.Ki);
      }
      else{
        Serial.print("Kp = ");
        Serial.println(pos.Kp);
        Serial.print("Ki = ");
        Serial.println(pos.Ki);
        Serial.print("Kd = ");
        Serial.println(pos.Kd);  
      }  
    }  
    else if (incomingData[0]=='T'||incomingData[0]=='t'){
      Serial.print("Loop Time = ");
      Serial.print(loopTime);
      Serial.println("us");
    }
    else if (incomingData[0]=='P'||incomingData[0]=='p'){
      Serial.print("X = ");
      Serial.println(x);
      Serial.print("Y = ");
      Serial.println(y);
      Serial.print("Theta = ");
      Serial.print(theta);
      Serial.print(" = ");
      Serial.print((theta*180)/3.14); 
    }
    else if (incomingData[0]=='H'||incomingData[0]=='h'){ //Help, List possible commands
      Serial.println("HELP:");
        Serial.println("A");  
        Serial.println("C");  
        Serial.println("M");  
        Serial.println("K");  
        Serial.println("T");
        Serial.println("P"); 
        Serial.println("VR");    
      
    }
    else{  //Quickly set desired motor positions using number
      targetPosA = atoi(incomingData);
      targetPosB =targetPosA;
      Serial.print("Motor A & B : ");
      Serial.println(targetPosB);
    };
    Serial.flush();
  }  
}
