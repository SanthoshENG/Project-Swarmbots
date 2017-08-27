//ROBOT CODE




//Pin Declaration

//5V to power bar on breadboard 
//GND to one rail on breadboard

//Motor ONE

const int phasePinONE = 4;
const int enablePinONE = 5;

//Motor TWO

const int phasePinTWO = 8;
const int enablePinTWO = 9;

//encoder variables

const int encoderPinA1 = 2;// These are the pins that the encoder uses to tell you if the wheel is rotating
const int encoderPinB1 = 3;// and in what direction. (OUTA and OUTB on pololu page diagram)
const int encoderPinA2 = 6;// These are the pins that the encoder uses to tell you if the wheel is rotating
const int encoderPinB2 = 7;// and in what direction. (OUTA and OUTB on pololu page diagram)
const int trigPin = 10;
const int echoPin = 11;
int wheelSpeed = 100;
int refreshRate = 2; // interval at which the RPM refreshes (In seconds)
// These variables are declared volatile because they are used in interrupts, 
// where the variable can be changed at any time, potentially making values in main code innacurate 
volatile int revolutions = 0;
volatile bool rotationDirection; //1=clockwise, 0=counterclockwise
bool beginReading = false; // used in choosing when to count mini wheel pulses
const float PPR = 1819.44; // Pulses Per Revolution, how many rotations of small wheel per big wheel
                           // Gear ratio*12*2, Gear ratio * 12 gives PPR (on polulu site)
                           // multiplied by 2 because both encoder pins were given interrupts
                           // Gear ratio used 75.81 (https://www.pololu.com/product/2209) 


//functions

//Distance sensor functions
float findNearestObjDist(int trigPin,int echoPin);
void sensorsDontLagMotors(float fieldOfView, float linSpeed, float reactTime);
void saveDistanceSensorPower(int distSensorVCC, int echoPin, float nearestObjDist, float linSpeed, float reactTime, float timeOff);
void dontGetTooClose(float nearestObjDist, float linSpeed, float reactTime);

//Motor-Driver functions
void goFwdFull();
void goFwdHalf();
void goBackFull();
void goBackHalf();
void Stop();
void spinCounterClockwise();
void spinClockwise();
void turnSoftRight();
void turnSoftLeft();

//Encoder functions
void rotatingEncoder();
float findLinearSpeed();
float findAngularSpeed();
void rotatingEncoder();



void setup() {

//Pin Setup

//Distance Sensor
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);

  
//Motor 1 Pins

pinMode(phasePinONE, OUTPUT);
digitalWrite(phasePinONE, LOW);
pinMode(enablePinONE, OUTPUT);
digitalWrite(enablePinONE, LOW);

//Motor 2 Pins

pinMode(phasePinTWO, OUTPUT);
pinMode(phasePinTWO, LOW);
pinMode(enablePinTWO,OUTPUT);
digitalWrite(enablePinTWO,LOW);

//Encoder
  
pinMode(encoderPinA1,INPUT_PULLUP); //pullup resistors built into arduino used
pinMode(encoderPinB1,INPUT_PULLUP); // pullup resistors ensure High and Low values are accurate
pinMode(encoderPinA2,INPUT_PULLUP); //pullup resistors built into arduino used
pinMode(encoderPinB2,INPUT_PULLUP); // pullup resistors ensure High and Low values are accurate
  
attachInterrupt(digitalPinToInterrupt(2),rotatingEncoder, RISING);
attachInterrupt(digitalPinToInterrupt(3),rotatingEncoder, RISING);

Serial.begin(9600); 
}




void loop() {
        
            }

  float findNearestObjDist(int trigPin,int echoPin){
    //time a pulse, find duration of high pulse, find distance in cm using duration and distance sensor equation
    //CHANGE: need to change digWrite to digRead, that way we read when the pin is high, then time it and =duration
    const double SoundSpeed = 0.034029; //340m/s = 340mm/ms = 34 cm/ms = 0.034cm/Microsec
    long duration;
    int distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2); 
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    //recorded in microseconds
    duration = pulseIn(echoPin, HIGH);
    // distance in centimeters
    distance = (duration*SoundSpeed)/2;
  /* BREAKING BOUNDARIES
      if (distance >= 200 )||
    {
      Serial.println("Reading too far from sensor");
    }
    else if (distance <= 0)
    {
      Serial.println("Reading too close to sensor");
    }
    else {
      Serial.print(distance);
      Serial.println(" cm");
    }
    
  }
   */
   
  } //findNearestObjDist
  
  void sensorsDontLagMotors(float fieldOfView, float linSpeed, float reactTime){
    float minFieldOfView = linSpeed*reactTime; //calculate the minimum FOV the robot would need, based on its linear speed
    if(fieldOfView >= minFieldOfView){
    }
    else if (fieldOfView < minFieldOfView){
      float timeOn = minFieldOfView/linSpeed;
      float timeOff = reactTime-timeOn;
      //CHANGE: still need the command to turn the motors off for timeOff
    } //if real FOV not large enough, stop robot for the distance it can't see
  }
 void saveDistanceSensorPower(int distSensorVCC, int echoPin, float nearestObjDist, float linSpeed, float reactTime, float timeOff){
    
    if (nearestObjDist > linSpeed*(reactTime+timeOff)){
      //S.K.: Cut power to distance sensor, NOTE: requires distance sensor VCC pin to be a digital pin
      //M.H.: Why is that so?
      digitalWrite(distSensorVCC,LOW);
      delay(timeOff/1000);// THIS IS IF timeOff IS GIVEN IN MILLISECONDS
    }
  }
 void dontGetTooClose(float nearestObjDist, float linSpeed, float reactTime){
    
    // the order of the two requirements were swapped so that
    // the emergency procedure would happen before the caution procedure
    if(nearestObjDist < 1.2*linSpeed*reactTime){
    //CHANGE: ADD COMMAND TO GO BACKWARDS FOR 0.8*linSpeed*reactTime THEN STOP
    }
  
    if(nearestObjDist < 2*linSpeed*reactTime){
    //CHANGE: ADD COMMAND TO STOP MOVING
    }//if
  }// dontGetTooClose
//loop




//motor driver


//MOTOR AND DRIVER CODE:
//ONE: LEFT WHEEL
//TWO: RIGHT WHEEL
//BAUD: 9600
//Encoder ONE
//const int vccPinONE = 3;
//const int phasePinONE = 4;
//const int enablePinONE =5;
//Encoder TWO
//const int vccPinTWO = 6;
//const int phasePinTWO = 7;
//const int enablePinTWO = 8;
//int wheelSpeed = 100;
/*void setup() {
Serial.begin(9600);
//Encoder 1 Pins
//pinMode(vccPinONE, OUTPUT);
//digitalWrite(vccPinONE,HIGH);
pinMode(phasePinONE, OUTPUT);
digitalWrite(phasePinONE, LOW);
pinMode(enablePinONE, OUTPUT);
digitalWrite(enablePinONE, LOW);
//Encoder 2 Pins
//pinMode(vccPinTWO, OUTPUT);
//digitalWrite(vccPinTWO,HIGH);
pinMode(phasePinTWO, OUTPUT);
pinMode(phasePinTWO, LOW);
pinMode(enablePinTWO,OUTPUT);
digitalWrite(enablePinTWO,LOW);
   
}*/
   
//directions based on truth table on driver page:
// https://www.pololu.com/product/2990
void goFwdFull()
{ 
    digitalWrite(phasePinONE,LOW);
    digitalWrite(phasePinTWO,LOW);
    analogWrite(enablePinONE,255);
    analogWrite(enablePinTWO,255);
} 
 void goFwdHalf() 
 { 
    digitalWrite(phasePinONE,LOW);
    digitalWrite(phasePinTWO,LOW);
    analogWrite(enablePinONE,123);
    analogWrite(enablePinTWO,123);
 }
 void goBackFull() 
 { 
    digitalWrite(phasePinONE,HIGH);
    digitalWrite(phasePinTWO,HIGH);
    analogWrite(enablePinONE,255);
    analogWrite(enablePinTWO,255);
 }
 void goBackHalf() // two wheels at half instead of one wheel at full
 { 
    digitalWrite(phasePinONE,HIGH);
    digitalWrite(phasePinTWO,HIGH);
    analogWrite(enablePinONE,123);
    analogWrite(enablePinTWO,123);
 } 
  void Stop() 
  {
    digitalWrite(phasePinONE,LOW);
    digitalWrite(phasePinTWO,LOW);
    analogWrite(enablePinONE,0);
    analogWrite(enablePinTWO,0);
  } 
  void spinCounterClockwise() 
  {
    digitalWrite(phasePinONE,LOW);
    digitalWrite(phasePinTWO,HIGH);
    analogWrite(enablePinONE,255);
    analogWrite(enablePinTWO,255);
  } 
  void spinClockwise() 
  {
    digitalWrite(phasePinONE,HIGH);
    digitalWrite(phasePinTWO,LOW);
    analogWrite(enablePinONE,255);
    analogWrite(enablePinTWO,255);
  }
   void turnSoftRight() {
    
      digitalWrite(phasePinONE,LOW);
    digitalWrite(phasePinTWO,LOW);
    analogWrite(enablePinONE,255);
    analogWrite(enablePinTWO,200);
    
    }
 void turnSoftLeft() 
 { 
    digitalWrite(phasePinONE,LOW);
    digitalWrite(phasePinTWO,LOW);
    analogWrite(enablePinONE,200);
    analogWrite(enablePinTWO,255);
 }



  //encoder


  
/*int refreshRate = 2; // interval at which the RPM refreshes (In seconds)
// These variables are declared volatile because they are used in interrupts, 
// where the variable can be changed at any time, potentially making values in main code innacurate 
volatile int revolutions = 0;
volatile bool rotationDirection; //1=clockwise, 0=counterclockwise
bool beginReading = false; // used in choosing when to count mini wheel pulses
const int encoderPinA = 2;// These are the pins that the encoder uses to tell you if the wheel is rotating
const int encoderPinB = 3;// and in what direction. (OUTA and OUTB on pololu page diagram)
//const int vccPin = 10; //used to power the encoder logic
const float PPR = 1819.44; // Pulses Per Revolution, how many rotations of small wheel per big wheel
                           // Gear ratio*12*2, Gear ratio * 12 gives PPR (on polulu site)
                           // multiplied by 2 because both encoder pins were given interrupts
                           // Gear ratio used 75.81 (https://www.pololu.com/product/2209)*/
                           
/*void setup() {
  
  Serial.begin(9600);
  pinMode(encoderPinA,INPUT_PULLUP); //pullup resistors built into arduino used
  pinMode(encoderPinB,INPUT_PULLUP); // pullup resistors ensure High and Low values are accurate
  
  //pinMode(vccPin,OUTPUT);     
  //digitalWrite(vccPin,HIGH);
  attachInterrupt(digitalPinToInterrupt(2),rotatingEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(3),rotatingEncoder, RISING);
  
}*/
/*void loop() {
  
//Example of using encoder
//Serial.println(findLinearSpeed(findRPM(),4));
//Serial.println(rotationDirection);
//delayMicroseconds(500);
}*/
// this interrupt function is what counts how many times the mini wheel "pulses"
// it also gives the direction of rotation at the same time
// the direction of rotation is a global variable (rotationDirection) which is changed by the interrupt
void rotatingEncoder(){
  
  //wont count revolutions unless asked
  if(beginReading == true){
    revolutions++;
  }
    // the encoder pins are phase shifted by 90 degrees
    // if pinA leads, it is rotating clockwise
    // if pinB leads, it is rotating counterclockwise
    // this works because the interrupts are triggered only by RISING (low to high)logic
    
if (digitalRead(encoderPinA1) == HIGH && digitalRead(encoderPinB1) == LOW){
      rotationDirection = true; // clockwise
    }
    else if(digitalRead(encoderPinA1) == LOW && digitalRead(encoderPinB1) == HIGH){
      rotationDirection = false; // counterclockwise
    }
if (digitalRead(encoderPinA2) == HIGH && digitalRead(encoderPinB2) == LOW){
      rotationDirection = true; // clockwise
    }
    else if(digitalRead(encoderPinA2) == LOW && digitalRead(encoderPinB2) == HIGH){
      rotationDirection = false; // counterclockwise
    }
}
  float findRPM(){
    
  // as soon as you call findRPM(), revolutions start being counted
  beginReading = true;
  // millis() is the amount of time since the program began running (in milliseconds)
  
  float timeOld = millis();
  float rpm = 0.0;
  // the encoder is recording revolutions for as long as "refreshRate" amount of seconds
  while((millis()-timeOld)/1000 <= refreshRate){
   
  }
  
  // now stops recording revolutions
  beginReading = false;
  // rpm = (Pulses recorded/given interval)*(1 Big wheel rotation / Pulses per big wheel rotation, PPR)*(60seconds/1minute)
  rpm = (revolutions*60.0)/PPR;
  
  revolutions = 0;
  return rpm;
 
  
}
float findLinearSpeed(int RPM, int WheelRadius){
  
// formula given by Matt on google doc
float linearSpeed = RPM*WheelRadius*(3.141593/30); // in cm
return linearSpeed;
  
}
float findAngularSpeed(float linearSpeed, float radius, bool isClockwise)
{
  float angularSpeed = linearSpeed/radius;
  return angularSpeed;
}


void IR_sensor() {
  float volts = analogRead(A1)*0.0048828125;   
  float distance = 65*pow(volts, -1.10);
  float nearestdistance=distance-45;
  if (distance<67)
  {
    Serial.println("There is an obstacle");
    
    
  }
  else
  {
    Serial.println("Path is Clear");
  }
  Serial.println(nearestdistance);                      
  delay(100);                                     
}
