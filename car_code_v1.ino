/************************************************************************************************************************************************************************                                              
 * - Team             : McGill Chem-E Car            *                                               
 * - Project          : Nationals 2023               *                                                                                              
 * - Main purpose     : Full car tests               *                                                                                  
                  
 * - Date             : 30/03/2023                   *
 * ***********************************************************************************************************************************************************************/

#include <Stepper.h>
#include "DFRobot_EC.h"
#include <EEPROM.h>

//Sensor initialisation
#define EC_PIN A1
float voltage,ecValue,temperature = 25;
DFRobot_EC ec;
float data;

//General var initialisation
const int timeDelay=333; //time interval for sensor reading (in millisec)
float startTime=0;
float time=millis();
int stoppingFlag=0;
int loopIterations = 0;

//stepper connections
const int s0=8;
const int s1=9;
const int s2=10;
const int s3=11;

//Injection var and setup
const int stepsPerRevolution = 200;
const float RevsForInjection = 10.0; //How many steps to fully inject
const int injectionSpeed = 60; //motor rpm
// initialize the Stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, s0, s1, s2, s3);

//Main motor
const int transistor = 12;

//Mathematical analysis + sensor
const int numReadings = 120*1000/timeDelay; //number of readings in 2 minutes
//float readings [numReadings+5]; //storage for readings + some safety margin
float readings[360]; //Use the line above normally but its bugged for some reason
const int considered = 10; //#of points used in the moving avg.
const float stdRef = 0.032;
const float lowBound = 2;
float moving[2];

void setup() {
  // set the stepper at given speed
  myStepper.setSpeed(injectionSpeed);
  // initialize the serial port:
  Serial.begin(9600);
  //pin configuration
  pinMode(transistor, OUTPUT);

  //sensor initialisation
  ec.begin();

  //starts the reaction
  startInjecting();
  startTime= millis();
  startEngine();
}

void loop() {
  time = millis();
  if (stoppingFlag) {return;} //The experiment has stopped
  //checkDelay(time); //debugging purposes
  
  while (time<timeDelay*loopIterations+startTime){}
  //Waits until it's time to read again (preserves memory)
  readings[loopIterations] = readProbe();
  
  if (loopIterations<considered){return;}//not enough points to perform avg

  movingAverage(); //updates moving mean and std
  featureDetection(); //checks if the updated mean and std indicate to stop
  
  loopIterations +=1;
  if (loopIterations > numReadings){
    finalAbort();
    stoppingFlag=1;
  }
}

void startInjecting() {
  myStepper.step(stepsPerRevolution);
  delay(5000); //waits out the injection time
}

void startEngine() {
  digitalWrite(transistor, HIGH);
}

void stopEngine() {
  digitalWrite(transistor, LOW);
}

float readProbe() {
  voltage = analogRead(EC_PIN)/1024.0*5000;   // read the voltage
  ecValue =  ec.readEC(voltage,temperature);  // convert voltage to EC with temperature compensation
  return ecValue;
}

void movingAverage(){
  float mean=0;
  float std=0;
  for(int i=0; i<considered; i++) {
    mean+=readings[loopIterations-i];
  }
  mean = mean/considered;
  for(int i=0; i<considered; i++) {
    std += pow(readings[loopIterations-i] - mean, 2);
  }
  std = sqrt(std/considered);
  moving[0] = mean;
  moving[1] = std;
}

void featureDetection(){
  if (moving[0]<stdRef and moving[1]>lowBound) { //We got a stop condition!
    stopEngine();
    stoppingFlag=1;
  }
}

void checkDelay(float time) {
  if (time>timeDelay*loopIterations+timeDelay+startTime) {
    Serial.println("Program too slow");
    Serial.print("Current time: ");
    Serial.print(time);
    Serial.print(", Expected time: ");
    Serial.println(timeDelay*loopIterations+timeDelay);
  }
}

void finalAbort() {
  //If this function is called then the 2 minutes elapsed without
  //stopping the car properly, this means the feature detections
  //was not performed correctly, try tweaking the following:
  //lowBound, stdRef, considered
  Serial.println("Allowed time elapsed, forcing the car to stop");
  Serial.println("Check feature detection function");
  Serial.println("You did not fail, you just found one more way that does not work");
  stopEngine();
}
