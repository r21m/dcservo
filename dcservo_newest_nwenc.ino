/* This one is not using any PinChangeInterrupt library */
/*
   This program uses an Arduino for a closed-loop control of a DC-motor. 
   Motor motion is detected by a quadrature encoder.
   Two inputs named STEP and DIR allow changing the target position.
   Serial port prints current position and target position every second.
   Serial input can be used to feed a new location for the servo (no CR LF).
   
   Pins used:
   Digital inputs 2 & 8 are connected to the two encoder signals (AB).
   Digital input 3 is the STEP input.
   Analog input 0 is the DIR input.
   Digital outputs 9 & 10 control the PWM outputs for the motor (I am using half L298 here).


   Please note PID gains kp, ki, kd need to be tuned to each different setup. 
*/

#include <EEPROM.h>
#include <PID_v1.h>
#include <AccelStepper.h>
#include <Encoder.h>
#define encoder0PinA  2   // PD2; 
#define encoder0PinB  8   // PC0;
#define stepInput     3   // 
//
#define STEP_OUT      5    // Stepper
#define DIR_OUT       6    // Stepper
#define MS1           9    // Stepper
#define MS2           10   // Stepper
#define MS3           15   // Stepper
#define  MAX_SPEED 32767
#define  MIN_SPEED 1

Encoder myEnc(encoder0PinA, encoder0PinB);
AccelStepper axis(1, STEP_OUT, DIR_OUT);

byte pos[1000]; int p=0; 
double kp=3,ki=0,kd=0.0;
double input=0, output=0,setpoint=0;
PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
volatile long encoder0Pos = 0;
boolean auto1=false, auto2=false,counting=false;
long previousMillis = 0;        // will store last time LED was updated
long target1=0;  // destination location at any moment
//for motor control ramps 1.4
bool newStep = false;
bool oldStep = false;
bool dir = false;
byte skip=0;
//---------------------
volatile long Old;
volatile long New;
int lag;
float linear_pos;
float linear_pos_err;
float stepper_enc = 0.000833;   //2400/360 
//--------------------
int d;
//--------------------
// Install Pin change interrupt for a pin, can be called multiple times
/*void pciSetup(byte pin) 
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group 
}
*/
void change_res(int res)
  {
    if (res == 1){digitalWrite(MS1, LOW); digitalWrite(MS2, LOW); digitalWrite(MS3,  LOW);}
    if (res == 2){digitalWrite(MS1, HIGH);digitalWrite(MS2, LOW); digitalWrite(MS3,  LOW);}
    if (res == 3){digitalWrite(MS1, LOW); digitalWrite(MS2, HIGH);digitalWrite(MS3,  LOW);}
    if (res == 4){digitalWrite(MS1, HIGH);digitalWrite(MS2, HIGH);digitalWrite(MS3,  LOW);} 
    if (res == 5){digitalWrite(MS1, HIGH);digitalWrite(MS2, HIGH);digitalWrite(MS3, HIGH);}
  }

void setup() { 
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT);  
//  pciSetup(encoder0PinB);
  attachInterrupt(digitalPinToInterrupt (encoder0PinA), encoderInt, CHANGE);
  attachInterrupt(digitalPinToInterrupt (stepInput),     countStep, RISING);
//  TCCR1B = TCCR1B & 0b11111000 | 1; // set 31Kh PWM
  Serial.begin (115200);
  help();
  recoverPIDfromEEPROM();
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-32767,32767);
  axis.setMaxSpeed(MAX_SPEED);
  axis.setAcceleration(1);
  //axis.setMinPulseWidth(3);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  
} 

void loop(){
    change_res(5);
    lag = setpoint - encoder0Pos;
    input = encoder0Pos; 
    setpoint=target1;
    myPID.Compute();
    if(Serial.available()) process_line(); // it may induce a glitch to move motion, so use it sparingly  
    step_dir(output);
    if(auto1) if(millis() % 3000 == 0) target1=random(2000); // that was for self test with no input from main controller
    if(auto2) if(millis() % 1000 == 0) printPos();
    //if(counting && abs(input-target1)<15) counting=false; 
    //if(counting &&  (skip++ % 5)==0 ) {pos[p]=encoder0Pos; if(p<999) p++; else counting=false;}
    linear_pos = setpoint * stepper_enc;
    linear_pos_err = lag * stepper_enc;
}

void step_dir(int out) 
  {
      axis.runSpeed();
      axis.setSpeed(out);         
  }

void shaft(int y)
{ 
     if (abs( y) < 1000) {change_res(5);}
     if (abs( y) > 2000) {change_res(4);} 
     if (abs( y) > 3500) {change_res(3);}
     if (abs( y) > 4500) {change_res(2);}
     if (abs( y) > 5000) {change_res(1);}         
}

void encoderInt() {
    encoder0Pos = myEnc.read();
}

void countStep(){
  if (PINC&B0000001) target1--;
  else target1++;
//  d = (target1 - setpoint);
}
 
void process_line() {
 char cmd = Serial.read();
 if(cmd>'Z') cmd-=32;
 switch(cmd) {
  case 'P': kp=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'D': kd=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'I': ki=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case '?': printPos(); break;
  case 'X': target1=Serial.parseInt(); p=0; counting=true; for(int i=0; i<300; i++) pos[i]=0; break;
  
  case 'L': kp=Serial.parseFloat(); //myParam.SetParameter(kp,ki,kd); break; //LAG
  case 'M': kp=Serial.parseFloat(); //myParam.SetParameter(kp,ki,kd); break; //Microstepping
  case 'C': kp=Serial.parseFloat(); //myParam.SetParameter(kp,ki,kd); break; //Calc constant
  
  case 'T': auto1 = !auto1; break;
  case 'A': auto2 = !auto2; break;
  case 'Q': Serial.print("P="); Serial.print(kp); Serial.print(" I="); Serial.print(ki); Serial.print(" D="); Serial.println(kd); break;
  case 'H': help(); break;
  case 'W': writetoEEPROM(); break;
  case 'K': eedump(); break;
  case 'R': recoverPIDfromEEPROM() ; break;
  case 'S': for(int i=0; i<p; i++) Serial.println(pos[i]); break;
 }
 while(Serial.read()!=10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}

void printPos() 
{
  Serial.print(F("Position="));Serial.print(encoder0Pos);
  Serial.print(F(" PID_output="));Serial.print(output);
  Serial.print(F(" Target="));Serial.println(setpoint);
  Serial.print(F(" Lag="));Serial.println(lag);
  Serial.print(F(" Stepper="));Serial.println(output);
  Serial.print(F(" Position="));Serial.println(linear_pos);
  Serial.print(F(" Position error="));Serial.println(linear_pos_err);

}
void help() {
 Serial.println(F("\nPID stepper controller and interface"));
 Serial.println(F("by misan, stepper by R21m"));
 Serial.println(F("Available serial commands: (lines end with CRLF or LF)"));
 Serial.println(F("P123.34 sets proportional term to 123.34"));
 Serial.println(F("I123.34 sets integral term to 123.34"));
 Serial.println(F("D123.34 sets derivative term to 123.34"));
 Serial.println(F("X123 sets the target destination for the motor to 123 encoder pulses"));
 Serial.println(F("L123 sets maximum lag"));     //L
 Serial.println(F("M1-5 sets microstepping"));   //M
 Serial.println(F("C0.0338 sets calc conctant"));//C
 Serial.println(F("? prints out current encoder, output and setpoint values"));
 Serial.println(F("T will start a sequence of random destinations (between 0 and 2000) every 3 seconds. T again will disable that"));
 Serial.println(F("Q will print out the current values of P, I , D and optionaly parameters")); 
 Serial.println(F("W will store current values of P, I and D parameters into EEPROM")); 
 Serial.println(F("H will print this help message again")); 
 Serial.println(F("A will toggle on/off showing regulator status every second\n")); 
}

void writetoEEPROM() { // keep PID set values in EEPROM so they are kept when arduino goes off
  eeput(kp,0);
  eeput(ki,4);
  eeput(kd,8);
  double cks=0;
  for(int i=0; i<12; i++) cks+=EEPROM.read(i);
  eeput(cks,12);
  
  Serial.println("\nPID values stored to EEPROM");
  Serial.println(cks);
}

void recoverPIDfromEEPROM() {
  double cks=0;
  double cksEE;
  for(int i=0; i<12; i++) cks+=EEPROM.read(i);
  cksEE=eeget(12);
  //Serial.println(cks);
  if(cks==cksEE) {
    Serial.println(F("*** Found PID values on EEPROM"));
    kp=eeget(0);
    ki=eeget(4);
    kd=eeget(8);
    myPID.SetTunings(kp,ki,kd); 
  }
  else Serial.println(F("*** Bad checksum"));
}


void eeput(double value, int dir) 
{ 
  char * addr = (char * ) &value;
  for(int i=dir; i<dir+4; i++)  EEPROM.write(i,addr[i-dir]);
}

double eeget(int dir)
  {
  double value;
  char * addr = (char * ) &value;
  for(int i=dir; i<dir+4; i++) addr[i-dir]=EEPROM.read(i);
  return value;
}

void eedump() {
 for(int i=0; i<16; i++) 
 { 
  Serial.print(EEPROM.read(i),HEX); 
  Serial.print(" "); }Serial.println(); 
}
