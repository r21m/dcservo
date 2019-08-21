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
#include <Wire.h>

//IO
#define encoder0PinA  2   // PD2; 
#define encoder0PinB  8   // PC0;
//
#define stepInput     3   //
#define dirInput      A0  //A0 
//
#define STEP_OUT      5   // Stepper
#define DIR_OUT       6   // Stepper
//
#define ena_in        A1
#define ena_out       9
#define error_out     7

//#define MS1           9    // Stepper
//#define MS2           10   // Stepper
//#define MS3           15   // Stepper

#define  MAX_SPEED 5000
//#define  MIN_SPEED 1000
#define  limit 6000

Encoder myEnc(encoder0PinA, encoder0PinB);
AccelStepper axis(1, STEP_OUT, DIR_OUT);

byte pos[1000]; int p=0; 
double kp=3,ki=0,kd=0.0;
byte addr; //Adress fot IIC 0-255
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
long lag;
float linear_pos;
float linear_pos_err;
float stepper_enc = 0.000833;   //2400/360 
//--------------------
bool dirIn = 0;
bool enaIn = 0;
//--------------------
/*void pciSetup(byte pin) 
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group 
}
*/
/*
void change_res(int res)
  {
    if (res == 1){digitalWrite(MS1, LOW); digitalWrite(MS2, LOW); digitalWrite(MS3,  LOW);}
    if (res == 2){digitalWrite(MS1, HIGH);digitalWrite(MS2, LOW); digitalWrite(MS3,  LOW);}
    if (res == 3){digitalWrite(MS1, LOW); digitalWrite(MS2, HIGH);digitalWrite(MS3,  LOW);}
    if (res == 4){digitalWrite(MS1, HIGH);digitalWrite(MS2, HIGH);digitalWrite(MS3,  LOW);} 
    if (res == 5){digitalWrite(MS1, HIGH);digitalWrite(MS2, HIGH);digitalWrite(MS3, HIGH);}
  }
*/
void setup() { 
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT);  
//  pciSetup(encoder0PinB);
  attachInterrupt(digitalPinToInterrupt (encoder0PinA), encoderInt, CHANGE);
  attachInterrupt(digitalPinToInterrupt (encoder0PinB), encoderInt, CHANGE);
  attachInterrupt(digitalPinToInterrupt (stepInput),     countStep, RISING);
  attachInterrupt(digitalPinToInterrupt (dirInput),      countStep, CHANGE);
  //TCCR1B = TCCR1B & 0b11111000 | 1; // set 31Kh PWM
  Serial.begin (115200);
  help();
  recoverPIDfromEEPROM();
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(8000 * (-1),8000);
  axis.setMaxSpeed(8000);
  axis.setAcceleration(1);
  //axis.setMinPulseWidth(0);
  // pinMode(MS1, OUTPUT);
  // pinMode(MS2, OUTPUT);
  // pinMode(MS3, OUTPUT);
  pinMode(stepInput,  INPUT);
  pinMode(dirInput,   INPUT);
  pinMode(ena_in,     INPUT);
  pinMode(ena_out,   OUTPUT);
  pinMode(error_out, OUTPUT);

  
} 

void loop(){
  //  change_res(5);
    lag   = setpoint - encoder0Pos;
    input = encoder0Pos; 
    setpoint=target1;
    myPID.Compute();
    if(Serial.available()) process_line(); // it may induce a glitch to move motion, so use it sparingly  
    step_dir(output);
    if(auto1) if(millis() % 3000 == 0) target1=random(2000); // that was for self test with no input from main controller
    if(auto2) if(millis() % 1000 == 0) printPos();
    //if(counting && abs(input-target1)<15) counting=false; 
    if(counting &&  (skip++ % 5)==0 ) {pos[p]=encoder0Pos; if(p<999) p++; else counting=false;}
    linear_pos = setpoint * stepper_enc;
    linear_pos_err = lag * stepper_enc;
    
}

void step_dir(int out) 
  {
      axis.runSpeed();
      enable();
      axis.setSpeed(out );
      error();         
  }
void encoderInt() {
       encoder0Pos = myEnc.read();
}

void countStep()
{
  dirIn = digitalRead(dirInput);
  if (dirIn == LOW)
  {
     target1++;
  } 
  else
  {
     target1--;
  }
  // stepInput & dirInput ADC7 B0000001 PINC&B0000001
}

void error()
{
  if (abs(lag) > 600) 
  {
    digitalWrite(error_out,HIGH);
  }
  else
  {
    digitalWrite(error_out,LOW);
  }
}

void enable() 
   {
     enaIn = digitalRead(ena_in);  
     if (enaIn == LOW)
         {
     digitalWrite(ena_out, LOW);
         }
     else
         {
     digitalWrite(ena_out, HIGH);
    // target1 = 0;
     output  = 0;
     }
  }
 
void process_line() {
 char cmd = Serial.read();
 if(cmd>'Z') cmd-=32;
 switch(cmd) {
  case 'P': kp=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'D': kd=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'I': ki=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case '?': printPos(); break;
  case 'X': target1=Serial.parseInt(); p=0; counting=true; for(int i=0; i<0; i++) pos[i]=0; break;
  case 'Y': addr=Serial.parseFloat();
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
  Serial.print(F(" Position="));Serial.print(encoder0Pos);
  Serial.print(F(" PID_output="));Serial.print(output);
  Serial.print(F(" Target="));Serial.println(setpoint);
  Serial.print(F(" Lag="));Serial.println(lag);
  Serial.print(F(" Stepper="));Serial.println(output);
  Serial.print(F(" Position="));Serial.println(linear_pos);
  Serial.print(F(" Position error="));Serial.println(linear_pos_err);
  Serial.print(F(" Direction="));Serial.println(dirIn);
  Serial.print(F(" Enable="));Serial.println(enaIn);
  Serial.print(F(" Adress="));Serial.println(addr);
  
}
void help() {
 Serial.println(F("\nPID stepper controller and interface"));
 Serial.println(F("by misan, stepper by R21m"));
 Serial.println(F("Available serial commands: (lines end with CRLF or LF)"));
 Serial.println(F("P123.34 sets proportional term to 123.34"));
 Serial.println(F("I123.34 sets integral term to 123.34"));
 Serial.println(F("D123.34 sets derivative term to 123.34"));
 Serial.println(F("X123 sets the target destination for the motor to 123 encoder pulses"));
 Serial.println(F("Y10 sets the module I2C adress to 0x10"));
 Serial.println(F("? prints out current encoder, output and setpoint values"));
 Serial.println(F("T will start a sequence of random destinations (between 0 and 2000) every 3 seconds. T again will disable that"));
 Serial.println(F("Q will print out the current values of P, I , D and optionaly parameters")); 
 Serial.println(F("W will store current values of P, I and D parameters into EEPROM")); 
 Serial.println(F("H will print this help message again")); 
 Serial.println(F("A will toggle on/off showing regulator status every second\n")); 
}

void writetoEEPROM() { // keep PID set values in EEPROM so they are kept when arduino goes off and adress
  eeput(kp,0);
  eeput(ki,4);
  eeput(kd,8);
  eeput(addr,12);
  double cks=0;
  for(int i=0; i<16; i++) cks+=EEPROM.read(i);
  eeput(cks,16);
  
  Serial.println("\nPID values stored to EEPROM");
  Serial.println(cks);
}

void recoverPIDfromEEPROM() {
  double cks=0;   //
  double cksEE;   //checksum
  for(int i=0; i<16; i++) cks+=EEPROM.read(i);
  cksEE=eeget(16);
  //Serial.println(cks);
  if(cks==cksEE) {
    Serial.println(F("*** Found PID values on EEPROM"));
    kp=eeget(0);
    ki=eeget(4);
    kd=eeget(8);
    myPID.SetTunings(kp,ki,kd);
    Serial.println(F("IIC adress"));
    addr = eeget(12); 
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
 for(int i=0; i<20; i++) 
 { 
  Serial.print(EEPROM.read(i),HEX); 
  Serial.print(" "); }Serial.println(); 
}
