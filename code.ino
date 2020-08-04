/////////Ball and Plate///////////////////////////////
/*
BALL AND PLATE PID CONTROL
*/
//////////////////////////////////////////////////////
///Libraries///
#include <PID_v1.h>
#include <stdint.h>
#include "TouchScreen.h"
#include <SPI.h>
#include <Wire.h>
#include <wiinunchuk.h>
#include<Servo.h>
// Definitions TOUCH PINS
#define YP A0 //0
#define XM A1 //1
#define YM 3  //3
#define XP 4 //4
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
int buttonPushCounter = 1;   // counter for the number of button presses
int lastButtonState = 0;     // previous state of the button
int flag , flagZ ;


float xVal , yVal ;
int cCount=0;
int flagC=0;
int flagK=0;
float kk=0;
int fl=0;
double l =0.00;
unsigned int noTouchCount = 0; //viariable for noTouch
double  k=0;
// PID values
double Setpoint, Input, Output; //for X
double Setpoint1, Input1, Output1; //for Y
//
int Modulo;
long lastcas=0;
// servos variables
Servo servo1; //X axis
Servo servo2; //Y axis

uint16_t homeX = 550;            // raw data value for center of touchscreen
uint16_t homeY = 550;            // raw data value for center of touchscreen             

float convertX = 151.0 / 955.0;  // converts raw x values to mm. found through manual calibration
float convertY = 91.0 / 927.0;   // converts raw y values to mm. found through manual calibration
/////TIME SAMPLE
int Ts = 50; 
unsigned long Stable=0; 
//PID const
float Kp = 0.3;                                                     
float Ki = 0.03;                                                      
float Kd = 0.13;

float Kp1 = 0.3;                                                       
float Ki1 = 0.08;                                                      
float Kd1 = 0.13;
long cas=0; 
//INIT PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID1(&Input1, &Output1, &Setpoint1,Kp1,Ki1,Kd1, DIRECT);

void setup()
{
  servo1.attach(5);
  servo2.attach(6);
  Output=95;
  Output1=95;
  servo1.write(Output);
  servo2.write(Output1);
  
  //init NUN
  nunchuk_setpowerpins();
  nunchuk_init();
  nunchuk_get_data(); 
 
  //INIT PINS
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(9,LOW); //LED INIT
  digitalWrite(8,LOW);

  Serial.begin(115200);
  
  //INIT OF TOUSCHSCREEN
   TSPoint p = ts.getPoint();
 Input=120;
 Input1=65;
  //INIT SETPOINT
  Setpoint=120;
  Setpoint1=65;
  //// Make plate flat
  servo1.attach(5); 
  servo2.attach(6);
  Output=95;
  Output1=95;
  servo1.write(Output);
  servo2.write(Output1);
  
  //Zapnutie PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(20, 160);
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(20, 160);
  // TIME SAMPLE
  myPID1.SetSampleTime(Ts); 
  myPID.SetSampleTime(Ts);  
  /////
  delay(100);
  
 
  ///
 }
 
void loop()
{
  while(Stable<125) //REGULATION LOOP
  {
   TSPoint p = ts.getPoint();   //measure pressure on plate
   if (p.z > ts.pressureThreshhold) //ball is on plate
   {  
      servo1.attach(5); //connect servos
      servo2.attach(6); 
      setDesiredPosition();  
      noTouchCount = 0;  
      TSPoint p = ts.getPoint(); // measure actual position 
      Input=(p.x * convertX);  // read and convert X coordinate
      Input1=(p.y * convertY); // read and convert Y coordinate
      
          if((Input>Setpoint-2 && Input<Setpoint+2 && Input1>Setpoint1-2 && Input1<Setpoint1+2))//if ball is close to setpoint
          {
              Stable=Stable+1; //increment STABLE
              digitalWrite(9,HIGH);
                 
          }
          else
          {
              digitalWrite(9,LOW);
          }
       myPID.Compute();  //action control X compute
       myPID1.Compute(); //   action control  Y compute   
    
  }
   else //if there is no ball on plate
  {
    noTouchCount++; //increment no touch count

    if(noTouchCount == 75) 
    {
     noTouchCount++; 
     Output=95; //make plate flat
     Output=93;
     servo1.write(Output); 
     servo2.write(Output1);
    }
    if(noTouchCount == 150) //if there is no ball on plate longer
    {
     servo1.detach(); //detach servos
     servo2.detach();     
   
    }
  }
  servo1.write(Output);//control
  servo2.write(Output1);//control 
  Serial.print(Setpoint);   Serial.print(",");  Serial.print(Setpoint1);  Serial.print(",");  Serial.print(Input);Serial.print(","); Serial.println(Input1); 
     
}////END OF REGULATION LOOP///
  
  servo1.detach();//detach servos
  servo2.detach();
  
 ///KONTROLA STABILITY////
 while(Stable==125)//if is stable
 { //still measure actual postiion
    setDesiredPosition(); 
    TSPoint p = ts.getPoint();
     Input=(p.x * convertX);  //read X
      Input1=(p.y * convertY); //read Y
    if(Input<Setpoint-2 || Input>Setpoint+2 || Input1>Setpoint1+2 || Input1<Setpoint1-2  ) //if ball isnt close to setpoint
    {
      servo1.attach(5); //again attach servos
      servo2.attach(6);
      digitalWrite(9,LOW);
      Stable=0; //change STABLE state
    }
    
  }//end of STABLE LOOP
}//loop end

////////////////////////Functions////////////////// 
///// DESIRED POSITION
void setDesiredPosition()
{
 
  
 nunchuk_get_data(); 
 //if zbutton is pressed, zero positions
 
 int c = nunchuk_zbutton();
 if (c != lastButtonState) {
    // if the state has changed, increment the counter
  if (c == HIGH && digitalRead(11)==0 ) {
      // if the current state is HIGH then the button
      // wend from off to on:
      buttonPushCounter++;   
    }
  }
   lastButtonState =c;
   
   switch (buttonPushCounter)
   {
    case 1:
    Setpoint=120;
    Setpoint1=70;
    fl=1;
    break;
    case 2:
    Setpoint=52;
    Setpoint1=70;
    fl=2;
    break;
    case 3:
    Setpoint=52;
    Setpoint1=40;
    fl=3;
    break; 
    case 4:
    Setpoint=120;
    Setpoint1=40;
    buttonPushCounter=0;
    fl=4;
    break;  
   }  
    if (nunchuk_cbutton()&&fl==1)///LEMNISCATE TRAJECOTRY
  {
    Setpoint = 85+ (50*cos(k))/(1+sin(k)*sin(k));
    Setpoint1 = 55+ (50*sin(k)*cos(k))/(1+sin(k)*sin(k));
    buttonPushCounter=0;
    k=k+0.008; 
  }
  if (nunchuk_cbutton()&&fl==2)// CIRCLE TRAJECTORY
  {
    Setpoint = 85+ 25*cos(k);
    Setpoint1 = 55+ 25*sin(k);
    buttonPushCounter=0;
    k=k-0.02; 
  }
  if (nunchuk_cbutton()&&fl==3)/// ELLIPSE TRAJECORY
  {
    Setpoint = 85+ 40*cos(k);
    Setpoint1 = 55+ 25*sin(k);
    buttonPushCounter=0;
    k=k-0.02; 
  }
  if (nunchuk_cbutton()&&fl==4) //PENTAGRAM TRAJECOTRY
  {
    Setpoint =85+  18*cos(k)+12*cos(k*150);//
    Setpoint1 =55+ 18*sin(k)-12*sin(k*150);//
    buttonPushCounter=0;
    k=k+0.01; 
  }
}
 
