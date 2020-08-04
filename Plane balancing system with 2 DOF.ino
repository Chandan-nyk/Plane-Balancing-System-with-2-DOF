#include <Servo.h>
#include <PID_v1.h>
#include "MegunoLink.h"
#include "RunningAverage.h"

//Touch panel filter (Running Average or Moving Average filter)
RunningAverage myRAX(6);
RunningAverage myRAY(6);

int plotCounter = 0;

int Xnew, Ynew;
int xServoPin = 3;
int yServoPin = 5;

//Reference positions or desired positions
double xRef = 500;
double yRef = 500;

//Parameters of PID controller

43

double kpx = 0.04;
double kix = 0.018;
double kdx = 0.018;

double kpy = 0.03;
double kiy = 0.022;
double kdy = 0.018;

double xInput, yInput;
double xOutput, yOutput;

//Sampling time*******
int Ts = 50;

XYPlot MyPlot;
TimePlot MyYTimePlot;
TimePlot MyXTimePlot;

PID xPID(&xInput, &xOutput, &xRef, kpx, kix, kdx, DIRECT);
PID yPID(&yInput, &yOutput, &yRef, kpy, kiy, kdy, DIRECT);

Servo xServo, yServo;

44

void setup() {

xServo.attach(xServoPin);
yServo.attach(yServoPin);

xOutput = 90;
yOutput = 90;

Serial.begin(9600);

xPID.SetMode(AUTOMATIC);
xPID.SetOutputLimits(60, 120);
xPID.SetSampleTime(Ts);

yPID.SetMode(AUTOMATIC);
yPID.SetOutputLimits(60, 120);
yPID.SetSampleTime(Ts);

xServo.write(xOutput);
yServo.write(yOutput);
delay(5000);
}

45

void loop() {

Xnew = getX();
myRAX.addValue(Xnew);
xInput = myRAX.getAverage();
xPID.Compute();
xServo.write(180 - xOutput);

Ynew = getY();
myRAY.addValue(Ynew);
yInput = myRAY.getAverage();
yPID.Compute();
yServo.write(180 - yOutput);

//Plot values
if(plotCounter == 100){
plotCounter = 0;
//MyPlot.SendData("SetXY", xRef, yRef);
MyPlot.SendData("Filter", xInput, yInput);
MyPlot.SendData("Original", Xnew, Ynew);
//MyTimePlot.SendData(F("Ori"), Ynew);
//MyYTimePlot.SendData(F("FilY"), yInput);
//MyYTimePlot.SendData(F("SetY"), yRef);

46
//MyXTimePlot.SendData(F("FilX"), xInput);
//MyXTimePlot.SendData(F("SetX"), xRef);
}
plotCounter++;
}

int getX(){

int X;

pinMode(A0, OUTPUT);
pinMode(A2, OUTPUT);
pinMode(A3, INPUT);
pinMode(A1, INPUT);

digitalWrite(A0, HIGH);
digitalWrite(A2, LOW);
delay(1);

X = analogRead(A3);
delay(1);

return X;

47

}

int getY(){

int Y;

pinMode(A0, INPUT);
pinMode(A2, INPUT);
pinMode(A3, OUTPUT);
pinMode(A1, OUTPUT);

digitalWrite(A3, HIGH);
digitalWrite(A1, LOW);
delay(1);

Y = analogRead(A0);
delay(1);

return Y;
}
