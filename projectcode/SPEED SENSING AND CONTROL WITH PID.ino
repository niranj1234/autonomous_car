/***************************************************************************/
//TEAMJ TERM PROJECT : AUTONOMOUS CAR WITH ADAPTIVE CRUISE CONTROL
//AND AUTOMATIC SPEED REDUCTION 
//FEATURE:SPEED SENSING AND CONTROL WITH PID
//AUTHOR : MAYA KARUTHEDATH
/***************************************************************************/
////////////////////////////////////////////////////////
//Inclusions
#include <PID_v1.h>
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//variable declarations
int sensor = 2;
int myPins[6] = {5, 4, 7, 8, 9, 6};
unsigned long start_time = 0;
unsigned long end_time = 0;
int steps=0;
float steps_old=0;
float temp=0;
float rps=0;
int sensorRpm=0;
int targetRpm=0;
int lastError;
const int buffersize = 5;
int buffer[buffersize] ;
int index;
 boolean Pidvar= false;
//PID variables
double Setpoint, Input, Output;
double P,D;
double Kp=0.5;
double Ki=0;
double Kd=3;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//configuration function
void setup() 
{
  Serial.begin(9600);
  Serial.begin(9600);
  pinMode(sensor,INPUT_PULLUP);
  for (int i = 0; i < 6; i++) {
    pinMode(myPins[i], OUTPUT);
  }
   // Set speed to 25% duty = 64 . 
   moveRobot(64,64);
}
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//Program function 
void loop()
{
 //LM393 Sensor read for 1 second
 int sensorspeed;
 start_time=millis();
 end_time=start_time+1000;
 while(millis()<end_time)
 {
   if(digitalRead(sensor))
   {
    steps=steps+1; 
    while(digitalRead(sensor));
   }
 }
    // calculate total sensor recorded steps and 
	//find average
    temp=steps-steps_old;
    steps_old=steps;
    sensorspeed = average();
	// no of slots in the encoder wheel =  20
    rps=(sensorspeed/20);  

    sensorRpm=rps * 60;

    //update average buffer
    buffer[index]=temp;
    index++;
    if(index>=buffersize)
    {
      index=0;
    }
    
    //PID Control
    //target RPM
    targetRpm = 660; 
    Setpoint = targetRpm;
    Input = sensorRpm;

   int error = targetRpm - sensorRpm;
   int readtime = millis();

    Serial.print("targetRpm:");
    Serial.print(targetRpm);
    Serial.print("\t");
    Serial.print("sensorRpm:");
    Serial.println(sensorRpm);
    //10 s delay for allowing the speed to stabilize
    if((readtime == 10000) && (error != 0))
    {
          // calculate the proportional term
        if (error >= 0)
        {
          P = Kp * error;
        }
        else
        {
		  // For negative error , reduce the output by 1/4th
          P = Kp * error * 0.25;
        }
    
        // calculate the derivative term
        D = Kd * ((error - lastError) / 0.5); // 0.5 is the sampling time in seconds
        
        // calculate the output of the PID controller
        Output = P + D;
        
        // update the last error
        lastError = error;
        
        // check if the output is within the allowed range
        if(Output > 255)
        {
          Output = 255;
        }
        else if(Output < -255)
        {
          Output = -255;
        }
        
        Serial.print("proportional=");
        Serial.print(P);   
        Serial.print("Differential=");
        Serial.print(D);
        Serial.print("Output=");
        Serial.print(Output);

        // Motor output power   
        moveRobot(abs(Output),abs(Output));
 
    }
    
  }

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//Motor function
void moveRobot(int leftSpeed, int rightSpeed)
{
  //Move left cond1
  if (leftSpeed >= 0) {
    digitalWrite(myPins[1], 0);
    digitalWrite(myPins[2], 1);
  }
  else {
    digitalWrite(myPins[1], 1);
    digitalWrite(myPins[2], 0);
  //Move right cond1

  }

  if (rightSpeed >= 0) {
    digitalWrite(myPins[3], 0);
    digitalWrite(myPins[4], 1);
    //Move right cond2

  }
  else {
    digitalWrite(myPins[3], 1);
    digitalWrite(myPins[4], 0);
   //Move left cond1

  }

  analogWrite(myPins[0], abs(leftSpeed));
  analogWrite(myPins[5], abs(rightSpeed));
  Serial.print("\nLeft speed:");
  Serial.println("abs(leftSpeed)");
   Serial.print("\nRight  speed");
  Serial.println("abs(rightSpeed)");

}
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//sensor reading averaging function
int average()
{
   long sum = 0;
   int i;
   int avg;
   
   for(i=0;i<buffersize;i++)
   {
     sum += buffer[i]  ;
   }
   
   avg = (sum/buffersize);
   
   return avg;
}
////////////////////////////////////////////////////////
