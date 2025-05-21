/***************************************************************************/
//TEAMJ TERM PROJECT : AUTONOMOUS CAR WITH ADAPTIVE CRUISE CONTROL
//AND AUTOMATIC SPEED REDUCTION 
//FEATURE: BAR CODE DETECTION AND SPEED REDUCTION WITH PIXY 2
//AUTHOR : MAYA KARUTHEDATH
/***************************************************************************/
////////////////////////////////////////////////////////
//Inclusions
#include <Pixy2.h>
#include <SPI.h>
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//variable declarations
Pixy2 pixy;
int slow = 0;
int fast = 1;
int myPins[6] = {5, 4, 7, 8, 9, 6};
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//configuration function
void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");

  // we need to initialize the pixy object
  pixy.init();
  // Change to line tracking program
  pixy.changeProg("line");

  //set the original speed
   moveRobot(150,150);
}
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//Program function
void loop()
{
  int8_t i;
  char buf[128];
  int8_t Linedata;
 Linedata = pixy.line.getMainFeatures();

  //   barcodes detected 
  if(Linedata&LINE_BARCODE)
  {
     // print  barcode
    pixy.line.barcodes->print();
    //slow down
    if(slow == pixy.line.barcodes->m_code)
    {
       moveRobot(70,70);
    }
    else if(fast == pixy.line.barcodes->m_code)
    {
       moveRobot(150,150);
	   //back to normal speed
    }   
  }
  else
  {
    // do nothing
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