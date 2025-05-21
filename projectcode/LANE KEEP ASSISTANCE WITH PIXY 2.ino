/***************************************************************************/
//TEAMJ TERM PROJECT : AUTONOMOUS CAR WITH ADAPTIVE CRUISE CONTROL
//AND AUTOMATIC SPEED REDUCTION 
//FEATURE: LANE KEEP ASSISTANCE WITH PIXY 2
//AUTHOR : NIRANJAN CHENNASAMUDRAM BALAJI
/***************************************************************************/
////////////////////////////////////////////////////////
//Inclusions and macro definitions
#include <Pixy2.h>
#include <SPI.h>
#define X_CENTER (pixy.frameWidth/2)
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//variable declarations
Pixy2 pixy;
int myPins[6] = {5, 4, 7, 8, 9, 6};
float left_slope=0;
float right_slope=0;
int xdif =0;
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//Configuration function
void setup()
{
   pixy.init();
   //turn the toggle lamp on
  pixy.setLamp(1, 1);
  Serial.begin(115200);
  Serial.print("Starting...\n");
    for (int i = 0; i < 6; i++) {
    pinMode(myPins[i], OUTPUT);
  }
  // we need to initialize the pixy object
  pixy.init();
  // Change to line tracking program
  pixy.changeProg("line");
}
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//Program function
void loop()
{
  int8_t i;
  char buf[128];
  //get all lines
  pixy.line.getAllFeatures();
  Serial.print("No of vwctors :");
  Serial.println(pixy.line.numVectors);
  //atleast 1 vector must be detected
  if((pixy.line.numVectors<=2) && (pixy.line.numVectors > 0))
  {
    for (i=0; i<pixy.line.numVectors; i++)
    {
      sprintf(buf, "line %d: ", i);
      Serial.print(buf);
      pixy.line.vectors[i].print();
      // If the left line is the first vector , find left slope , else find right slope
      if(pixy.line.vectors[i].m_x0 < X_CENTER){
        left_slope = ((pixy.line.vectors[i].m_y1 - pixy.line.vectors[i].m_y0) / (pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0));
      Serial.print("left_slope=");
     Serial.println(left_slope);
      }
      else{
        right_slope =( (pixy.line.vectors[i].m_y1 - pixy.line.vectors[i].m_y0) / (pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0));
                 Serial.print("right_slope=");
     Serial.println(right_slope);
      }
    }
  }
  
   Serial.print("center=");
   Serial.println(X_CENTER);
   //calculate difference if 2 vectors are found
   if(pixy.line.numVectors == 2)
   {
     xdif = (pixy.line.vectors[0].m_x0 + pixy.line.vectors[1].m_x0);
   }
   else if(pixy.line.numVectors == 1)
   {
	 //calculate difference if 1 vector is found
     xdif = (pixy.line.vectors[0].m_x0*2);
   }
   Serial.print("difference=");
   Serial.println(xdif);
  //difference is in the range of frameWidth+/- 10 -  go straight 
  if((xdif >= 68 && xdif<= 88))
  {  //Change 10 value 
    moveRobot(70,70);
    Serial.print("Go straight");
  }
  else
  {
   //consider shich slope is greater
    if((left_slope > right_slope)){
      moveRobot(-60,80);  //turn left
       Serial.print("turn left");
    }
    else if((right_slope > left_slope)){
      moveRobot(80,-60);  //turn right
       Serial.print("turn right");
    }

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