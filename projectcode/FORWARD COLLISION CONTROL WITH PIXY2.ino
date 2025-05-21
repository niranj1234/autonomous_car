/***************************************************************************/
//TEAMJ TERM PROJECT : AUTONOMOUS CAR WITH ADAPTIVE CRUISE CONTROL
//AND AUTOMATIC SPEED REDUCTION 
//FEATURE: FORWARD COLLISION CONTROL WITH PIXY2
//AUTHOR : NIRANJAN CHENNASAMUDRAM BALAJI
/***************************************************************************/
////////////////////////////////////////////////////////
//Inclusions
#include <SPI.h>
#include <Pixy2.h>

////////////////////////////////////////////////////////
//pin declarations
// ENA IN1 IN2 IN3 IN4 ENB
int myPins[6] = {5, 4, 7, 8, 9, 6};
////////////////////////////////////////////////////////
//variable declarations
float deadZone = 0.15;
Pixy2 pixy;
int cont = 0;
int signature, x, y, width, height;
float cx, cy, area;
int object_found;
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//configuration function
void setup() {
  Serial.begin(115200);
  Serial.print("Starting...n");
  pixy.init();
  for (int i = 0; i < 6; i++) {
    pinMode(myPins[i], OUTPUT);
  }
}
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//Program function
void loop() {
  object_found = 0;
  //check pixy objects
  float turn = pixyCheck();
  int sensordistance;
  //FCA range
  if(height >=188)
  {
    sensordistance = 20;
  }
  else if ((height <=96 ) && (height >=94)) //Distance = 50
  {
    sensordistance = 50;
  }
  else if(height < 94)  // Distance >50 -> Move at normal speed
  {
     moveRobot(150,150);
     Serial.print("normal speed: ");
  }
   Serial.print("sensordistance: ");
   Serial.print(sensordistance);
   //Reduced speed
  if((sensordistance<= 50) && (sensordistance > 20))
  {
    moveRobot(64,64);
    Serial.println("reduced speed: ");
  }
  else if(sensordistance <=20)   // Forward Collision Avoidance
  {
    moveRobot(0,0);
    Serial.println("stop: ");
  }

  
}
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//Pixy function
float pixyCheck() {
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];
  // grab blocks!
  blocks = pixy.ccc.getBlocks();
  Serial.print(" \nchecking for blocks ");

  // If there are detect blocks, print them!
  if (blocks)
  {
    signature = pixy.ccc.blocks[0].m_signature;
    height = pixy.ccc.blocks[0].m_height;
    width = pixy.ccc.blocks[0].m_width;
    x = pixy.ccc.blocks[0].m_x;
    y = pixy.ccc.blocks[0].m_y;
    cx = (x + (width / 2));
    cy = (y + (height / 2));
    cx = mapfloat(cx, 0, 316, -1, 1);
    cy = mapfloat(cy, 0, 208, 1, -1);
    area = width * height;
    object_found = 1;
    Serial.print(" \nobject found ");

            Serial.print("sig: ");
            Serial.print(signature);
            Serial.print(" x:");
            Serial.print(x);
            Serial.print(" y:");
            Serial.print(y);
            Serial.print(" width: ");
            Serial.print(width);
            Serial.print(" height: ");
            Serial.print(height);
            Serial.print(" cx: ");
            Serial.print(cx);
            Serial.print(" cy: ");
            Serial.println(cy);

  }
  else {
    
    cont += 1;
    if (cont == 100) {
      cont = 0;
      cx = 0;
      Serial.print(" \nNo object ");

    }
  }
  
  return cx;
}
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//Mapping function
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
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