/***************************************************************************/
//TEAMJ TERM PROJECT : AUTONOMOUS CAR WITH ADAPTIVE CRUISE CONTROL
//AND AUTOMATIC SPEED REDUCTION 
//FEATURE: FORWARD COLLISION CONTROL AND ADAPTIVE CRUISE CONTROL 
//WITH ULTRASONIC SENSOR
//AUTHOR : MAYA KARUTHEDATH
/***************************************************************************/
////////////////////////////////////////////////////////
// ENA IN1 IN2 IN3 IN4 ENB
int myPins[6] = {5, 4, 7, 8, 9, 6};
float deadZone = 0.15;
//int baseSpeed = 130;
const int trigPin = 1;  // ultrasonic sensor trigger pin 
const int echoPin = 2; // ultrasonic sensor echo pin 
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//variable declarations
const int buffersize = 10;
int buffer[buffersize] ;
int index;
long duration;
int distance;
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//function declarations
int average();
int ultrasonic_sensor_Read();
void moveRobot(int leftSpeed, int rightSpeed);
////////////////////////////////////////////////////////
//set up -  configuration
void setup() {
  // configure pin direction 
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
 //motor pins as output
  for (int i = 0; i < 6; i++) {
    pinMode(myPins[i], OUTPUT);
  }
  Serial.begin(9600);        // set baudrate as 9600

}
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
void loop() {
  
  //local variables
  int sensordistance;
  sensordistance = ultrasonic_sensor_Read();
  // Forward collision control
  if((sensordistance <= 50) && (sensordistance >= 20))
  {
    moveRobot(70,70);
  }
  else if(sensordistance <=20)   // adaptive cruise control
  {
    moveRobot(0,0);
  }
  else
  {
     moveRobot(150,150);

  }

}
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//sensor read function - ultrasonic sensor
int ultrasonic_sensor_Read()
{
  
  //ultrasonic sensor code
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;  //distance =  speed of sound * time/2 |(
                                    //speed of sound =  340 m/s =.034cm/microsecond)


  return distance;
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
