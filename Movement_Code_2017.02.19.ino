#include <TimeLib.h>

//Variable Declarations// 
int wheel_pwm[] = {5,6,7}; //pins on motor drive to control speed
int wheel_direction[] = {2,3,4}; //pins to control wheel direction
double wheel_angles[] = {270, 30, 150}; //wheel angles are defined
double wheel_speed[3]; //speed of each wheel is stored in this array
double max_speed = 70; //maximum speed
double max_speedR = 40; //rotate speed
const int motor_relay=8; //Does not want to be changed
time_t timerTime; //Time after Arduino is powered on that timing starts
time_t timerReading; //Time since timing starts
int boxLocation [2] = {0,0}; //Location of the robot in the room, represented as grid, with origin and back left corner. 
float Compass = 0; //Angle the robot is facing, relative to the room with 0 deg straight back into room, assumes robot is facing 0 deg. 
const float velocity = 200; //velocity in cm/s. !!!!!!.!!!!!NEED TO CALIBRATE!!!!!!!!!!!
float boxSize = 50; //Size of the gridsquares that make up the room.
//Variable Declarations//

//Setup//
void setup() {     //does not return anything
  Serial.begin(9600);  // begin serial to computer
  Serial3.begin(9600);    // begin serial to BASE ARDUINO
  Serial1.begin(9600);    // begin serial to SD card
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    pinMode(wheel_direction[i], OUTPUT); //wheel directions are OUTPUT
  }
  pinMode(motor_relay,OUTPUT);
  digitalWrite(motor_relay,HIGH);
  delay(100);  //delay of .1 seconds so that you do not compile errors
  // find_red2();
  // setTime(7,0,0,1,1,1);    // (hr,min,sec,day,month,yr)
}
//Setup//

void loop(){
  translatePolar(100,100);
}

void complexMove(int goalX, int goalY, float orientation){ //Robot winds up in a specified grid, facing a specified direction, measured in grid squares and degrees.  
  translateCart(goalX, goalY); //Robot moves to a specified location
  float dTheta = orientation - Compass;
  Compass = orientation; 
  rotate(dTheta); //Robot rotates to face a specified direction
}

void translatePolar(float azimuth, float destDistCm){ //Robot moves a specified distance and direction. 
  timerReset(); //Reset the timer. 
  float duration = 1000 * (destDistCm / velocity); //time needed to travel in ms.
  Serial.print("The Robot Will Now Move for ");
  Serial.print(duration/1000);
  Serial.println("Seconds");
  wheel_speeds(40,azimuth); //speed in "motospd units"
  while (timerReading < duration){
  boxUpdate(azimuth,timerReading,velocity);
  timerRead(); 
  Serial.print("The Robot is in Box: ");
  Serial.print(boxLocation[1]  );
  Serial.println(boxLocation[2]  );
  delay(10);
  }
  Serial.print("The Robot has Stopped");
  motospd(0,0,0); //Stop the robot
}

void translateCart(int goalX, int goalY){ //Determines the distance and direction the robot needs to move to reach a specified grid (X,Y);
  int destBox[2] = {goalX, goalY};
  int deltaBox[2];//How many boxes right/left, up/down the robot needs to move to reach desired location.
  float deltaCm[2]; //Distance in cm that the robot needs to move up/down/left/right. 
  for (int i = 0; i <3; ++i) //Math
  {
    deltaBox[i] = destBox[i]-boxLocation[i];
    deltaCm[i] = boxSize*deltaBox[i];
  }
  float destDistCm = sqrt(pow(deltaCm[1],2)+pow(deltaCm[2],2));
  float azimuth = atan2(deltaCm[2],deltaCm[1]);
  translatePolar(azimuth, destDistCm); //Once the robot determines the distance and 
}

void rotate(float deg){ //input the angle you want to face, relative to the direction the robot is currently facing. 
  
  while (deg >= 360){ //puts angle in correct range
  deg = deg - 360;
  }
  
  while (deg <= -360){ //puts angle in correct range
  deg = deg + 360;
  }
  
  if (deg > 0 && deg <=180){
    motospd(30,30,30);
  }
  else if (deg <= -180 && deg >-360){
    deg = 360 + deg;
    motospd(30,30,30);
  }
  else if(deg >180 && deg < 360){
    deg = 360-deg; 
    motospd(-30,-30,-30);
  }
  else if (deg <0 && deg > -180){
    deg = -1*deg;
    motospd(-30,-30,-30);
  }
  else if (deg = 0) {
    motospd(0,0,0);
  }
  float runtime = 1000*(0.0276*deg - 0.2101); //time it takes to rotate in milliseconds, based on calibration data.
  unsigned long longTime = (unsigned long) runtime;
  delay(longTime);
  motospd(0,0,0);
  delay(100); //extra delay to avoid errors. 
  
}

void wheel_speeds(float tot_spd, float ang){
  float results[4];
  Relative_ang(results,ang);

  float Motor_2 = ((((-1/sqrt(3))*tan((results[0]/180)*M_PI))-1)*(tot_spd/(1/cos((results[0]/180)*M_PI))));
  float Motor_1 = -1*((((1/sqrt(3))*tan((results[0]/180)*M_PI))-1)*(tot_spd/(1/cos((results[0]/180)*M_PI))));  

  float Motor_Comp = .0018*pow(results[0],3) - 0.1019*pow(results[0],2) + 2.446*results[0] + 0.8571;

  int a = abs(results[1])-1;
  int b = abs(results[2])-1;
  int c = abs(results[3])-1;

 float motors[3];
 motors[a]= (results[1]/(abs(results[1]))) * Motor_1;
 motors[b]= Motor_2;
 motors[c]= Motor_Comp;
}

void Relative_ang(float *ptr, float ang){
//float a,b,c,d;
// float new_config[4]={0,0,0,0};
// float *ptr[4];
 float new_ang;
    if(ang >= 0 && ang<= 30){
      new_ang = ang;
     //new_config = {new_ang, 3,2,1};
      ptr[0] = new_ang;
      ptr[1] = 3;
      ptr[2] = 2;
      ptr[3] = 1;      
    }
     else if(ang >= 330 && ang<= 360){
      new_ang = ang;
     //new_config = {new_ang, 3,2,1};
      ptr[0] = new_ang - 360;
      ptr[1] = 3;
      ptr[2] = 2;
      ptr[3] = 1;      
    }
    else if (ang > 30 && ang <= 90){
     new_ang = ang - 60;
    //new_config = {new_ang, -2,-1,3};
     ptr[0] = new_ang;
      ptr[1] = -2;
      ptr[2] = -1;
      ptr[3] = 3;
    }
    else if (ang > 90 && ang <= 150){
      new_ang = ang - 120;
     //new_config = {new_ang, 3,1,2};
      ptr[0] = new_ang;
      ptr[1] = 3;
      ptr[2] = 1;
      ptr[3] = 2;
    }
    else if(ang > 150 && ang<= 210){
      new_ang = ang - 180;
     // new_config = {new_ang, -3,-2,1};
      ptr[0] = new_ang;
      ptr[1] = -3;
      ptr[2] = -2;
      ptr[3] = 1;
    }
    else if(ang > 210 && ang<= 270){
      new_ang = ang - 240;
     // new_config = {new_ang, 2,1,3};
      ptr[0] = new_ang;
      ptr[1] = 2;
      ptr[2] = 1;
      ptr[3] = 3;
    }
    else if(ang > 270 && ang< 330){
      new_ang = ang - 300;
     // new_config = {new_ang, -3,-1,2};
      ptr[0] = new_ang;
      ptr[1] = -3;
      ptr[2] = -1;
      ptr[3] = 2;
    }
 //return 0;
}
void boxUpdate(float dirc, float dur, float spd){ //Take input direction, time, and speed of robot. Direction taken from some other function, time, from timer, and spd assumed to be absolute. 
      boxLocation [1] = (int) (spd*dur*10*cos(dirc))/boxSize;
      boxLocation [2] = (int) (spd*dur*10*sin(dirc))/boxSize;    
    }
  
//Timing Functions//
void timerReset(){
    timerTime = millis();   
    }

void timerRead(){
  timerReading = millis()-timerTime;
}
    
void timerTest() {
  timerReset();
  Serial.println("Start Exercise");
  delay(1000);
  for (int i=0; i<10; i++){
    timerRead();
  Serial.print("The Measured Time From Exercise Start is: ");
  Serial.print(timerReading);
  Serial.println("ms");
  Serial.print("The abssolute time is ");
  Serial.print(millis());
  Serial.println("ms");
  Serial.println("--------------------------------------------------------------------------");
  delay(1000);
  }
  Serial.println("Exercise End");
  delay(5000);
  }
//Timing Functions//
    
void motospd(int sp1,int sp2,int sp3){
 if(sp1>0){
  digitalWrite(wheel_direction[0],HIGH);
  }else{
    digitalWrite(wheel_direction[0],LOW);
  }
  if(sp2>0){
    digitalWrite(wheel_direction[1],HIGH);
  }else{
    digitalWrite(wheel_direction[1],LOW);
  }
  if(sp3>0){
    digitalWrite(wheel_direction[2],HIGH);
  }else{
    digitalWrite(wheel_direction[2],LOW);
  }
  analogWrite(wheel_pwm[0],abs(sp1));
  analogWrite(wheel_pwm[1],abs(sp2));
 analogWrite(wheel_pwm[2],abs(sp3));
}
