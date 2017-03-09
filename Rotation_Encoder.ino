//Based on encoder code by Mat Wolf

 int val; 
 int encoder0PinA = 10;
 int encoder0PinB = 9;
 int encoder0PinALast = LOW;
 int n = LOW;
 const float pi = 3.14159;
 float robotRadius = 23; //cm
 float wheelRadius = 5.5; //cm
 
int wheel_pwm[] = {2,3,4}; //pins on motor drive to control speed
int wheel_direction[] = {5,6,7}; //pins to control wheel direction
double wheel_angles[] = {270, 30, 150}; //wheel angles are defined
double wheel_speed[3]; //speed of each wheel is stored in this array
double max_speed = 70; //maximum speed
double max_speedR = 40; //rotate speed
const int motor_relay=8;


 void setup() { 
   pinMode (encoder0PinA,INPUT);
   pinMode (encoder0PinB,INPUT);
   Serial.begin (9600);
    pinMode(LED_BUILTIN, OUTPUT);
  
  for (int i = 0; i < 3; i++) {
    pinMode(wheel_direction[i], OUTPUT); //wheel directions are OUTPUT
  }
  pinMode(motor_relay,OUTPUT);
  digitalWrite(motor_relay,HIGH);
    
  Serial3.begin(1550);    // begin serial to BASE ARDUINO


 } 

void loop(){
  
  rotate(90);
  delay(1000);
  rotate(90);
  delay(5000);
  rotate(-90);
  delay(1000);
  rotate(-90);
  delay(5000);
  
  
//motospd(30,30,30);
}

 void rotate(float thetaDeg) { 
   float thetaRad = pi/180*thetaDeg;
   int encoder0Pos = 0;
   Serial.print("Target Angle");  
   Serial.println(thetaRad);
//   while (thetaRad >= 2*pi){ //puts angle in correct range
//  thetaRad = thetaRad - 2*pi;
//  }
//  while (thetaRad <= -2*pi){ //puts angle in correct range
//    thetaRad = thetaRad + 2*pi;
//    }
   if (thetaRad > 0 && thetaRad <=pi){ //Readjust angle based on if rotating clockwise or counterclockwise is shorter.
    motospd(50,50,50);
    Serial.println("Start the Robot");
  }
  else if (thetaRad <= -pi && thetaRad >2*pi){
    thetaRad = pi + thetaRad;
    motospd(50,50,50);
  }
  else if(thetaRad > pi && thetaRad < 2*pi){
    thetaRad = 2*pi-thetaRad; 
    motospd(-50,-50,-50);
  }
  else if (thetaRad < 0 && thetaRad > -2*pi){
    thetaRad = -1*thetaRad;
    motospd(-50,-50,-50);
  }
  else {
    motospd(0,0,0);
    Serial.print("I'm not gonna rotate");
  }
   int targetPos = (int) (104/pi)*robotRadius*thetaRad/wheelRadius; //Testing shows ~104 pulses per 180deg of rotation.
   Serial.print ("Target Position: ");
   Serial.println (targetPos);
   while(abs(encoder0Pos) < abs(targetPos))
   {
     n = digitalRead(encoder0PinA);
     if ((encoder0PinALast == LOW) && (n == HIGH)) {
       if (digitalRead(encoder0PinB) == LOW) {
         encoder0Pos--;
       } else {
         encoder0Pos++;
       }
  
       Serial.print (encoder0Pos);
       Serial.println ("/");
     } 
     encoder0PinALast = n;
     //Serial.println ("ayyy");
   }
   Serial.println("Stop the Robot");
   motospd(0,0,0); //Stop the Robot
  
 } 
 
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
