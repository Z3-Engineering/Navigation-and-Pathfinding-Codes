//Based on encoder code by Mat Wolf

 int val; 
 int encoder0PinA = 8;
 int encoder0PinB = 9;
 int encoder0PinALast = LOW;
 int n = LOW;
 const float pi = 3.14159;
 float robotRadius = 19.486; //cm
 float wheelRadius = 5.5; //cm

 void setup() { 
   pinMode (encoder0PinA,INPUT);
   pinMode (encoder0PinB,INPUT);
   Serial.begin (9600);
 } 

void loop(){

}

 void rotate(float thetaDeg) { 
   float thetaRad = 180/pi*thetaDeg;
   int encoder0Pos = 0;
   
  while (thetaRad >= 2*pi){ //puts angle in correct range
    thetaRad = thetaRad - 2*pi;
  }
  while (thetaRad <= -2*pi){ //puts angle in correct range
    thetaRad = thetaRad + 2*pi;
    }
  if (thetaRad > 0 && thetaRad <=pi){ //Readjust angle based on if rotating clockwise or counterclockwise is shorter.
    //motospd(30,30,30);
  }
  else if (thetaRad <= -pi && thetaRad >2*pi){
    thetaRad = pi + thetaRad;
    //motospd(30,30,30);
  }
  else if(thetaRad > pi && thetaRad < 2*pi){
    thetaRad = 2*pi-thetaRad; 
    //motospd(-30,-30,-30);
  }
  else if (thetaRad < 0 && thetaRad > -2*pi){
    thetaRad = -1*thetaRad;
    //motospd(-30,-30,-30);
  }
  else {
    //motospd(0,0,0);
    Serial.print("I'm not gonna rotate");
  }
   int targetPos = (int) (pi/100)*robotRadius*thetaRad/wheelRadius; //Testing shows ~100 pulses per 180deg of rotation.
   while (encoder0Pos < targetPos){
     n = digitalRead(encoder0PinA);
     if ((encoder0PinALast == LOW) && (n == HIGH)) {
       if (digitalRead(encoder0PinB) == LOW) {
         encoder0Pos--;
       } else {
         encoder0Pos++;
       }
       Serial.print ("Target Position: ");
       Serial.println (targetPos);
       Serial.print ("Current Position: ");
       Serial.println (encoder0Pos);
       
      } 
      }
      //motospd(0,0,0); //Stop the Robot
      encoder0PinALast = n;
 } 
