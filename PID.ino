#include <PID_v1.h>
#include <NewPing.h>
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;
double cm[2]; //sensed distance of sonar sensor : 2 represents array(2 sonar sensors)
int trigger[2] = {36,32}; //trigger pins assignment  (sends)   
int echo[2] = {37,33}; //echo pins (receives)

int wheel_pwm[3] = {2,3,4}; //pins on motor drive to control speed
int wheel_direction[3] = {5,6,7}; //pins to control wheel direction
float omega1,omega2,omega3; //individual wheel speeds // rad/s
float omegas[3]; // resultant wheel speed array to be populated by specified movement params
float R = 0.0508; // radius of wheel // m
float l=8.5*2.54*.01; //lever arm for each wheel 
float root3=1.7321;

//set up the sonar sensors for NewPing Library
NewPing sonar[2] =
{
  NewPing(trigger[0], echo[0], 500), //500 represents type of sensors
  NewPing(trigger[1], echo[1], 500), //500 represents type of sensor

};
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup()
{
  //initialize the variables we're linked to

  // unimatMoveSensedForward(15,15,0,2);
  //delay(5000);
  //motospd(0,0,0);
  Setpoint = 40;
  Serial.begin(9600);
  //turn the PID on
//  myPID.SetMode(AUTOMATIC);
//  myPID.SetOutputLimits(0,30);
}

void loop()
{
//  Serial.println(PID_Wall_Adjust());
//unimatMoveSensedForward(0,1,0,2);
//  delay(1000);
motospd(0,30,-30);
}
void updateSonar() {
  for (int i = 0; i < 2; i++) {
    //cm[i]=sonar[i].ping_cm();
    unsigned int uS = sonar[i].ping();
    cm[i] = sonar[i].convert_cm(uS);
    delay(50);

//    Serial.println(cm[i]);//



  }
}
void sonarInfoDisplay() {
  Serial.println("Distance Display: ");
  for (int i = 0; i < 2; i++) {
    Serial.print("Sonar Pin ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(cm[i]);
  }
  delay(50);
}





float PID_Wall_Adjust(){
   updateSonar();
  sonarInfoDisplay();
  Input = cm[0];
  if(Input<Setpoint)
    myPID.SetControllerDirection(DIRECT);
  else
    myPID.SetControllerDirection(REVERSE);
    
  float gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
  myPID.Compute();
  Serial.println(myPID.GetDirection());
  return Output;
  
}

void unimatMovestrict(double Vx, double Vy, double omega,int roboID){
  // first some physics to convert the desired velocities to component angular wheel speeds
  omega1 = 19.685 * (-1*Vx +    0*Vy + l*omega);
  Serial.print("\n omega1:  ");Serial.print(omega1,5);
  omega2 = 19.685 * (.5*Vx + .866*Vy + l*omega);
  Serial.print("\n omega2:  ");Serial.print(omega2,5);
  omega3 = 19.685 * (.5*Vx - .866*Vy + l*omega);
  Serial.print("\n omega3:  ");Serial.print(omega3,5);
  // now store in array and convert to motospeeds
  omegas[1]=omega1;
  omegas[2]=omega2;
  omegas[3]=omega3;
  for(int i;i<3;i++){
    omegas[i]=calibrator(omegas[i],roboID);
    Serial.print("\n motoomega");Serial.print(i);Serial.print(":  ");Serial.print(omegas[i],5);
  }
  motospd(omegas[1],omegas[2],omegas[3]);
}

void unimatMoveSensedForward(float Vx, float Vy, float omega,int roboID){
  // first some physics to convert the desired velocities to component angular wheel speeds
//  omega1 = R * (-0.6667*Vx  + 0.3333*Vy +  0.3333*omega);
//  Serial.print("\n omega1:  ");Serial.print(omega1);
//  omega2 = R * ( 0*Vx       + 0.629*Vy  + -0.629*omega);
//  Serial.print("\n omega2:  ");Serial.print(omega2);
//  omega3 = R * ( 1.544*Vx   + 1.544*Vy  +  1.544*omega);
//  Serial.print("\n omega3:  ");Serial.print(omega3);
  omega1 = .19685 * (1*Vx  + 0*Vy +  l*omega);
  Serial.print("\n omega1:  ");Serial.print(omega1);
  omega2 = .19685 * ( -0.50*Vx       + 3.464*Vy  + l*omega);
  Serial.print("\n omega2:  ");Serial.print(omega2);
  omega3 = .19685 * ( -0.50*Vx   + -3.464*Vy  +  l*omega);
  Serial.print("\n omega3:  ");Serial.print(omega3);
  // now store in array and convert to motospeeds
  omegas[0]=omega1;
  omegas[1]=omega2;
  omegas[2]=omega3;
 //Serial.println(omegas[1]); 
  for(int i;i<3;i++){
    omegas[i]=calibrator(omegas[i],roboID); 
    Serial.print("\n motoomega");Serial.print(i);Serial.print(":  ");Serial.print(omegas[i]);
  }

  return motospd(omegas[0],omegas[1],omegas[2]);
}



float calibrator(float omega, int roboID){
  switch (roboID){
    case 0:
       return 59.918*omega; //robot 0
    case 1:
       return 63.41*omega; //robot 1
    case 2:
      // Serial.print(omega);
       return 61.236*omega;//robot 2
  }
}
   


void motospd(int sp1, int sp2, int sp3) {
  if (sp1 > 0) {
    digitalWrite(wheel_direction[0], HIGH);
  } else {
    digitalWrite(wheel_direction[0], LOW);
  }
  if (sp2 > 0) {
    digitalWrite(wheel_direction[1], HIGH);
  } else {
    digitalWrite(wheel_direction[1], LOW);
  }
  if (sp3 > 0) {
    digitalWrite(wheel_direction[2], HIGH);
  } else {
    digitalWrite(wheel_direction[2], LOW);
  }
  analogWrite(wheel_pwm[0], abs(sp1));
  analogWrite(wheel_pwm[1], abs(sp2));
  analogWrite(wheel_pwm[2], abs(sp3));
}


