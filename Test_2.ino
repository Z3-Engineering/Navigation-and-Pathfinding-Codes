#include <StandardCplusplus.h>
#include <system_configuration.h>
#include <unwind-cxx.h>
#include <utility.h>
#include <iostream>
#include <string>
#include <iterator>
#include <sstream>
#include <Pixy.h>
#include <SPI.h>
#include <NewPing.h>

using namespace std;

//------------------------function prototypes for compiler------------------------//

void motospd(int,int,int);
void unimatMoveSensedForward(float, float, float, int);
float calibrator(float, int);
void Pixy_Align();
int whichWall(int);
void counterbasic();
void wallCount();
void pixyCheck();
void printer(int, int);
string ToString(int);
int toOctalFromDecimal (int);
int toDecimalFromOctal (int);
bool anyCollision(int) ;
void updateSonar();
void sonarInfoDisplay();
void get_coordinates();
bool align_to_block(int);

//------------------------stream insertion configuration of "cout" for arduino use------------------------//

namespace std {
  ohserialstream cout(Serial);
}

//------------------------global variables and objects------------------------//

Pixy pixy;

int tolerance = 20; 
uint16_t blocks;
int count = 0;
int counter =0;
static int i=0;
int j;
int blue=1;
int green=2;
int pink=3;
int yellow=4;
int brown=5;
int purple=6;
int orange=7;
//int bluegreen=12;
//int greenpink=23;
//int blueyellow=14;
//int pinkblue=13;
// changing from decimal to octal for comparison becuase pixyMon displays decimal but interacts with octal
int bluegreen=10;
int greenpink=19;
int blueyellow=12;
int pinkblue=11;
int dimensions[2];
char resourceNames[3]={'W','C','I'};
int wallColors[4]={bluegreen,greenpink,blueyellow,pinkblue};
char wallColorNames[4]={'bluegreen','greenpink','blueyellow','pinkblue'};
int cornerColors[4] = {blue,green,pink,yellow}; //change later to signatures
int roboColors= brown; //color markers on robots
int resourceColors[2]={purple, orange};//water, CO2, light
int currentColor;
int observedWall;
int startCorner=0;
int endCorner=0;
int wallCounter=0;
int wheel_pwm[3] = {4,3,2}; //pins on motor drive to control speed
int wheel_direction[3] = {7,6,5}; //pins to control wheel direction
double wheel_angles[3] = {270, 30, 150}; //wheel angles are defined
double wheel_speed[3]; //speed of each wheel is stored in this array
double max_speed = 70; //maximum speed
double max_speedR = 40; //rotate speed
double cm[4]; //sensed distance of sonar sensor : 2 represents array(2 sonar sensors)
int colliDistance_cm = 12; //distances to define collision in cm/in
int colliDistance_in = 5;
int rotation_update_delay = 10;
float omega1, omega2, omega3; //individual wheel speeds // rad/s
float omegas[3];
float R = 0.0508; // radius of wheel // m
float l=8.5*2.54*.01; //lever arm for each wheel
int trigger[6] = {31, 32, 46, 36, 38, 40}; //trigger pins assignment  (sends)   
int echo[6] = {30, 33, 47, 37, 39, 41}; //echo pins (receives)C
const int motor_relay = 8; 
int rotationdelay;
NewPing sonar[6] =
{
  NewPing(trigger[0], echo[0], 500), //500 represents type of sensor
  NewPing(trigger[1], echo[1], 500),
  NewPing(trigger[2], echo[2], 500),
  NewPing(trigger[3], echo[3], 500),
  NewPing(trigger[4], echo[4], 500),
  NewPing(trigger[5], echo[5], 500),
};


//-------------------------------------------------------------------------------------------------
//----------------------------------------PID Setup Stuff------------------------------------------
//-------------------------------------------------------------------------------------------------

#include <PID_v1.h>
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp = .5, aggKi = .06125, aggKd = .02;
double consKp = 1.75, consKi = .03, consKd = .01;//Robot 1 consKp = .4, consKi = .030625, consKd = .02

float root3 = 1.7321;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//-------------------------------------------------------------------------------------------------
//---------------------------------------/PID Setup Stuff------------------------------------------
//-------------------------------------------------------------------------------------------------


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  pixy.init();
  for (int i = 0; i < 3; i++) {
    pinMode(wheel_direction[i], OUTPUT); //wheel directions are OUTPUT
  }
  pinMode(motor_relay, OUTPUT);
  digitalWrite(motor_relay, HIGH);

  //initialize the variables we're linked to
  Setpoint = 20;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 15);
  pixy.getBlocks(17);
  speed_up(60,0,0);
  wallCount(); 
  rotate(0,true);
  wallCount(); 

}

void loop() {
   
// put your main code here, to run repeatedly:
//  wallCount();
//  delay(500);
//motospd(60,60,60);
//delay(3000);
//motospd(-60,-60,-60);
//delay(3000);
//PID_Superposition(60,0,0,1);
//updateSonar();
//sonarInfoDisplay();
//rotate(1,true);
//delay(1000);

}

//------------------------movement algorithm------------------------//

void motospd(int sp1,int sp2,int sp3){// basic servo moveemnt accpeting analogWrite byte values from 0 to 255 (truncates all other data types)
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



void unimatMoveSensedForward(float Vx, float Vy, float omega, int roboID) {
  // first some physics to convert the desired velocities to component angular wheel speeds
  //  omega1 = R * (-0.6667*Vx  + 0.3333*Vy +  0.3333*omega);
  //  Serial.print("\n omega1:  ");Serial.print(omega1);
  //  omega2 = R * ( 0*Vx       + 0.629*Vy  + -0.629*omega);
  //  Serial.print("\n omega2:  ");Serial.print(omega2);
  //  omega3 = R * ( 1.544*Vx   + 1.544*Vy  +  1.544*omega);
  //  Serial.print("\n omega3:  ");Serial.print(omega3);
  omega1 = .02 * (1 * Vx  + 0 * Vy +  l * omega);//.01925 for robo1
 // Serial.print("\n omega1:  "); Serial.print(omega1);
  omega2 = 0.022 * ( -0.50 * Vx       + 3.464 * Vy  + l * omega);
 // Serial.print("\n omega2:  "); Serial.print(omega2);
  omega3 = .0235 * ( -0.50 * Vx   + -3.464 * Vy  +  l * omega);
//  Serial.print("\n omega3:  "); Serial.print(omega3);
  // now store in array and convert to motospeeds
  omegas[0] = omega1;
  omegas[1] = omega2;
  omegas[2] = omega3;
  //Serial.println(omegas[1]);
  for (int i; i < 3; i++) {
    omegas[i] = calibrator(omegas[i], roboID);
 //   Serial.print("\n motoomega"); Serial.print(i); Serial.print(":  "); Serial.print(omegas[i]);
  }

  return motospd(omegas[0], omegas[1], omegas[2]);
}

void speed_up(float vx, float vy,int robo){
  for (int i = 15; i <=fabs(vx) || i<=fabs(vy) ;i++){
    if(i<fabs(vx)){
      unimatMoveSensedForward((vx/fabs(vx))*i,vy,0,robo);
    }
    else if(i<fabs(vy)){
      unimatMoveSensedForward(vx,(vy/fabs(vy))*i,0,robo);
    }
    else{
      unimatMoveSensedForward(vx,vy,0,robo);  
      break;
    }
    delay(10);
  }
}

void rotate(int roboID, bool clockwise){
  if(clockwise)
    unimatMoveSensedForward(0,0,-200,roboID);
  else
    unimatMoveSensedForward(0,0,200,roboID);

  delay(rotationdelay);
  motospd(0,0,0);
}

float calibrator(float omega, int roboID) { //calibration function for aggregated frictional effects on robots 
    switch (roboID) {
        case 0:
            rotationdelay = 1200;
            return 59.918 * omega; //robot 0

        case 1:
            rotationdelay = 2500;
            return 63.41 * omega; //robot 1
            
        case 2:
            // Serial.print(omega);
            return 61.236 * omega; //robot 2
    }
}

//------------------------marker counting algorithm and utility functions------------------------//
void clear_blocks(){
  for(int i = 0; i <sizeof(pixy.blocks); i++){
    pixy.blocks[i].x = 0;
  }
}
bool align_to_block(int i){
  //int tolerance = 10;
  bool centered = false;
  clear_blocks();
  pixy.getBlocks();
  if(pixy.blocks[i].x == 0)
    return false;

  float pos = pixy.blocks[i].x;
  while (!centered){
    PID_Superposition(60,0,0,1);
    clear_blocks();
    pixy.getBlocks();
    if(pixy.blocks[i].x == 0)
        return false;
     if (pos > (160+tolerance)){
        
          Serial.print("The block is currently at: ");
          Serial.println(pixy.blocks[i].x);
          PID_Superposition(60,0,0,1);
          Serial.print("Moving Right to align with block: ");
          Serial.println(i);
          pixy.getBlocks();
//          if(num_blocks == 0){
//             Serial.println("No blocks on screen1111111`");
//             return false;
//          }
          pixy.getBlocks();
          pos = pixy.blocks[i].x;
          
      }
//     if(num_blocks == 0){
//          Serial.println(pixy.getBlocks());
//          Serial.println("No blocks on screen");
//          return false;
//      }
      else{
        centered = true;
      }     
  }
  
   Serial.print("I have successfully aligned with block: ");
   Serial.println(i);
   return true;      
}




void get_coordinates(){ //function to determine coordinates of present block
  int blocks = pixy.getBlocks(17);
  for (int k = 0; k < blocks; k++) {
    if (pixy.blocks[k].signature == 1) {
      Serial.print("The X coordinate is:");
      Serial.print(pixy.blocks[k].x);
      Serial.println();
      Serial.print("The Y coordinate is:");
      Serial.print(pixy.blocks[k].y);
      Serial.println();  
    }
  }
}

void Pixy_Align(){//alignment algorithm for seeking the centre a single block
  Serial.println("-------------------------------------------------");
  Serial.println("-------------------------------------------------");
  get_coordinates();
  int rotation_update_delay = 10;
  //int tolerance = 7;
  int blocks = pixy.getBlocks(17);
  if (pixy.blocks[0].x < (160 - tolerance)){
    Serial.println("Rotating CounterClockwise...");
    motospd(20,20,20);
    delay(rotation_update_delay);
    Pixy_Align();
  }
  else if(pixy.blocks[0].x > (160 + tolerance)){
    Serial.println("Rotating Clockwise...");
    motospd(-20, -20, -20);
    delay(rotation_update_delay);
    Pixy_Align();
  }
  else{
    //    Pixy_finetune();
    Serial.println("pixy has been aligned");
    motospd(0, 0, 0);
    return;
  }
}

int whichWall(int Signature){ //checks rectangle for which side and returns an element for input into dimensions array
    int temp;
    if(Signature==(bluegreen||blueyellow)){
      Serial.println('This is the x wall');
                temp=0;
    }
    else if(Signature==(pinkblue||greenpink)){
        Serial.println('This is the y wall');
            temp=1;
    }
    return temp;
}

void counterbasic(){ //basic counter sans movement
  blocks=pixy.getBlocks();
  if (blocks){
    i++;
    if (i%75==0){
      for (j=0; j<blocks; j++){
        blocks=pixy.getBlocks();
        if (pixy.blocks[j].signature==toDecimalFromOctal(greenpink)){
          counter+=1;
          cout<<"counter: "<<counter<<endl;
        }
        else if(pixy.blocks[j].signature!=toDecimalFromOctal(greenpink)){
          cout<<pixy.blocks[j].signature<<endl;
          cout<<"cant see"<<endl;
        }
        else {
          ;
        }
      }
    }
  }
}
bool align_to_zero(){
  pixy.getBlocks();
 while (pixy.blocks[0].x < 150)
 {
  PID_Superposition(60,0,0,1);
  Serial.print("Moving Right to redefine block 0 at my right");
  pixy.getBlocks();
 }
 while (pixy.blocks[0].x > 170)
   {
          Serial.print("The block is currently at: ");
          Serial.println(pixy.blocks[0].x);
          PID_Superposition(60,0,0,1);
          Serial.print("Moving Right to align with block: ");
          Serial.println(0);
          pixy.getBlocks();
    
    
  }
  return true;
}
void wallCount(){ //counts walls by moving along the wall and waiting for the centre of the block to count the marker
   // int tolerance = 10;
    pixy.getBlocks(17);
//    int blocks = pixy.getBlocks();
//     char buf[32]; 
//    sprintf(buf, "Detected %d:\n", blocks);
//      Serial.print(buf);
//      for (int j=0; j<blocks; j++)
//      {
//        sprintf(buf, "  block %d: ", j);
//        Serial.print(buf); 
//        pixy.blocks[j].print();
//      }
    //unimatMoveSensedForward(5,0,0,1);  //1 corresponds to robot number 1 and should be changed for different robots
    
    
  while(true){
    PID_Superposition(60,0,0,1);
    blocks = pixy.getBlocks();
      if ((pixy.blocks[0].signature== 19 || pixy.blocks[0].signature== 12 ||pixy.blocks[0].signature== 10 || pixy.blocks[0].signature== 11 || pixy.blocks[0].signature== 28 ||(toOctalFromDecimal(pixy.blocks[0].signature)%10 == 5)) && (pixy.blocks[0].x > (160+tolerance)))
      {

      
        bool cnt = align_to_zero();
        if(cnt)
          wallCounter+=1;
//          motospd(0,0,0);  //makes robot stop after aligning to every block
//          delay(500);
          cout<<"\nThe Count is now: "<<wallCounter<<endl;
          observedWall=pixy.blocks[0].signature;
          Serial.print("Observed wall is: ");
         Serial.println(observedWall);
         if(toOctalFromDecimal(pixy.blocks[0].signature)%10 == 5){
          motospd(0,0,0);
          endCorner=pixy.blocks[0].signature;
          cout<<"\nFound end of wall with signature "<<endCorner<<" as block "<<0<<endl;
          cout<<"\nI found "<<wallCounter<<" coordinates along the wall with signature "<<observedWall<<endl;
          dimensions[whichWall(observedWall)]=wallCounter; 
          cout<<"\nThe Dimenstion is: " <<dimensions[0]<<", "<<dimensions[2]<<endl;
          return;
         }
      }
//      else if ((toOctalFromDecimal(pixy.blocks[0].signature)%10 == 5) && (pixy.blocks[0].x > (160+tolerance)))
//      {
//        motospd(0,0,0);
//        endCorner=pixy.blocks[0].signature;
//        cout<<"\nFound end of wall with signature "<<endCorner<<" as block "<<0<<endl;
//        cout<<"\nI found "<<wallCounter<<" coordinates along the wall with signature "<<observedWall<<endl;
//        dimensions[whichWall(observedWall)]=wallCounter; 
//
//        cout<<"\nThe Dimenstion is: " <<dimensions[0]<<", "<<dimensions[2]<<endl;
//     
//        
//        return;
//      }
    if (blocks == 0){
        Serial.println("Moving right until i can see some blocks");
//        delay(10);
      }
  }
    
}

void pixyCheck(){ //checks if corner colour (usually run whenever approacing a corner to make sure pixy is facing that corner
  int i=0;
  while (i<4){
//    delay(100);
    if(pixy.blocks[0].signature==cornerColors[i]){      //loop through to check if color seen is corner color
      Pixy_Align();   //if it is a corner color, align with it and break
      startCorner=pixy.blocks[0].signature;
      break;
    }
    else if (pixy.blocks[0].signature!=cornerColors[i]){            //if its not a corner color
      Serial.println("Searching for Corner Colour \n Rotating CounterClockwise \n");
      motospd(40,40,40);                                          //rotate
      delay(rotation_update_delay);
      pixyCheck();                                     //run through pixyCheck again to see if corner color
    }
    i++;
  }
}

void printer(int startnum, int endnum){ //printing function to make sure Pixy is recieving correct octal relationals
  int octo=0;
  int deco=0;
  cout<<"i\t\toctal\t\tdecimal"<<endl;
  for (int i=startnum;i<endnum;i++){
    octo=toOctalFromDecimal(i);
    deco=toDecimalFromOctal(octo);
    cout<<i<<"\t\t"<<octo<<"\t\t"<<deco<<endl;
  }
}

string ToString(int val){ //deprecated utility function that converts integer to string (just use itoa)
    basic_stringstream<char> stream;
    stream << val;
    return stream.str();
}


int toOctalFromDecimal (int num) {// utility function for converting octadecimal to decimal (base 10)
    basic_ostringstream<char> o;
    o << std::oct << num;
    return atoi((o.str()).c_str());
}

int toDecimalFromOctal (int num) { //utility functions for converting decimal to octadecimal (base 8)
    int deci = 0, i = 0;
    while(num != 0)
    {
      if (num<10){
        deci += (num%10) * pow(8,i++);
        num/=10;
      }
      else {
        deci += (num%10) * pow(8,i++) + 1 ;
        num/=10;
      }
    }
    i = 1;
    return deci;
}

//------------------------sonar-specific functions------------------------//

bool anyCollision(int distance_cm) { // check for a collision within a certain distance
    updateSonar();                        // using the sonar in the front
    //sonarInfoDisplay();
    for (int i = 0; i < 2; i++) {
        if (cm[i] < distance_cm && cm[i] > 1) {
            return true;
        }
    }
    return false;
}

void updateSonar() { //updating sonar value
    for (int i = 0; i < 4; i++) {
        //cm[i]=sonar[i].ping_cm();
        unsigned int uS = sonar[i].ping();
        cm[i] = sonar[i].convert_cm(uS);
//        delay(100);
        //Serial.println(cm[i]);  
    }
}

void sonarInfoDisplay() { //human-readable sonar info
    Serial.println("Distance Display: ");
    for (int i = 0; i < 4; i++) {
        Serial.print("Sonar Pin ");
        Serial.print(i);
        Serial.print(" : ");
        Serial.println(cm[i]);
    }
    delay(500);
}



//-------------------------------------------------------------------------------------------------
//----------------------------------------PID Wall Adjust------------------------------------------
//-------------------------------------------------------------------------------------------------
void PID_Superposition(float vx, float vy, float ang, int robot)
{
 float adjust =  PID_Wall_Adjust();
Serial.println(adjust);
 vy = vy+adjust;
 unimatMoveSensedForward(vx, vy, ang, robot);
  
}
float PID_Wall_Adjust() {
  updateSonar();
  //sonarInfoDisplay();

  Serial.print(cm[0]);
  
  Serial.print(",");
  Input = cm[0];
  if (Input < Setpoint)
    myPID.SetControllerDirection(DIRECT);
  else
    myPID.SetControllerDirection(REVERSE);

  float gap = fabs(Setpoint - Input); //distance away from setpoint
  if (gap < 30)
  { //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();
 // Serial.println(myPID.GetDirection());
  if(myPID.GetDirection() ==0)
    return -1*Output;
  else
    return Output;

}
//-------------------------------------------------------------------------------------------------
//---------------------------------------/PID Wall Adjust------------------------------------------
//-------------------------------------------------------------------------------------------------



