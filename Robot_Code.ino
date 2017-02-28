//#include <Array.h>
#include <TimeLib.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Pixy.h>
#include <NewPing.h>
#include <Math.h>
#include <Time.h>
#include <SD.h>
#include <StandardCplusplus.h>
#include <system_configuration.h>
#include <unwind-cxx.h>
#include <utility.h>

#include <vector>
//#include <Math.h>
#include <iostream>
using namespace std;


#define X_CENTER 160L
int abs(double i) {
  return i >= 0 ? i : -i;
}
struct node 
  {
    int posx;
    int posy;
    int gcost;
    int hcost;
    bool walkable;
    bool in_path; //added in path for visual shit
    struct node* parent;
  };
vector<struct node *> path;


int boxLocation[2] = {0,0}; //Location of the robot in the room, represented as grid, with origin and back left corner. 

int wheel_pwm[] = {6,7,5}; //pins on motor drive to control speed
int wheel_direction[] = {3,4,2}; //pins to control wheel direction
double wheel_angles[] = {270, 30, 150}; //wheel angles are defined
double wheel_speed[3]; //speed of each wheel is stored in this array
double max_speed = 70; //maximum speed
double max_speedR = 40; //rotate speed

//create a new instance of Pixy
Pixy pixy;

double cm[1]; //sensed distance of sonar sensor : 2 represents array(2 sonar sensors)
int trigger[1] = {37}; //trigger pins assignment  (sends)   
int echo[1] = {36}; //echo pins (receives)

//set up the sonar sensors for NewPing Library
NewPing sonar[1] =
{
  NewPing(trigger[0], echo[0], 500), //500 represents type of sensor

};

int IR_proximity_pins[] = {A0, A1, A2, A3}; // pins for the proximity detection
int IR_proximity_state[4];              // state of IR pins





const int motor_relay = 8;   //does not want relay to change.

void setup() {     //does not return anything
  Serial.begin(9600);  // begin serial to computer

  for (int i = 0; i < 3; i++) {
    pinMode(wheel_direction[i], OUTPUT); //wheel directions are OUTPUT
  }
  for (int i = 0; i < 4; i++) { //i++ for adding
    pinMode(IR_proximity_pins[i], INPUT);
  }
  pinMode(motor_relay, OUTPUT);
  digitalWrite(motor_relay, HIGH);

  Serial3.begin(9600);    // begin serial to BASE ARDUINO
  Serial1.begin(9600);    // begin serial to SD card

  pixy.init();  // initialize pixy camera library functions
  delay(100);  //delay of .1 seconds so that you do not compile errors
  // find_red2();
  // setTime(7,0,0,1,1,1);    // (hr,min,sec,day,month,yr)


//  translateCart(0,1);
  //translateCart(1,1);
  gridInit();

}
/*
  #include <SoftwareSerial.h>

  SoftwareSerial mySerial(0, 1); // RX, TX

  void setup()
  {
  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }


  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(4800);
  mySerial.println("Hello, world?");
  }

  void loop() // run over and over
  {
  if (mySerial.available())
     Serial.write(mySerial.read());
  if (Serial.available())
     mySerial.write(Serial.read());
  }
*/
void loop() {
 //motospd(0,40,40);
//Pixy_Align();
//updateSonar();
//sonarInfoDisplay();
//Do_Thingy();
//Go_To_Object();
//wheel_speeds(40,0);
//motospd(40,40,40);
//translateCart(0,1);

//velocity_calib();
//Serial.println(velocity);

reset_path(); //first you reset path
reset_lists(); //then reset list
update_grid(boxLocation[0],boxLocation[1],18,0); //then update grid for obstacles 
 FindPath(boxLocation[0],boxLocation[1],18,0); // then find the path based on updated grid
      show_Grid(); //then show the vidualized grid
translateCart(path[path.size()-1]->posx,path[path.size()-1]->posy);
delay(5000);

}
void Do_Thingy(){
  boolean aligned = false;
  Serial.print(Am_I_Close());
  if(!aligned){
    Pixy_Align();
    aligned = true;
  }
  else if (!Am_I_Close()){
    Go_To_Object();
  }
  else{
    return;
  }
}
void Go_To_Object(){
  if(Am_I_Close()){
    motospd(0,0,0);
  }
  else{
    motospd(0,40,-40);
  }
  delay(100);
    
}
boolean Am_I_Close(){
   unsigned int uS = sonar[2].ping();
    cm[1] = sonar[1].convert_cm(uS);
    cm[2] = sonar[2].convert_cm(uS);
    Serial.print(cm[1]);
    Serial.print(cm[2]);
    if (cm[1] <=30 && cm[1] != 0 || cm[2] <= 30 && cm[2] != 0){
      return true;
    }
    else{
      return false;
    }
    
}
void Pixy_Align()
{
  Serial.println("-------------------------------------------------");
  Serial.println("-------------------------------------------------");
  get_coordinates();
  int rotation_update_delay = 10;
  int tolerance = 15;
  int blocks = pixy.getBlocks(17);
  if (pixy.blocks[0].x < (160 - tolerance) )
  {
    Serial.println("Rotating CoutnerClockwise...");
    motospd(20, 20, 20);
    delay(rotation_update_delay);
    Pixy_Align();
  }
  else if (pixy.blocks[0].x > (160 + tolerance) )
  {
    Serial.println("Rotating Clockwise...");
    motospd(-20, -20, -20);
    delay(rotation_update_delay);
    Pixy_Align();
  }
  else {
//    Pixy_finetune();
 Serial.println("pixy has been aligned");
    motospd(0, 0, 0);
    return;
  }


}

void Pixy_finetune() {

  Serial.println("------------||FINE TUNE IN PROGRESS||-----------");
  Serial.println("-------------------------------------------------");
  get_coordinates();
  int rotation_update_delay = 10;
  int tolerance = 5;
  int spd = 10;
  int blocks = pixy.getBlocks(17);
  if (pixy.blocks[0].x < (160 - tolerance) )
  {
    Serial.println("Rotating CoutnerClockwise...");
    motospd(spd, spd, spd);
    delay(rotation_update_delay);
    Pixy_finetune();
  }
  else if (pixy.blocks[0].x > (160 + tolerance) )
  {
    Serial.println("Rotating Clockwise...");
    motospd(-1 * spd, -1 * spd, -1 * spd);
    delay(rotation_update_delay);
    Pixy_finetune();
  }
  else {
    Serial.println("pixy has been aligned");
    motospd(0, 0, 0);
    return;
  }

}

void get_coordinates()
{

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
void wheel_speeds(float tot_spd, float ang){
  float results[4];
  Relative_ang(results,ang);

  float Motor_2 = abs(((((-1/sqrt(3))*tan((results[0]/180)*M_PI))-1)*(tot_spd/(1/cos((results[0]/180)*M_PI)))));
  float Motor_1 = abs(((((1/sqrt(3))*tan((results[0]/180)*M_PI))-1)*(tot_spd/(1/cos((results[0]/180)*M_PI)))));  

  float Motor_Comp = .0024*pow(results[0],3) - 0.1019*pow(results[0],2) + 2.446*results[0] + 0.8571;

int a = abs(results[1])-1;
int b = abs(results[2])-1;
int c = abs(results[3])-1;

 float motors[3];
 motors[a]= (results[1]/(abs(results[1]))) * Motor_1;
 motors[b]= (results[2]/(abs(results[2]))) * Motor_2;
 motors[c]= (results[3]/(abs(results[3]))) * Motor_Comp;

//Serial.println(results[0]);
//Serial.println(results[1]);
//Serial.println(results[2]);
//Serial.println(results[3]);
//Serial.println("------------------------------------");
//Serial.println("--------------Motors---------------");
//Serial.println("------------------------------------");
//Serial.println(motors[0]);
//Serial.println(motors[1]);
//Serial.println(motors[2]);
//Serial.println("------------------------------------");
//Serial.println("--------------Results---------------");
//Serial.println("------------------------------------");

 motospd(motors[0],motors[1],motors[2]); //At Min Speed, set compensator to 18. 
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
      ptr[1] = -3;
      ptr[2] = 2;
      ptr[3] = -1;      
       
    }
     else if(ang > 330 && ang<= 360){
      new_ang = ang;
     //new_config = {new_ang, 3,2,1};
      ptr[0] = new_ang - 360;
      ptr[1] = -3;
      ptr[2] = 2;
      ptr[3] = 1;      
       
    }
    else if (ang > 30 && ang <= 90){
     new_ang = ang - 60;
    //new_config = {new_ang, -2,-1,3};
     ptr[0] = new_ang;
      ptr[1] = 2;
      ptr[2] = -1;
      ptr[3] = -1*(new_ang/abs(new_ang))*3;
    }
    else if (ang > 90 && ang <= 150){
      new_ang = ang - 120;
     //new_config = {new_ang, 3,1,2};
      ptr[0] = new_ang;
      ptr[1] = 3;
      ptr[2] = -1;
      ptr[3] = -1*(new_ang/abs(new_ang))*2;
    }
    else if(ang > 150 && ang<= 210){
      new_ang = ang - 180;
     // new_config = {new_ang, -3,-2,1};
      ptr[0] = new_ang;
      ptr[1] = 3;
      ptr[2] = -2;
      ptr[3] = -1*(new_ang/abs(new_ang))*1;
    }
    else if(ang > 210 && ang<= 270){
      new_ang = ang - 240;
     // new_config = {new_ang, 2,1,3};
      ptr[0] = new_ang;
      ptr[1] = -2;
      ptr[2] = 1;
      ptr[3] = -1*(new_ang/abs(new_ang))*3;
    }
    else if(ang > 270 && ang<= 330){
      new_ang = ang - 300;
     // new_config = {new_ang, -3,-1,2};
      ptr[0] = new_ang;
      ptr[1] = -3;
      ptr[2] = 1;
      ptr[3] = -1*(new_ang/abs(new_ang))*2;
     }
 
 
 return 0;
          
 
}

//float Relative_ang(float ang){
// //float a,b,c,d;
// float new_config[4]={0,0,0,0};
// float *ptr[4];
//  float new_ang;
//  
//    
//     if((ang > 330 && ang<= 360) || (ang >= 0 && ang<= 30)){
//       new_ang = ang;
//      new_config = {new_ang, 3,2,1};
//        a= new_ang;
//        
//        
//     }
//     else if (ang > 30 && ang <= 90){
//      new_ang = ang + 300;
//     new_config = {new_ang, -2,-1,3};
//     }
//     else if (ang > 90 && ang <= 150){
//       new_ang = ang + 240;
//      new_config = {new_ang, 3,1,2};
//     }
//     else if(ang > 150 && ang<= 210){
//       new_ang = ang + 180;
//      new_config = {new_ang, -3,-2,1};
//     }
//     else if(ang > 210 && ang<= 270){
//       new_ang = ang + 120;
//      new_config = {new_ang, 2,1,3};
//     }
//     else if(ang > 270 && ang<= 330){
//       new_ang = ang + 60;
//      new_config = {new_ang, -3,-1,2};
//      }
//  
//  for(int i = 0; i < 4; i++){
//    ptr[i] = &new_config[i]; // assign the address of integer.
//  }
//  return **ptr;
//           
//  
//}



void Lets_Move(double ang, int tot_spd, int comp){
float Motor_2;
float Motor_1;
  
  Motor_2 = ((((-1/sqrt(3))*tan((ang/180)*M_PI))-1)*(tot_spd/(1/cos((ang/180)*M_PI))));
  Motor_1 = -1*((((1/sqrt(3))*tan((ang/180)*M_PI))-1)*(tot_spd/(1/cos((ang/180)*M_PI))));
  Serial.println("-----------------------");
  Serial.println("-----------------------");
  Serial.print("Motor 1 Speed:");
  Serial.println(Motor_1);
  Serial.print("Motor 2 Speed:");
  Serial.println(Motor_2);

 motospd(comp,Motor_1,Motor_2); //At Min Speed, set compensator to 18. 
 delay(1000);
}



//writes motor speed values to motor
// used to move in obscure directions
void motorWrite() {
  for (int i = 0; i < 3; i++) {
    if (wheel_speed[i] < 0) {
      digitalWrite(wheel_direction[i], LOW);
    } else {
      digitalWrite(wheel_direction[i], HIGH);
    }
    analogWrite(wheel_pwm[i], abs(wheel_speed[i]));
  }
}

//direct way to assign values to motors
//used for easy angles
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

//display updated wheel speed values
void speedInfoDisplay() {
  Serial.print("Speed Display: ");
  for (int i = 0; i < 3; i++) {
    Serial.print(wheel_speed[i]);
    Serial.print("|");
  }
  Serial.println(" ");
  Serial.println(" ");
  delay(10);
}

//math tool
//return the cosine value for an angle in degree
double newCos(double phi) {
  return cos(phi * M_PI / 180);
}

//return the sine value for an angle in degree
double newSin(double phi) {
  return sin(phi * M_PI / 180);
}

//return the tan value when you input y and x;
//the outcome ranges from 0 to 360 degrees
double newAtan(double x, double y) {
  double theta = atan(y / x) * 180 / M_PI;
  if (x < 0) {
    return theta + 180;
  } else if (y < 0) {
    return theta + 360;
  } else {
    return theta;
  }
}



//update sonar sensor values in terms of distance
void updateSonar() {
  for (int i = 0; i < 1; i++) {
    //cm[i]=sonar[i].ping_cm();
    unsigned int uS = sonar[i].ping();
    cm[i] = sonar[i].convert_cm(uS);
    delay(50);

//    Serial.println(cm[i]);//



  }
}

//display updated sonar sensor values
void sonarInfoDisplay() {
  Serial.println("Distance Display: ");
  for (int i = 0; i < 6; i++) {
    Serial.print("Sonar Pin ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(cm[i]);
  }
  delay(50);
}

// update IR sensor states
void updateIRprox() {
  for (int i = 0; i < 4; i++) {
    IR_proximity_state[i] = analogRead(IR_proximity_pins[i]);
    delay(50);
  }
  delay(100);
}

// print updates IR sensor values
void IRProxInfoDisplay() {
  Serial.println("Proximity Display: ");
  for (int i = 0; i < 4; i++) {
    Serial.print("IR Pin ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(IR_proximity_state[i]);
  }
  delay(100);
}
float velocity = 24.6; //velocity in cm/s. !!!!!!.!!!!!NEED TO CALIBRATE!!!!!!!!!!!
time_t timerTime; //Time after Arduino is powered on that timing starts
time_t timerReading; //Time since timing starts
void velocity_calib(){
  updateSonar();
  double start_pos = cm[0];
  Serial.println(cm[0]);//
  timerReset();
  wheel_speeds(60,3);
 //motospd(-40,0,40);
  while(cm[0] >20){
    updateSonar();
    timerRead();
  }
  motospd(0,0,0);
  velocity = start_pos/(timerReading/1000);
  Serial.println(velocity);
  return;
}
void timerReset(){
    timerTime = millis();   
    timerReading = 0;
    }

void timerRead(){
  timerReading = millis()-timerTime;
}


float Compass = 0; //Angle the robot is facing, relative to the room with 0 deg straight back into room, assumes robot is facing 0 deg. 

float boxSize = 50; //Size of the gridsquares that make up the room.



void complexMove(int goalX, int goalY, float orientation){ //Robot winds up in a specified grid, facing a specified direction, measured in grid squares and degrees.  
  translateCart(goalX, goalY); //Robot moves to a specified location
  float dTheta = orientation - Compass;
  Compass = orientation; 
  rotate(dTheta); //Robot rotates to face a specified direction
}

void translatePolar(float azimuth, float destDistCm){ //Robot moves a specified distance and direction. 
  timerReset(); //Reset the timer. 
//  timerRead();
  float duration = 1000 * (destDistCm / velocity); //time needed to travel in ms.
  Serial.print("The Robot Will Now Move for ");
  Serial.print(duration/1000);
  Serial.println("Seconds");
  wheel_speeds(60,azimuth+360); //speed in "motospd units"
  Serial.println(timerReading);
  while (timerReading < duration){
    boxUpdate(azimuth,timerReading,velocity);
    timerRead(); 
    Serial.print("The Robot is in Box: ");
    Serial.print(boxLocation[0]);
    Serial.println(boxLocation[1]);
    delay(10);
  }
  Serial.println("The Robot has Stopped");
  motospd(0,0,0); //Stop the robot

}

void translateCart(int goalX, int goalY){ //Determines the distance and direction the robot needs to move to reach a specified grid (X,Y);
  if(boxLocation[0] == goalX && boxLocation[1] == goalY)
    return;
  int destBox[2] = {goalX, goalY};
  int deltaBox[2];//How many boxes right/left, up/down the robot needs to move to reach desired location.
  float deltaCm[2]; //Distance in cm that the robot needs to move up/down/left/right. 
  for (int i = 0; i < 2; i++) //Math
  {
    deltaBox[i] = destBox[i]-boxLocation[i];
    deltaCm[i] = boxSize*deltaBox[i];
  }
  float destDistCm = sqrt(pow((boxLocation[0]-goalX)*boxSize,2)+pow((boxLocation[1]-goalY)*boxSize,2));
  Serial.println("destDistCm:");
  Serial.println(destDistCm);
  float azimuth = atan2(deltaCm[1],deltaCm[0]);
  Serial.println("The crystal Gem is:");
  Serial.println(((azimuth*180)/PI)-90);
  translatePolar(((azimuth*180)/PI)-90, destDistCm); //Once the robot determines the distance and 
  boxLocation[0] = goalX;
  boxLocation[1]= goalY;
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

void boxUpdate(float dirc, float dur, float spd){ //Take input direction, time, and speed of robot. Direction taken from some other function, time, from timer, and spd assumed to be absolute. 
//      boxLocation [0] = (int) (spd*dur*10*cos(dirc))/boxSize;
//      boxLocation [1] = (int) (spd*dur*10*sin(dirc))/boxSize;    
    }
























//look at pathfinding in the repo for comments and explanation for a* algo. Here is where i explain obstacle tracking




struct node grid[20][20];
int gridsizex = 20;

int gridsizey = 20;
void gridInit(){
  for (int i = 0; i < gridsizex; i++)
  {
    for (int j = 0; j < gridsizey; j++)
    {
      grid[i][j].posx = i;
      grid[i][j].posy = j;
      grid[i][j].walkable = true;
    }
  }
  grid[0][10].walkable = false;
  grid[0][9].walkable = false;
  grid[1][10].walkable = false;
  grid[2][9].walkable = false;


}

void show_Grid(){
  for(int i = 0; i <20; i++){
    for(int j = 0; j <20; j++){  
      if(grid[j][i].walkable && !grid[j][i].in_path){
        Serial.print("0 ");  
      }
      else if (grid[j][i].in_path)
        Serial.print("X ");
      else
        Serial.print("1 ");
      
    }
     Serial.println();
  }

}

//using structures to represent nodes for openList and closed lists
vector<struct node *> open; // nodes that need to be evaluated
vector<struct node *> closed; //already been evaluated

vector<struct node *> neighbours;




struct node * get_node(int x, int y){

  return &grid[x][y];

}

//calculate fcost
int fcost(struct node *a)
{
  return a->gcost + a->hcost;
}

int getDistance(struct node *a, struct node *b){
  int dstX = abs(a->posx - b->posx);
  int dstY = abs(a->posy - b->posy);

  return dstX + dstY;
}

bool inClosed(struct node *current)
{
  for(int i = 0; i< closed.size(); i++){
    if (current == closed[i])
      return true;
  }
  return false;
}

bool inOpen(struct node *current)
{
  for(int i = 0; i< open.size(); i++){
    if (current == open[i])
      return true;
  }
  return false;

}



void printnode(struct node* node){ // print this bitch
 Serial.print( node->posx);
  Serial.print(" ");
  Serial.println(node->posy);
}

void RetracePath(struct node *startNode, struct node *endNode){
 // cout << "[retrace]" << "\n";
  path.clear();
  struct node *currentNode = endNode;

  // TODO WTF DO WE IF THERES NO PATH BRAH
  while(currentNode != startNode){
      currentNode->in_path = true;
    path.push_back(currentNode);
    currentNode = currentNode->parent;
  Serial.print("[");
   Serial.print(currentNode->posx);
    Serial.print(", ");
     Serial.print(currentNode->posy);
      Serial.println("]");

    if (!currentNode) {
      // THERE'S NO PATH THO.
      break;


    }
  }
}
void GetNeighbours(struct node *current){
  neighbours.clear();

  for(int x = -1; x<=1;x++){ 
    for (int y = -1; y <= 1; y++){
      if(abs(x)==abs(y))
        continue;

      int checkX = current->posx + x;
      int checkY = current->posy + y;

      if(checkX >= 0 && checkX < gridsizex && checkY >= 0 && checkY<gridsizey){
        neighbours.push_back(&grid[checkX][checkY]);
      }
    }
  }
  return; 
}
void FindPath (int start_x, int start_y, int target_x, int target_y){
  struct node *startNode = get_node(start_x,start_y);
  struct node *targetNode = get_node(target_x,target_y);
  //cout << "[1]" << "\n";
  open.push_back(startNode);
  //cout << open[0]->posy << "\n";

 

  while(open.size() > 0 ){
    //cout << "[2]" << "\n";
     
    struct node *currentNode = open[0];
    int index = 0;

    for (int i =1; i<open.size(); i++){
      Serial.println("HIIHIHIHIHIHI");
      if(fcost(open[i]) < fcost(currentNode) || fcost(open[i]) == fcost(currentNode) && open[i]->hcost < currentNode->hcost){
        currentNode = open[i];
        index = i;
        
      }
      
    }
    open.erase(open.begin() + index);
    closed.push_back(currentNode);
   // printnode(currentNode);
    if(currentNode == targetNode){
      RetracePath(startNode, targetNode);
      return;
    }

    GetNeighbours(currentNode);
    for(int i = 0; i< neighbours.size(); i++){  
      if (!neighbours[i]->walkable || inClosed(neighbours[i])){
        continue;
      }
      int newMovementCostToNeighbour = currentNode->gcost + getDistance(currentNode,neighbours[i]);
      if(newMovementCostToNeighbour < neighbours[i]->gcost || !inOpen(neighbours[i])){
        neighbours[i]->gcost = newMovementCostToNeighbour;
        neighbours[i]->hcost = getDistance(neighbours[i], targetNode);
        neighbours[i]->parent = currentNode;

        if(!inOpen(neighbours[i]))
          open.push_back(neighbours[i]);
      }

    }
  }
} 


// this is where obstacle detection and avoidance starts

//dtermines if the locations detected by sonar are occupied or not
void update_grid(int x, int y, int target_x,int target_y){
  updateSonar();
  //distance of object detected
  float sonarleft = cm[0];

//adjust 150 to what you want, 150cm is what i consider accurate
  if(sonarleft <= 150 && sonarleft != 0){
    int unoccupied = sonarleft/25;
    //adjust 25 to what you want. i wanted each box in the grid to be 25x25cm. I will increase this to match the demensions of the robot
    
    //this is to remove obsticales that were there before, but were removed. marks true if no obstacle
    for (int u = 0; u <= unoccupied; u++){
      grid[x + u][y].walkable = true;
    }
    
    grid[x + unoccupied + 1][y].walkable = false;

  }
  //if there is no obsticale, mark the boxes as unoccupied.....so since 150cm is our range, range/nodesize = # of nodes 
  else{
    int unoccupied = 150/25;

      for (int u = 0; u <= unoccupied; u++){
      grid[x + u][y].walkable = true;
    }
  }
     
        

}
//resets in_path to false for all locations 
void reset_path() {
  for(int i = 0; i <20; i++){
    for(int j = 0; j <20; j++){  
     grid[i][j].in_path = false;
      }
    }
}
//erases open and closed list
void reset_lists(){
  open.clear();
  closed.clear();
}



