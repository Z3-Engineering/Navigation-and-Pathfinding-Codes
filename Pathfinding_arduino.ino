//look at pathfinding in the repo for comments and explanation for a* algo. Here is where i explain obstacle tracking

#include <StandardCplusplus.h>
#include <system_configuration.h>
#include <unwind-cxx.h>
#include <utility.h>

#include <vector>
//#include <Math.h>
#include <iostream>
using namespace std;



void setup() {
  // put your setup code here, to run once:
   Serial.begin(57600);
gridInit();
//FindPath(0,0,18,0);

 //show_Grid();
}

void loop() {
  // put your main code here, to run repeatedly:

//    gridInit();
//  FindPath(0,0,10,10);
 // cout << "[""]" << "\n";

// updateSonar();
reset_path();
reset_lists();
update_grid(0,0,18,0);
 FindPath(0,0,18,0);
      show_Grid();

delay(5000);
}







struct node 
  {
    int posx;
    int posy;
    int gcost;
    int hcost;
    bool walkable;
    bool in_path;
    struct node* parent;
  };

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


int abs(int i) {
  return i >= 0 ? i : -i;
}

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
  vector<struct node *> path;
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

#include <NewPing.h>
double cm[1]; //sensed distance of sonar sensor : 2 represents array(2 sonar sensors)
int trigger[1] = {40}; //trigger pins assignment  (sends)   
int echo[1] = {41}; //echo pins (receives)

NewPing sonar[1] =
{
  NewPing(trigger[0], echo[0], 500), //500 represents type of sensor
};

void updateSonar() {
  for (int i = 0; i < 1; i++) {
    //cm[i]=sonar[i].ping_cm();
    unsigned int uS = sonar[i].ping();
    cm[i] = sonar[i].convert_cm(uS);
    delay(50);

    Serial.println(cm[i]);



  }
}
void update_grid(int x, int y, int target_x,int target_y){
  updateSonar();
  float sonarleft = cm[0];


  if(sonarleft <= 150 && sonarleft != 0){
    int unoccupied = sonarleft/25;
    
    for (int u = 0; u <= unoccupied; u++){
      grid[x + u][y].walkable = true;
    }
    
    grid[x + unoccupied + 1][y].walkable = false;

  }
  else{
    int unoccupied = 150/25;

      for (int u = 0; u <= unoccupied; u++){
      grid[x + u][y].walkable = true;
    }
  }
     
        

}

void reset_path() {
  for(int i = 0; i <20; i++){
    for(int j = 0; j <20; j++){  
     grid[i][j].in_path = false;
      }
    }
}

void reset_lists(){
  open.clear();
  closed.clear();
}
