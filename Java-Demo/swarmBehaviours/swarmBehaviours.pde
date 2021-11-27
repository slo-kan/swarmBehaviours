import java.util.ArrayList;

//function to calculate cosine similarity between two vectors/points
static float cosine_sim(PVector a, PVector b)
{ return a.dot(b)/(a.mag()*b.mag()); }

//global vars
ArrayList<Drone> drones = new ArrayList<Drone>();
final int DRONE_COUNT = 10;
final int DRONE_DIRECTIONS = 8;
final int MAX_TICKS = 450;
final int UPDATE_PORTION = 2;
final float PIXEL_METRIC_CONV = 0.06;
int ticks = 0;

//switch between the behaviors
//AttRep_Behavior behavior;
ConSteer_Behavior behavior;

void setup() 
{
  size(1200, 800);
  
  for(int it=0; it<DRONE_COUNT; ++it)
    //drones.add(new Drone(random(width), random(height))); //att_rep
    drones.add(new Drone(random(width), random(height), DRONE_DIRECTIONS)); //con_steer
  
  //behavior = new AttRep_Behavior(drones);
  behavior = new ConSteer_Behavior(drones, DRONE_DIRECTIONS);

  //takes number of attraction and repulsion objects
  //behavior.setup(1,1);  //for test mode
  behavior.setup(8,4); //for automatic mode
  //behavior.setup(0,0); //actually manual mode need no setup
}

// for manual mode
void mousePressed()
{ behavior.mousePressed(); }

// for pausing
void keyPressed()
{ 
  if(key==' ' && looping) noLoop(); 
  else if(key==' ' && !looping) loop();
}

// for automatic mode
void auto_update(int portion)
{
  //this block updates the attraction and repulsion objects 
  //automatically after a certain time
  if(ticks==MAX_TICKS)
  {
    ticks = 0;
    behavior.update(portion);
  }
  ++ticks;
}

void draw() 
{
  background(51);
  
  auto_update(UPDATE_PORTION); //for automatic mode
  behavior.draw();
  
  stroke(255);
  strokeWeight(4);
  for(Drone drone:drones) 
  {
    //beahvior.primitive_attRep(drone); //for att_rep
    //behavior.advanced_attRep(drone); //for att_rep
    behavior.conSteer(drone); //for con_steer
    
    drone.update();
    drone.show(height,width);
  }
  behavior.removeVisited();
}
