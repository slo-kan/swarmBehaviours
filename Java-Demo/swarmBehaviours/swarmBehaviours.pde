import java.util.ArrayList;

ArrayList<Drone> drones = new ArrayList<Drone>();
ConSteer_Behavior behavior;
int ticks = 0;

void setup() 
{
  size(1200, 800);
  
  for(int it=0; it<100; ++it)
    drones.add(new Drone(random(width), random(height),8));
  
  behavior = new ConSteer_Behavior(drones,8);
  behavior.setup();
}

void mousePressed()
{ 
  //behavior.mousePressed()
}

void draw() 
{
  background(51);
  
  if(ticks==448)
  {
    ticks = 0;
    behavior.update();
  }

  behavior.draw();
  
  stroke(255);
  strokeWeight(4);
  for(Drone drone:drones) 
  {
    //beahvior.primitive_attRep(drone);
    //behavior.advanced_attRep(drone);
    behavior.conSteer(drone);
    
    drone.update();
    drone.show();
  }
  ++ticks;
}
