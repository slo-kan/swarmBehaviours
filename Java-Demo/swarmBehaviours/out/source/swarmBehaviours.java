import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.util.ArrayList; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class swarmBehaviours extends PApplet {



//function to calculate cosine similarity between two vectors/points
public static float cosine_sim(PVector a, PVector b)
{ return a.dot(b)/(a.mag()*b.mag()); }

//global vars
ArrayList<Drone> drones = new ArrayList<Drone>();
final int DRONE_COUNT = 10;
final int DRONE_DIRECTIONS = 8;
final int MAX_TICKS = 450;
final int UPDATE_PORTION = 2;
int ticks = 0;

//switch between the behaviors
//AttRep_Behavior behavior;
ConSteer_Behavior behavior;

public void setup() 
{
  
  
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
public void mousePressed()
{ behavior.mousePressed(); }

// for pausing
public void keyPressed()
{ 
  if(key==' ' && looping) noLoop(); 
  else if(key==' ' && !looping) loop();
}

// for automatic mode
public void auto_update(int portion)
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

public void draw() 
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
}
class AttRep_Behavior
{
  //global vars
  ArrayList<Drone> drones;
  ArrayList<PVector> attractors = new ArrayList<PVector>();
  ArrayList<PVector> repellPoints = new ArrayList<PVector>();
  float comfy_dist = 48;
  float perlimiter = 24;

  //constructor
  AttRep_Behavior(ArrayList<Drone> drones)
  { this.drones = drones; }

  //for initial setup of certain number of random attractors and repell points
  public void setup(int ATTs,int REPs)
  {
    for(int it = 0; it < ATTs; ++it)
        this.attractors.add(new PVector(random(width-4), random(height-4)));
    
    for(int it = 0; it < REPs; ++it)
      this.repellPoints.add(new PVector(random(width-4), random(height-4)));
  }

  //manually add attraction or repulsion points
  public void mousePressed()
  { 
    if(mouseButton==LEFT && !keyPressed) this.attractors.add(new PVector(mouseX, mouseY)); 
    else if(mouseButton==RIGHT && !keyPressed) this.repellPoints.add(new PVector(mouseX, mouseY));
    else if(mouseButton==LEFT && key == DELETE && !this.attractors.isEmpty()) this.attractors.remove(this.attractors.size()-1); 
    else if(mouseButton==RIGHT && key == DELETE && !this.repellPoints.isEmpty()) this.repellPoints.remove(this.repellPoints.size()-1); 
  }
  
  //automatically manipulate points
  public void update(int portion)
  {
    for(int it = 0; it < max(this.attractors.size()/portion,1); ++it){
      if(!this.attractors.isEmpty()) this.attractors.remove(0);
      this.attractors.add(new PVector(random(width-4), random(height-4)));
    }
    for(int it = 0; it < max(this.repellPoints.size()/portion,1); ++it){
      if(!this.repellPoints.isEmpty()) this.repellPoints.remove(0);
      this.repellPoints.add(new PVector(random(width-4), random(height-4)));
    }
  }

  //removes visited attraction_points
  public void removeVisited()
  {
    ArrayList<PVector> attractors_updated = new ArrayList<PVector>();
    for(PVector attractor: this.attractors)
    {
      PVector current = attractor.copy();
      attractors_updated.add(current);
      for(Drone drone:this.drones)
        if(floor(drone.pos.x) == floor(attractor.x) && 
           floor(drone.pos.y) == floor(attractor.y) )
          { attractors_updated.remove(current); break;}
    }
    this.attractors = attractors_updated;
  }

  //draw behavior specific parts
  public void draw()
  {
    removeVisited();

    stroke(0, 255, 0);
    strokeWeight(8);
    for (PVector attractor:this.attractors)
      point(attractor.x, attractor.y);
      
    stroke(255, 0, 0);
    strokeWeight(8);
    for (PVector repellPoint:this.repellPoints)
      point(repellPoint.x, repellPoint.y);
  }

  //behavior specific function for each drone
  public void primitive_attRep(Drone drone)
  {
    for(PVector attractor:this.attractors)
      drone.primitive_attraction(attractor,comfy_dist,10);

    for(PVector repellPoint:this.repellPoints)
      drone.primitive_repulsion(repellPoint,comfy_dist,20);

    for(Drone other:this.drones)
      if(other!=drone) drone.primitive_attraction(other.pos,2*comfy_dist,1);
    for(Drone other:this.drones)
      if(other!=drone) drone.primitive_repulsion(other.pos,1.2f*comfy_dist,5);  
  }

  //behavior specific function for each drone
  public void advanced_attRep(Drone drone)
  {
    for(PVector attractor:this.attractors)
      drone.linear_attraction(attractor,44); 

    for(PVector repellPoint:this.repellPoints)
      drone.simpleExponential_repulsion(repellPoint,perlimiter,44);

    for(Drone other:this.drones)
      if(other!=drone) drone.comfy_attraction(other.pos,comfy_dist,3);
    for(Drone other:this.drones)
      if(other!=drone) drone.complexExponential_repulsion(other.pos,perlimiter,3,10);
  }
};
class ConSteer_Behavior
{
    ArrayList<Drone> drones;
    ArrayList<PVector> goals = new ArrayList<PVector>();
    ArrayList<PVector> dangers = new ArrayList<PVector>();
    ArrayList<PVector> RAY_DIRS = new ArrayList<PVector>();
    boolean DEBUG = false;
    float SECTOR_COS_SIM;

    //constructor
    ConSteer_Behavior(ArrayList<Drone> drones, int directions)
    { 
        this.drones = drones; 
        for(int it=0; it<directions; ++it)
        {
          float angle = it * TWO_PI/directions;
          this.RAY_DIRS.add(PVector.fromAngle(angle));
        }
        this.SECTOR_COS_SIM = cos(PI/directions);
    }

    //for initial setup of certain number of random goals and dangers
    public void setup(int numGoals, int numDangers)
    {
      for(int it = 0; it < numGoals; ++it)
        this.goals.add(new PVector(random(width-4), random(height-4)));
    
      for(int it = 0; it < numDangers; ++it)
        this.dangers.add(new PVector(random(width-4), random(height-4)));
    }

    //manually add or remove goal or danger points
    public void mousePressed()
    { 
      if(mouseButton==LEFT && !keyPressed) this.goals.add(new PVector(mouseX, mouseY)); 
      else if(mouseButton==RIGHT && !keyPressed) this.dangers.add(new PVector(mouseX, mouseY));
      else if(mouseButton==LEFT && key == DELETE && !this.goals.isEmpty()) this.goals.remove(this.goals.size()-1);
      else if(mouseButton==RIGHT && key == DELETE && !this.dangers.isEmpty()) this.dangers.remove(this.dangers.size()-1);
    }
    
    //automatically manipulate points
    public void update(int portion)
    {
      for(int it = 0; it < max(this.goals.size()/portion,1); ++it){
        if(!this.goals.isEmpty()) this.goals.remove(0);
        this.goals.add(new PVector(random(width-4), random(height-4)));
      }
      for(int it = 0; it < max(this.dangers.size()/portion,1); ++it){
        if(!this.dangers.isEmpty()) this.dangers.remove(0);
        this.dangers.add(new PVector(random(width-4), random(height-4)));
      }
    }

    //removes visited attraction_points
    public void removeVisited()
    {
      ArrayList<PVector> goals_updated = new ArrayList<PVector>();
      for(PVector goal: this.goals)
      {
        PVector current = goal.copy();
        goals_updated.add(current);
        for(Drone drone:this.drones)
          if(floor(drone.pos.x) == floor(goal.x) && 
             floor(drone.pos.y) == floor(goal.y) )
            { goals_updated.remove(current); break;}
      }
      this.goals = goals_updated;
    }

    //draw behavior specific parts
    public void draw()
    {
      removeVisited();

      stroke(0, 255, 0);
      strokeWeight(8);
      for (PVector goal:this.goals)
        point(goal.x, goal.y);

      stroke(255, 0, 0);
      strokeWeight(8);
      for (PVector danger:this.dangers)
        point(danger.x, danger.y);

      
      if(this.DEBUG)
      {
        for(Drone drone: this.drones)
        {
          strokeWeight(2);
          stroke(212, 243, 107);
          line(drone.pos.x,drone.pos.y,drone.pos.x+this.RAY_DIRS.get(0).x*10,drone.pos.y+this.RAY_DIRS.get(0).y*10);
          stroke(112, 243, 107);
          line(drone.pos.x,drone.pos.y,drone.pos.x+this.RAY_DIRS.get(1).x*10,drone.pos.y+this.RAY_DIRS.get(1).y*10);
          stroke(107, 243, 212);
          line(drone.pos.x,drone.pos.y,drone.pos.x+this.RAY_DIRS.get(2).x*10,drone.pos.y+this.RAY_DIRS.get(2).y*10);
          stroke(107, 212, 243);
          line(drone.pos.x,drone.pos.y,drone.pos.x+this.RAY_DIRS.get(3).x*10,drone.pos.y+this.RAY_DIRS.get(3).y*10);
          stroke(107, 121, 243);
          line(drone.pos.x,drone.pos.y,drone.pos.x+this.RAY_DIRS.get(4).x*10,drone.pos.y+this.RAY_DIRS.get(4).y*10);
          stroke(212, 107, 243);
          line(drone.pos.x,drone.pos.y,drone.pos.x+this.RAY_DIRS.get(5).x*10,drone.pos.y+this.RAY_DIRS.get(5).y*10);
          stroke(243, 107, 212);
          line(drone.pos.x,drone.pos.y,drone.pos.x+this.RAY_DIRS.get(6).x*10,drone.pos.y+this.RAY_DIRS.get(6).y*10);
          stroke(243, 212, 107);
          line(drone.pos.x,drone.pos.y,drone.pos.x+this.RAY_DIRS.get(7).x*10,drone.pos.y+this.RAY_DIRS.get(7).y*10);
        }

        strokeWeight(2);
        stroke(212, 243, 107);
        line(width/2,height/2,width/2+this.RAY_DIRS.get(0).x*50,height/2+this.RAY_DIRS.get(0).y*50);
        stroke(112, 243, 107);
        line(width/2,height/2,width/2+this.RAY_DIRS.get(1).x*50,height/2+this.RAY_DIRS.get(1).y*50);
        stroke(107, 243, 212);
        line(width/2,height/2,width/2+this.RAY_DIRS.get(2).x*50,height/2+this.RAY_DIRS.get(2).y*50);
        stroke(107, 212, 243);
        line(width/2,height/2,width/2+this.RAY_DIRS.get(3).x*50,height/2+this.RAY_DIRS.get(3).y*50);
        stroke(107, 121, 243);
        line(width/2,height/2,width/2+this.RAY_DIRS.get(4).x*50,height/2+this.RAY_DIRS.get(4).y*50);
        stroke(212, 107, 243);
        line(width/2,height/2,width/2+this.RAY_DIRS.get(5).x*50,height/2+this.RAY_DIRS.get(5).y*50);
        stroke(243, 107, 212);
        line(width/2,height/2,width/2+this.RAY_DIRS.get(6).x*50,height/2+this.RAY_DIRS.get(6).y*50);
        stroke(243, 212, 107);
        line(width/2,height/2,width/2+this.RAY_DIRS.get(7).x*50,height/2+this.RAY_DIRS.get(7).y*50);
      }
    } 

    //behavior specific function for each drone
    public void conSteer(Drone drone)
    {
      //create all direction segements of each map
      for(int idx=0; idx<this.RAY_DIRS.size(); ++idx)
      {
        ArrayList<PVector> intrests = new ArrayList<PVector>();
        ArrayList<PVector> members = new ArrayList<PVector>();
        ArrayList<PVector> noFlyZones = new ArrayList<PVector>();

        for(PVector goal: this.goals)
          if(cosine_sim(this.RAY_DIRS.get(idx), PVector.sub(goal,drone.pos)) >= this.SECTOR_COS_SIM) 
            intrests.add(goal);
        for(Drone other: this.drones)
          if(other != drone && cosine_sim(this.RAY_DIRS.get(idx), PVector.sub(other.pos,drone.pos)) >= this.SECTOR_COS_SIM) 
            members.add(other.pos.copy()); 
        for(PVector danger: this.dangers)
          if(cosine_sim(this.RAY_DIRS.get(idx), PVector.sub(danger,drone.pos)) >= this.SECTOR_COS_SIM) 
            noFlyZones.add(danger); 

        drone.create_context_segment(idx, intrests, noFlyZones, members);
      }

      //evaluate context steering behavior
      drone.context_steering(this.RAY_DIRS,this.SECTOR_COS_SIM);
    }

}
class Drone {
  //main vars of any drone
  ArrayList<PVector> prev;
  PVector pos;
  PVector vel;
  PVector acc;
  final float G = 1.98f;
  final float MAX_SPEED = 3;
  final static boolean DEBUG = false;
  
  //context steering specific globals
  ArrayList<ArrayList<PVector>> contextMaps = new ArrayList<ArrayList<PVector>>();
  ArrayList<PVector> currentForces = new ArrayList<PVector>();
  PVector prevForce = new PVector();
  final int DIRECTIONS, VISUAL_SCALE = 5;
  final int GOAL_VECTORS = 0;
  final int DANGER_VECTORS = 1;
  final int ATT_MEMBER_VECTORS = 2;
  final int REP_MEMBER_VECTORS = 3;

  //constructors
  Drone(float x, float y) 
  { this(x,y,8); }

  Drone(float x, float y, int directions) 
  {
    this.pos = new PVector(x, y);
    this.prev = new ArrayList<PVector>();
    this.prev.add(new PVector(x, y));

    //set random initial velocity
    this.vel = PVector.random2D().setMag(random(MAX_SPEED/10, MAX_SPEED/5));
    //this.vel = new PVector(); //no initial velocity
    if(DEBUG) System.out.println("Init-Drone-Velocity = ("+this.vel.x+","+this.vel.y+")");
    this.acc = new PVector();

    //context steering specific initialization
    this.DIRECTIONS = directions;
    ArrayList<PVector> goals = new ArrayList<PVector>();
    ArrayList<PVector> att_members = new ArrayList<PVector>();
    ArrayList<PVector> rep_members = new ArrayList<PVector>();
    ArrayList<PVector> dangers = new ArrayList<PVector>();

    for(int it=0; it<this.DIRECTIONS; ++it)
    {
      goals.add(new PVector());
      att_members.add(new PVector());
      rep_members.add(new PVector());
      dangers.add(new PVector());
    }

    this.contextMaps.add(goals);
    this.contextMaps.add(dangers);
    this.contextMaps.add(att_members);
    this.contextMaps.add(rep_members);
  }

  //update a drones position based on simple physics(velocity)  
  //update velocity based on current acceleartion and reset the acceleartion
  public void update() 
  {
    if(DEBUG) System.out.println("Drone-Acceleration = ("+this.acc.x+","+this.acc.y+")");
    this.vel.add(this.acc);
    this.vel.limit(MAX_SPEED);
    if(DEBUG) System.out.println("Drone-Velocity = ("+this.vel.x+","+this.vel.y+")");
    this.pos.add(this.vel);
    this.acc.mult(0);
  }

  public void borders(int h, int w) {
    while(ceil(this.pos.x)<0) this.pos.x = this.pos.x+w;
    while(ceil(this.pos.y)<0) this.pos.y = this.pos.y+h;
    while(floor(this.pos.x)>w) this.pos.x = this.pos.x-w;
    while(floor(this.pos.y)>h) this.pos.y = this.pos.y-h;
  }

  //processing specific code to draw drones
  public void show(int h, int w) 
  {
    borders(h,w);

    stroke(255, 255);
    strokeWeight(6);
    point(this.pos.x, this.pos.y);

    if(DEBUG) System.out.println("Drone-Position = ("+this.pos.x+","+this.pos.y+")");
    
    if(DEBUG) 
    {
      stroke(0, 255, 0);
      for(PVector force: this.currentForces)
      {
        strokeWeight(2);
        line(this.pos.x,this.pos.y,this.pos.x+force.x*VISUAL_SCALE,this.pos.y+force.y*VISUAL_SCALE);
        strokeWeight(4);
        point(this.pos.x+force.x*VISUAL_SCALE,this.pos.y+force.y*VISUAL_SCALE);
      }
    }

    if(!DEBUG)
    {
      stroke(255, 175, 25);
      strokeWeight(2);
      line(this.pos.x,this.pos.y,this.pos.x+this.prevForce.x*VISUAL_SCALE,this.pos.y+this.prevForce.y*VISUAL_SCALE);
      strokeWeight(4);
      point(this.pos.x+this.prevForce.x*VISUAL_SCALE,this.pos.y+this.prevForce.y*VISUAL_SCALE);
    }

    if(!DEBUG) 
    {
      stroke(255, 0, 0);
      for(PVector danger: this.contextMaps.get(DANGER_VECTORS))
      {
        strokeWeight(2);
        line(this.pos.x,this.pos.y,this.pos.x+danger.x*VISUAL_SCALE,this.pos.y+danger.y*VISUAL_SCALE);
        strokeWeight(4);
        point(this.pos.x+danger.x*VISUAL_SCALE,this.pos.y+danger.y*VISUAL_SCALE);
      }

      stroke(0, 255, 255);
      for(PVector member: this.contextMaps.get(ATT_MEMBER_VECTORS))
      {
        strokeWeight(2);
        line(this.pos.x,this.pos.y,this.pos.x+member.x*VISUAL_SCALE,this.pos.y+member.y*VISUAL_SCALE);
        strokeWeight(4);
        point(this.pos.x+member.x*VISUAL_SCALE,this.pos.y+member.y*VISUAL_SCALE);
      }

      stroke(255, 0, 255);
      for(PVector member: this.contextMaps.get(REP_MEMBER_VECTORS))
      {
        strokeWeight(2);
        line(this.pos.x,this.pos.y,this.pos.x+member.x*VISUAL_SCALE,this.pos.y+member.y*VISUAL_SCALE);
        strokeWeight(4);
        point(this.pos.x+member.x*VISUAL_SCALE,this.pos.y+member.y*VISUAL_SCALE);
      }

      stroke(0, 255, 0);
      for(PVector goal: this.contextMaps.get(GOAL_VECTORS))
      {
        strokeWeight(2);
        line(this.pos.x,this.pos.y,this.pos.x+goal.x*VISUAL_SCALE,this.pos.y+goal.y*VISUAL_SCALE);
        strokeWeight(4);
        point(this.pos.x+goal.x*VISUAL_SCALE,this.pos.y+goal.y*VISUAL_SCALE);
      }
    }

    strokeWeight(2);
    stroke(255, 25);
    line(this.pos.x, this.pos.y, 
    this.prev.get(this.prev.size()-1).x, this.prev.get(this.prev.size()-1).y);
    
    stroke(255, 5);
    for(int it=this.prev.size()-1; it>0; --it)
      line(this.prev.get(it).x,this.prev.get(it).y,
      this.prev.get(it-1).x,this.prev.get(it-1).y);
    
    this.prev.add(new PVector(this.pos.x,this.pos.y));
    if(this.prev.size()>100) this.prev.remove(0);
  }

  //context steering 
  //update direction specific segments of all context maps
  public void create_context_segment(int dir, ArrayList<PVector> goals, ArrayList<PVector> dangers, ArrayList<PVector> members)
  {
    PVector strongest_force;
    strongest_force = new PVector();
    for(PVector goal:goals) 
      if(strongest_force.mag() < invGausain_Attraction(goal).mag()) 
        strongest_force = invGausain_Attraction(goal);
    this.contextMaps.get(GOAL_VECTORS).set(dir,strongest_force.copy());
    strongest_force = new PVector();
    for(PVector danger:dangers) 
      if(strongest_force.mag() < limExp_Repulsion(danger).mag()) 
        strongest_force = limExp_Repulsion(danger);
    this.contextMaps.get(DANGER_VECTORS).set(dir,strongest_force.copy());

    PVector strongest_att = new PVector();
    PVector strongest_rep = new PVector();
    for(PVector member:members) 
    {
      PVector force;
      force = gausain_Attraction(member);
      if(strongest_att.mag()<force.mag()) strongest_att = force.copy();
      force = limExp_Repulsion(member);
      if(strongest_rep.mag()<force.mag()) strongest_rep = force.copy();
    }
    this.contextMaps.get(ATT_MEMBER_VECTORS).set(dir,strongest_att.copy());
    this.contextMaps.get(REP_MEMBER_VECTORS).set(dir,strongest_rep.copy());
  }

  //context steering
  //update acceleration based on context steering behavior
  public void context_steering(ArrayList<PVector> rayDirs, float sectorCosSim)
  { 
    //calculate total force per direction
    ArrayList<PVector> forces = new ArrayList<PVector>();
    ArrayList<Boolean> mask = new ArrayList<Boolean>();
    for(int idx=0; idx<this.DIRECTIONS; ++idx)
    {
      PVector force = new PVector();
      force.add(this.contextMaps.get(GOAL_VECTORS).get(idx).mult(1));
      force.add(this.contextMaps.get(ATT_MEMBER_VECTORS).get(idx).mult(1));
      force.add(this.contextMaps.get(REP_MEMBER_VECTORS).get(idx).mult(1));
      force.add(this.contextMaps.get(DANGER_VECTORS).get(idx).mult(1));
      
      //danger based mask
      if(cosine_sim(rayDirs.get(idx),force) < sectorCosSim) mask.add(false);
      else mask.add(true);
      
      //less likely to switch directions
      float cosSim = cosine_sim(rayDirs.get(idx),this.prevForce);
      if(cosSim < sectorCosSim) force.mult(map(cosSim,-1.0f,sectorCosSim,0.01f,0.75f));
      
      forces.add(force.copy());
    }
    this.currentForces = forces;

    //select strongest force as main force direction
    PVector main_force = new PVector();
    int maxIdx = 0;
    while((maxIdx < forces.size()) && !mask.get(maxIdx)) ++maxIdx;
    for(int idx=maxIdx+1; idx<forces.size(); ++idx) 
      if(mask.get(idx) && (forces.get(idx).mag() > forces.get(maxIdx).mag())) 
        maxIdx = idx;
    if(maxIdx < forces.size()) main_force = PVector.mult(rayDirs.get(maxIdx),MAX_SPEED);
    if(DEBUG) System.out.println("Main Direction: "+maxIdx);

    PVector total_force = main_force.copy();
    if(maxIdx < forces.size() && forces.get(maxIdx).mag()>3) 
    {
      //interpolate between main and stronger neighbor force
      int leftIdx = (maxIdx-1 + this.DIRECTIONS)%this.DIRECTIONS;
      int rightIdx = (maxIdx+1 + this.DIRECTIONS)%this.DIRECTIONS;
      float leftMag = forces.get(leftIdx).mag(); 
      float rightMag = forces.get(rightIdx).mag();
      int neighborIdx = (leftMag<rightMag && mask.get(rightIdx))?rightIdx:(mask.get(leftIdx))?leftIdx:-1;
      if(DEBUG) System.out.println("Neighbor Direction:"+neighborIdx);
      if(neighborIdx>=0)
      {
        float magnitude = forces.get(neighborIdx).mag()/forces.get(maxIdx).mag();
        PVector secondary_force = PVector.mult(rayDirs.get(neighborIdx),MAX_SPEED*magnitude);
        total_force.add(secondary_force);
      }
    }

    //determine force output
    total_force.setMag(MAX_SPEED);  
    this.prevForce = total_force.copy();
    this.acc.add(total_force);
  }

  //context steering
  //calculate repulsion force from DANGER_VECTORS
  public PVector limExp_Repulsion(PVector target)
  {
    final float LIMIT = 40;
    final float SIGMA = 3.85f;

    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength;
    d = min(d, LIMIT);
    if (d<11) strength = G * (0.5f*d - LIMIT) / SIGMA; //linear_rep_limit_close
    else strength = -1*exp(-1*(G * ((LIMIT/8)*log(d) - (LIMIT/2-1.5f))/SIGMA)); //exp_rep_mid_far
    force = force.normalize();
    force = force.mult(strength);
    return force.copy();
  }

  //context steering
  //calculate attraction and repulsion forces from other drones
  public PVector gausain_Attraction(PVector target)
  {
    final float LIMIT = 40;
    final float SIGMA = 4.98f;
    final float GAMMA = 6;
    final float MEAN = 20;

    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = min(d, LIMIT);
    float strength;
    //strength = G*d-10; //simple linear
    if(d<LIMIT-10) strength = (G*(d-MEAN)-sq(d-MEAN))/(LIMIT-10)+SIGMA; //close_mid_range
    else strength = (-1)*(d/(G*G*G))+GAMMA; //low_att_far
    force = force.normalize();
    force = force.mult(strength);
    return force.copy();
  }
  
  //context steering
  //calculate attraction forces to GOAL_VECTORS
  public PVector invGausain_Attraction(PVector target)
  {
    final float LIMIT = 30.0f;
    final float SIGMA = 5.0f;
    final float MEAN = LIMIT/2;

    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = min(d, LIMIT);
  
    float tanh_inv_gauss = (float)(Math.tanh(exp(sq(d-MEAN)/(2*sq(SIGMA)))/(SIGMA*sqrt(TWO_PI)))*10-0.25f);
    float quad_eq = sq(d)-(MEAN*d)-LIMIT;
    float strength = (MEAN/10)*((G * tanh_inv_gauss) / (LIMIT/10) - quad_eq / (4*LIMIT))-(MEAN/100);
    
    //float strength = (G/(-3))*d+20; //simple linear
    force.normalize();
    force.mult(strength);
    return force.copy();
  }

  //att_rep behavior
  //calculate attraction force
  public void primitive_attraction(PVector target, float perlimiter, int multiplier) 
  {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = constrain(d, 1, 25);
    if(d > perlimiter)
    {
      float strength = (G*multiplier) / (d * d);
      force.setMag(strength); 
      this.acc.add(force);
    }
  }
  
  //att_rep behavior
  //calculate repulsion force
  public void primitive_repulsion(PVector target, float perlimiter, int multiplier)
  {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = constrain(d, 1, 25);
    if(d<perlimiter)
    {
      float strength = (G*multiplier) / (d * d);
      force.setMag(strength);
      this.acc.sub(force);
    }
  }
  
  //att_rep behavior
  //calculate linear attraction force to attrection points
  public void linear_attraction(PVector target, int multiplier) 
  {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = max(d,1);
    float strength = G*multiplier/d;
    force.setMag(strength); 
    this.acc.add(force);
  }
  
  //att_rep behavior
  //calculate exponential repulsion force to other drones
  public void complexExponential_repulsion(PVector target, float perlimiter, int a, int b)
  {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float c = (perlimiter*perlimiter) * (float)Math.log(b/a);
    float strength = (G*b) * (float)Math.exp(-(d*d)/c);
    force.setMag(strength);
    this.acc.sub(force);
  }
  
  //att_rep behavior
  //calculate comfortable attraction force to other drones
  public void comfy_attraction(PVector target, float perlimiter, int multiplier) 
  {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = (G*multiplier) * (d-perlimiter)/max(d,0.01f); 
    force.setMag(strength); 
    this.acc.add(force);
  }
  
  //att_rep behavior
  //calculate exponential repulsion force to repell points
  public void simpleExponential_repulsion(PVector target, float perlimiter, int multiplier)
  {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = (G*multiplier) * (float)Math.exp(-(d*d)/(2*perlimiter*perlimiter));
    force.setMag(strength);
    this.acc.sub(force);
  }
};
  public void settings() {  size(1200, 800); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "swarmBehaviours" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
