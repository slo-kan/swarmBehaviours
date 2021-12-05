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
final int DRONE_COUNT = 8;
final int DRONE_DIRECTIONS = 8;
final int MAX_TICKS = 450;
final int UPDATE_PORTION = 2;
final float PIXEL_METRIC_CONV = 0.06f;
int ticks = 0;

//switch between the behaviors
//AttRep_Behavior behavior;
ConSteer_Behavior behavior;

public void setup() 
{
  
  
  for(int it=0; it<DRONE_COUNT; ++it)
    //drones.add(new Drone(random(width), random(height))); //att_rep
    drones.add(new Drone(random(width), random(height), DRONE_DIRECTIONS)); //con_steer
  
  //behavior = new AttRep_Behavior(drones, width, height);
  behavior = new ConSteer_Behavior(drones, DRONE_DIRECTIONS, width, height);

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
  behavior.removeVisited();
}
class AttRep_Behavior
{
  //global vars
  ArrayList<Drone> drones;
  ArrayList<PVector> attractors = new ArrayList<PVector>();
  ArrayList<PVector> repellPoints = new ArrayList<PVector>();
  ArrayList<PVector> border_points = new ArrayList<PVector>();
  final int ATT_MULT = 128;
  final int REP_MULT = 44;
  final int DRONE_ATT_MULT = 2;
  final int DRONE_REP_MULT = 20;
  final float COMFY_DIST = 30;
  final float PERLIMITER = 25;
  final float BORDER_PERLIMITER = 5;
  final float DRONE_PERLIMITER = 10;
  

  //constructor
  AttRep_Behavior(ArrayList<Drone> drones, int w, int h)
  { 
    this.drones = drones; 

    for(int x=0; x<w; ++x)
    {
      border_points.add(new PVector(x,0));
      border_points.add(new PVector(x,h));
    }
    for(int y=0; y<h; ++y)
    {
      border_points.add(new PVector(0,y));
      border_points.add(new PVector(w,y));
    }
  }

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
      boolean add = true;
      for(Drone drone:this.drones)
        if(abs(drone.pos.x - attractor.x)<2 && 
           abs(drone.pos.y - attractor.y)<2 )
        { add = false; break; }
      if(add) attractors_updated.add(attractor.copy());
    }
    this.attractors.clear();
    this.attractors.addAll(attractors_updated);
  }

  //draw behavior specific parts
  public void draw()
  {
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
      drone.primitive_attraction(attractor,COMFY_DIST,10);

    for(PVector repellPoint:this.repellPoints)
      drone.primitive_repulsion(repellPoint,COMFY_DIST,20);

    for(Drone other:this.drones)
      if(other!=drone) drone.primitive_attraction(other.pos,2*COMFY_DIST,1);
    for(Drone other:this.drones)
      if(other!=drone) drone.primitive_repulsion(other.pos,1.2f*COMFY_DIST,5);  
  }

  //behavior specific function for each drone
  public void advanced_attRep(Drone drone)
  {
    for(PVector attractor:this.attractors)
      drone.linear_attraction(attractor,ATT_MULT); 

    for(PVector repellPoint:this.repellPoints)
      drone.simpleExponential_repulsion(repellPoint,PERLIMITER,REP_MULT);
    
    for(PVector repellPoint:this.border_points)
      drone.simpleExponential_repulsion(repellPoint,BORDER_PERLIMITER,REP_MULT);

    for(Drone other:this.drones)
      if(other!=drone) drone.comfy_attraction(other.pos,COMFY_DIST,DRONE_ATT_MULT);
    for(Drone other:this.drones)
      if(other!=drone) drone.complexExponential_repulsion(other.pos,DRONE_PERLIMITER,DRONE_ATT_MULT,DRONE_REP_MULT);
  }
};
class ConSteer_Behavior
{
    ArrayList<Drone> drones;
    ArrayList<PVector> goals = new ArrayList<PVector>();
    ArrayList<PVector> dangers = new ArrayList<PVector>();
    ArrayList<PVector> border_points = new ArrayList<PVector>();
    ArrayList<PVector> RAY_DIRS = new ArrayList<PVector>();
    boolean DEBUG = false;
    float SECTOR_COS_SIM;
    final float BORDER_TO_CLOSE = 3;
    final float MEMBER_TO_CLOSE = 4;
    final float SWARM_DIST = 12;

    final float GOAL_LIMIT = 30;
    final float DANGER_LIMIT = 40;
    final float DANGER_SIGMA = 3.85f;
    final float DANGER_GAMMA = 18.5f;
    final float DANGER_ALPHA = 5;
    final float MEMBER_REP_LIMIT = 30; 
    final float MEMBER_ATT_LIMIT = 40;
    final float MEMBER_ATT_CUT_OFF = 18.9f;
    final float MEMBER_ATT_SIGMA = 5;
    final float MEMBER_ATT_GAMMA = 4.25f;
    final float MEMBER_ATT_MEAN = 15;

    final float INV_GAUSSIAN_LIMIT = 30;
    final float INV_GAUSSIAN_SIGMA = 5;
    final float INV_GAUSSIAN_MEAN = 15;

    //constructor
    ConSteer_Behavior(ArrayList<Drone> drones, int directions, int w, int h)
    { 
        this.drones = drones; 
        for(int it=0; it<directions; ++it)
        {
          float angle = it * TWO_PI/directions;
          this.RAY_DIRS.add(PVector.fromAngle(angle));
        }
        this.SECTOR_COS_SIM = cos(PI/directions);

        for(int x=0; x<w; ++x)
        {
          border_points.add(new PVector(x,0));
          border_points.add(new PVector(x,h));
        }
        for(int y=0; y<h; ++y)
        {
          border_points.add(new PVector(0,y));
          border_points.add(new PVector(w,y));
        }
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
        boolean add = true;
        for(Drone drone:this.drones)
          if(abs(drone.pos.x - goal.x)<2 && 
             abs(drone.pos.y - goal.y)<2 )
          { add = false; break; }
        if(add) goals_updated.add(goal.copy());
      }
      this.goals.clear();
      this.goals.addAll(goals_updated);
    }

    //draw behavior specific parts
    public void draw()
    {
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
        ArrayList<PVector> intrest_forces = new ArrayList<PVector>();
        ArrayList<PVector> member_atts = new ArrayList<PVector>();
        ArrayList<PVector> member_reps = new ArrayList<PVector>();
        ArrayList<PVector> danger_forces = new ArrayList<PVector>();
        ArrayList<PVector> alignment_forces = new ArrayList<PVector>();
        boolean masked = false;

        for(PVector goal: this.goals)
          if(cosine_sim(this.RAY_DIRS.get(idx), PVector.sub(goal,drone.pos)) >= this.SECTOR_COS_SIM) 
            intrest_forces.add(drone.linear_Attraction(goal,GOAL_LIMIT));
        for(Drone other: this.drones)
          if(other != drone)
          {
            if(cosine_sim(this.RAY_DIRS.get(idx), PVector.sub(other.pos,drone.pos)) >= this.SECTOR_COS_SIM) 
            {
              float member_dist = PVector.sub(other.pos,drone.pos).mult(PIXEL_METRIC_CONV).mag();
              if(member_dist<=MEMBER_TO_CLOSE) masked = true;
              else if (member_dist<=SWARM_DIST) alignment_forces.add(other.vel);
              member_atts.add(drone.gausain_Attraction(other.pos,MEMBER_ATT_LIMIT,MEMBER_ATT_CUT_OFF,MEMBER_ATT_SIGMA,MEMBER_ATT_GAMMA,MEMBER_ATT_MEAN)); 
            }
            if(cosine_sim(this.RAY_DIRS.get(idx), PVector.sub(other.pos,drone.pos).rotate(PI)) >= this.SECTOR_COS_SIM) 
            {
              //float member_dist = PVector.sub(other.pos,drone.pos).mult(PIXEL_METRIC_CONV).mag();
              //if(member_dist<=MEMBER_TO_CLOSE) masked = true;
              member_reps.add(drone.linear_Repulsion(other.pos,MEMBER_REP_LIMIT)); 
            }
          }
        for(PVector danger: this.dangers)
          if(cosine_sim(this.RAY_DIRS.get(idx), PVector.sub(danger,drone.pos)) >= this.SECTOR_COS_SIM) 
            danger_forces.add(drone.limExp_Repulsion(danger,DANGER_LIMIT,DANGER_SIGMA,DANGER_GAMMA,DANGER_ALPHA)); 
        for(PVector danger: this.border_points)
          if(cosine_sim(this.RAY_DIRS.get(idx), PVector.sub(danger,drone.pos)) >= this.SECTOR_COS_SIM) 
          {
            float member_dist = PVector.sub(danger,drone.pos).mult(PIXEL_METRIC_CONV).mag();
            if(member_dist<=BORDER_TO_CLOSE) masked = true;
          }

        drone.create_context_segment(idx, intrest_forces, danger_forces, member_atts, member_reps, alignment_forces, !masked);
      }

      //evaluate context steering behavior
      drone.context_steering(this.RAY_DIRS,this.SECTOR_COS_SIM);
    }

}
class Drone {
  //main vars of any drone
  ArrayList<PVector> prev;
  PVector pos,vel,acc;
  final float G = 1.98f;
  final float MAX_SPEED = 3;
  final static boolean DEBUG = false;
  final static int CLIPPED = 0;
  final static int BOUNCING = 1;
  final static int WRAPED = 2;
  
  //context steering specific globals
  ArrayList<ArrayList<PVector>> contextMaps = new ArrayList<ArrayList<PVector>>();
  ArrayList<PVector> currentForces = new ArrayList<PVector>();
  ArrayList<PVector> alignmentForces = new ArrayList<PVector>();
  ArrayList<Boolean> memberMask = new ArrayList<Boolean>();
  PVector prevForce = new PVector();
  final float GOAL_MULT = 1.75f;
  final float DANGER_MULT = 2;
  final float MEMBER_ATT_MULT = 4;
  final float MEMBER_REP_MULT = 0.5f;
  final static int VISUAL_SCALE = 2;
  final static int GOAL_VECTORS = 0;
  final static int DANGER_VECTORS = 1;
  final static int ATT_MEMBER_VECTORS = 2;
  final static int REP_MEMBER_VECTORS = 3;
  int DIRECTIONS;

  //constructors
  Drone(float x, float y) 
  { 
    this.pos = new PVector(x, y);
    this.prev = new ArrayList<PVector>();
    this.prev.add(new PVector(x, y));

    //set random initial velocity
    this.vel = PVector.random2D().setMag(random(MAX_SPEED/10, MAX_SPEED/5));
    //this.vel = new PVector(); //no initial velocity
    if(DEBUG) System.out.println("Init-Drone-Velocity = ("+this.vel.x+","+this.vel.y+")");
    this.acc = new PVector(); 
  }

  Drone(float x, float y, int directions) 
  {
    this(x,y);

    //context steering specific initialization
    this.DIRECTIONS = directions;
    ArrayList<PVector> goals = new ArrayList<PVector>();
    ArrayList<PVector> att_members = new ArrayList<PVector>();
    ArrayList<PVector> rep_members = new ArrayList<PVector>();
    ArrayList<PVector> dangers = new ArrayList<PVector>();

    for(int it=0; it<this.DIRECTIONS; ++it)
    {
      goals.add(new PVector());
      dangers.add(new PVector());
      att_members.add(new PVector());
      rep_members.add(new PVector());
      memberMask.add(true);
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
    this.vel.limit(this.MAX_SPEED);
    if(DEBUG) System.out.println("Drone-Velocity = ("+this.vel.x+","+this.vel.y+")");
    this.pos.add(this.vel);
    this.acc.mult(0);
  }

  public void borders(int h, int w, int border_typ) {
    if(border_typ==CLIPPED)
    {
      while(ceil(this.pos.x)<0)  this.pos.x = 0;
      while(ceil(this.pos.y)<0)  this.pos.y = 0;
      while(floor(this.pos.x)>w) this.pos.x = w;
      while(floor(this.pos.y)>h) this.pos.y = h;
    }
    else if(border_typ==BOUNCING)
    {
      while(ceil(this.pos.x)<0)  this.pos.x = (this.pos.x*(-1))%w;
      while(ceil(this.pos.y)<0)  this.pos.y = (this.pos.y*(-1))%h;
      while(floor(this.pos.x)>w) this.pos.x = w-(this.pos.x%w);
      while(floor(this.pos.y)>h) this.pos.y = h-(this.pos.y%h);
    }
    else if(border_typ==WRAPED)
    {
      while(ceil(this.pos.x)<0)  this.pos.x = this.pos.x+w;
      while(ceil(this.pos.y)<0)  this.pos.y = this.pos.y+h;
      while(floor(this.pos.x)>w) this.pos.x = this.pos.x-w;
      while(floor(this.pos.y)>h) this.pos.y = this.pos.y-h;
    }
  }

  //processing specific code to draw drones
  public void show(int h, int w) 
  {
    borders(h,w,WRAPED);

    //context steering debug
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

    stroke(255, 255, 255, 255);
    strokeWeight(6);
    point(this.pos.x, this.pos.y);

    strokeWeight(2);
    stroke(255,255,255,25);
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
  public void create_context_segment(int dir, ArrayList<PVector> intrest_forces, ArrayList<PVector> danger_forces, ArrayList<PVector> member_atts, ArrayList<PVector> member_reps, ArrayList<PVector> alignment_forces, boolean mask_val)
  {
    PVector strongest_force;
    this.memberMask.set(dir,mask_val);
    for(PVector force: alignment_forces) this.alignmentForces.add(force);

    strongest_force = new PVector();
    for(PVector intrest_force:intrest_forces) 
      if(strongest_force.mag() < intrest_force.mag()) 
        strongest_force = intrest_force;
    this.contextMaps.get(GOAL_VECTORS).set(dir,strongest_force.copy());
    strongest_force = new PVector();
    for(PVector danger_force:danger_forces) 
      if(strongest_force.mag() < danger_force.mag()) 
        strongest_force = danger_force;
    this.contextMaps.get(DANGER_VECTORS).set(dir,strongest_force.copy());

    strongest_force = new PVector();
    for(PVector member_att:member_atts) 
      if(strongest_force.mag() < member_att.mag()) 
        strongest_force = member_att;
    this.contextMaps.get(ATT_MEMBER_VECTORS).set(dir,strongest_force.copy());
    strongest_force = new PVector();
    for(PVector member_rep:member_reps)
      if(strongest_force.mag() < member_rep.mag()) 
        strongest_force = member_rep;
    this.contextMaps.get(REP_MEMBER_VECTORS).set(dir,strongest_force.copy());
  }

  //context steering
  //update acceleration based on context steering behavior
  public void context_steering(ArrayList<PVector> rayDirs, float sectorCosSim)
  { 
    //calculate alignment vector
    PVector alignment = new PVector();
    for(PVector force:this.alignmentForces) alignment.add(force);
    alignment.add(this.vel);
    alignment.div(this.alignmentForces.size()+1);

    //calculate total force per direction
    ArrayList<PVector> forces = new ArrayList<PVector>();
    ArrayList<PVector> visualForces = new ArrayList<PVector>();
    ArrayList<Boolean> mask = new ArrayList<Boolean>();
    for(int idx=0; idx<this.DIRECTIONS; ++idx)
    {
      PVector force = new PVector();
      force.add(this.contextMaps.get(GOAL_VECTORS).get(idx).mult(this.GOAL_MULT));
      force.add(this.contextMaps.get(ATT_MEMBER_VECTORS).get(idx).mult(this.MEMBER_ATT_MULT));
      force.add(this.contextMaps.get(REP_MEMBER_VECTORS).get(idx).mult(this.MEMBER_REP_MULT));
      force.add(this.contextMaps.get(DANGER_VECTORS).get(idx).mult(this.DANGER_MULT));
      
      //danger based mask
      if((cosine_sim(rayDirs.get(idx),force) < 0.0f) || !memberMask.get(idx)) mask.add(false);
      else mask.add(true);
      
      //more likely to perform alignment otherwise less likely to switch directions
      float alignSim = cosine_sim(rayDirs.get(idx),alignment);
      float constrainSim = max(sectorCosSim,cosine_sim(alignment,this.vel));
      if(alignSim < constrainSim) force.mult(map(alignSim,-1.0f,constrainSim,0.25f,1.0f));
      
      forces.add(force.copy());
      if(mask.get(mask.size()-1)) visualForces.add(force.copy());
      else visualForces.add(new PVector());
    }
    this.currentForces = visualForces;

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
    if(maxIdx < forces.size() && forces.get(maxIdx).mag()>1) 
    {
      //interpolate between main and stronger neighbor force
      int leftIdx = (maxIdx-1 + this.DIRECTIONS)%this.DIRECTIONS;
      int rightIdx = (maxIdx+1 + this.DIRECTIONS)%this.DIRECTIONS;
      float leftMag = forces.get(leftIdx).mag(); 
      float rightMag = forces.get(rightIdx).mag();
      int neighborIdx = (leftMag<rightMag && mask.get(rightIdx))? rightIdx: (mask.get(leftIdx))? leftIdx: -1;
      if(DEBUG) System.out.println("Neighbor Direction:"+neighborIdx);
      if(neighborIdx>=0)
      {
        float magnitude = forces.get(neighborIdx).mag()/forces.get(maxIdx).mag();
        PVector secondary_force = PVector.mult(rayDirs.get(neighborIdx),MAX_SPEED*magnitude);
        total_force.add(secondary_force);
      }
    }
    else if(maxIdx >= forces.size() || forces.get(maxIdx).mag()<0.1f)
    this.vel.mult(0);

    //determine force output
    total_force.setMag(MAX_SPEED);  
    this.prevForce = total_force.copy();
    this.acc.add(total_force);
  }

  //context steering
  //calculate repulsion forces to other drones
  public PVector linear_Repulsion(PVector target, float limit)
  {
    PVector force = PVector.sub(target, this.pos).mult(PIXEL_METRIC_CONV);
    float d = force.mag();
    d = min(d, limit);

    float strength = ((-1/18)*d*d+50)*(-1); 

    force.normalize();
    force.mult(strength);
    return force.copy();
  }

  //context steering
  //calculate attraction forces to GOAL_VECTORS
  public PVector linear_Attraction(PVector target, float limit)
  {
    PVector force = PVector.sub(target, this.pos).mult(PIXEL_METRIC_CONV);
    float d = force.mag();
    d = min(d, limit);

    float strength = (G/(-3))*d+21;

    force.normalize();
    force.mult(strength);
    return force.copy();
  }

  //context steering
  //calculate repulsion force from DANGER_VECTORS
  public PVector limExp_Repulsion(PVector target, float limit, float sigma, float gamma, float alpha)
  {
    PVector force = PVector.sub(target, this.pos).mult(PIXEL_METRIC_CONV);
    float d = force.mag();
    float strength;
    d = min(d, limit);

    if (d<11) strength = G * (d/2 - limit) / sigma; //linear_rep_limit_close
    else strength = -1*exp(-1*(G * (alpha*log(d) - gamma)/sigma)); //exp_rep_mid_far

    force = force.normalize();
    force = force.mult(strength);
    return force.copy();
  }

  //context steering
  //calculate attraction forces to other drones
  public PVector gausain_Attraction(PVector target, float limit, float cutOff, float sigma, float gamma, float mean)
  {
    PVector force = PVector.sub(target, this.pos).mult(PIXEL_METRIC_CONV);
    float d = force.mag();
    d = min(d, limit);
    float strength;
    //strength = G*d-10; //simple linear

    if(d<cutOff) strength = (G*(d-mean)-sq(d-mean))/(2*mean)+sigma; //close_mid_range
    else strength = (-1)*(d/(G*G*G*G))+gamma; //low_att_far

    force = force.normalize();
    force = force.mult(strength);
    return force.copy();
  }

  //context steering
  //calculate attraction forces to GOAL_VECTORS
  public PVector invGausain_Attraction(PVector target, float limit, float sigma, float mean)
  {
    PVector force = PVector.sub(target, this.pos).mult(PIXEL_METRIC_CONV);
    float d = force.mag();
    d = min(d, limit);
  
    float tanh_inv_gauss = (float)(Math.tanh(exp(sq(d-mean)/(2*sq(sigma)))/(sigma*sqrt(TWO_PI)))*10-0.25f);
    float quad_eq = sq(d)-(mean*d)-limit;
    float strength = (mean/10)*((G * tanh_inv_gauss) / (limit/10) - quad_eq / (4*limit))-(mean/100);

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
