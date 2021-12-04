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
  void setup(int ATTs,int REPs)
  {
    for(int it = 0; it < ATTs; ++it)
      this.attractors.add(new PVector(random(width-4), random(height-4)));
    
    for(int it = 0; it < REPs; ++it)
      this.repellPoints.add(new PVector(random(width-4), random(height-4)));
  }

  //manually add attraction or repulsion points
  void mousePressed()
  { 
    if(mouseButton==LEFT && !keyPressed) this.attractors.add(new PVector(mouseX, mouseY)); 
    else if(mouseButton==RIGHT && !keyPressed) this.repellPoints.add(new PVector(mouseX, mouseY));
    else if(mouseButton==LEFT && key == DELETE && !this.attractors.isEmpty()) this.attractors.remove(this.attractors.size()-1); 
    else if(mouseButton==RIGHT && key == DELETE && !this.repellPoints.isEmpty()) this.repellPoints.remove(this.repellPoints.size()-1); 
  }
  
  //automatically manipulate points
  void update(int portion)
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
  void removeVisited()
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
  void draw()
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
  void primitive_attRep(Drone drone)
  {
    for(PVector attractor:this.attractors)
      drone.primitive_attraction(attractor,COMFY_DIST,10);

    for(PVector repellPoint:this.repellPoints)
      drone.primitive_repulsion(repellPoint,COMFY_DIST,20);

    for(Drone other:this.drones)
      if(other!=drone) drone.primitive_attraction(other.pos,2*COMFY_DIST,1);
    for(Drone other:this.drones)
      if(other!=drone) drone.primitive_repulsion(other.pos,1.2*COMFY_DIST,5);  
  }

  //behavior specific function for each drone
  void advanced_attRep(Drone drone)
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
