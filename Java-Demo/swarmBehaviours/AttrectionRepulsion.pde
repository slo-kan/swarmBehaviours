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
  void draw()
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
  void primitive_attRep(Drone drone)
  {
    for(PVector attractor:this.attractors)
      drone.primitive_attraction(attractor,comfy_dist,10);

    for(PVector repellPoint:this.repellPoints)
      drone.primitive_repulsion(repellPoint,comfy_dist,20);

    for(Drone other:this.drones)
      if(other!=drone) drone.primitive_attraction(other.pos,2*comfy_dist,1);
    for(Drone other:this.drones)
      if(other!=drone) drone.primitive_repulsion(other.pos,1.2*comfy_dist,5);  
  }

  //behavior specific function for each drone
  void advanced_attRep(Drone drone)
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
