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
    void setup(int numGoals, int numDangers)
    {
      for(int it = 0; it < numGoals; ++it)
        this.goals.add(new PVector(random(width-4), random(height-4)));
    
      for(int it = 0; it < numDangers; ++it)
        this.dangers.add(new PVector(random(width-4), random(height-4)));
    }

    //manually add or remove goal or danger points
    void mousePressed()
    { 
      if(mouseButton==LEFT && !keyPressed) this.goals.add(new PVector(mouseX, mouseY)); 
      else if(mouseButton==RIGHT && !keyPressed) this.dangers.add(new PVector(mouseX, mouseY));
      else if(mouseButton==LEFT && key == DELETE && !this.goals.isEmpty()) this.goals.remove(this.goals.size()-1);
      else if(mouseButton==RIGHT && key == DELETE && !this.dangers.isEmpty()) this.dangers.remove(this.dangers.size()-1);
    }
    
    //automatically manipulate points
    void update(int portion)
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
    void removeVisited()
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
    void draw()
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
    void conSteer(Drone drone)
    {
      //create all direction segements of each map
      for(int idx=0; idx<this.RAY_DIRS.size(); ++idx)
      {
        ArrayList<PVector> intrest_forces = new ArrayList<PVector>();
        ArrayList<PVector> member_atts = new ArrayList<PVector>();
        ArrayList<PVector> member_reps = new ArrayList<PVector>();
        ArrayList<PVector> danger_forces = new ArrayList<PVector>();
        boolean masked = false;

        for(PVector goal: this.goals)
          if(cosine_sim(this.RAY_DIRS.get(idx), PVector.sub(goal,drone.pos)) >= this.SECTOR_COS_SIM) 
            intrest_forces.add(drone.invGausain_Attraction(goal));
        for(Drone other: this.drones)
          if(other != drone)
          {
            if(cosine_sim(this.RAY_DIRS.get(idx), PVector.sub(other.pos,drone.pos)) >= this.SECTOR_COS_SIM) 
            {
              if(PVector.sub(other.pos,drone.pos).mult(PIXEL_METRIC_CONV).mag()<10) masked = true;
              member_atts.add(PVector.sub(drone.gausain_Attraction(other.pos).mult(0.66),drone.linear_Repulsion(other.pos).mult(0.33))); 
            }
            if(cosine_sim(this.RAY_DIRS.get(idx), PVector.sub(other.pos,drone.pos).rotate(PI)) >= this.SECTOR_COS_SIM) 
              member_reps.add(drone.linear_Repulsion(other.pos)); 
          }
        for(PVector danger: this.dangers)
          if(cosine_sim(this.RAY_DIRS.get(idx), PVector.sub(danger,drone.pos)) >= this.SECTOR_COS_SIM) 
            danger_forces.add(drone.limExp_Repulsion(danger)); 

        drone.create_context_segment(idx, intrest_forces, danger_forces, member_atts, member_reps, !masked);
      }

      //evaluate context steering behavior
      drone.context_steering(this.RAY_DIRS,this.SECTOR_COS_SIM);
    }

}
