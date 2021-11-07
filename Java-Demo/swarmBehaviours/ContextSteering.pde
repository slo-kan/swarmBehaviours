class ConSteer_Behavior
{
    ArrayList<Drone> drones;
    ArrayList<PVector> goals = new ArrayList<PVector>();
    ArrayList<PVector> dangers = new ArrayList<PVector>();
    ArrayList<PVector> RAY_DIRS = new ArrayList<PVector>();
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
        this.SECTOR_COS_SIM = cos((TWO_PI/directions)/2);
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
    void draw()
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
    }

    //behavior specific function for each drone
    void conSteer(Drone drone)
    {
      //create all direction segements of each map
      for(int idx=0; idx<RAY_DIRS.size(); ++idx)
      {
        ArrayList<PVector> intrests = new ArrayList<PVector>();
        ArrayList<PVector> members = new ArrayList<PVector>();
        ArrayList<PVector> noFlyZones = new ArrayList<PVector>();

        for(PVector goal: this.goals)
          if(cosine_sim(this.RAY_DIRS.get(idx), goal) >= this.SECTOR_COS_SIM) 
            intrests.add(goal);
        for(Drone other: this.drones)
          if(other.pos != drone.pos && 
             cosine_sim(this.RAY_DIRS.get(idx), other.pos) >= this.SECTOR_COS_SIM) 
            members.add(other.pos); 
        for(PVector danger: this.dangers)
          if(cosine_sim(this.RAY_DIRS.get(idx),danger) >= this.SECTOR_COS_SIM) 
            noFlyZones.add(danger); 

        drone.create_context_segment(idx, intrests, noFlyZones, members);
      }

      //evaluate context steering behavior
      drone.context_steering();
    }

}
