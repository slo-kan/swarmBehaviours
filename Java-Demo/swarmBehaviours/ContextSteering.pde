class ConSteer_Behavior
{
    ArrayList<Drone> drones;
    ArrayList<PVector> goals = new ArrayList<PVector>();
    ArrayList<PVector> dangers = new ArrayList<PVector>();
    ArrayList<PVector> RAY_DIRS = new ArrayList<PVector>();
    float SECTOR_COS_SIM;
    
    ConSteer_Behavior(ArrayList<Drone> drones, int directions)
    { 
        this.drones = drones; 
        for(int it=0; it<directions; ++it)
        {
          float angle = it * TWO_PI/directions;
          this.RAY_DIRS.add(PVector.fromAngle(angle));
        }
        this.SECTOR_COS_SIM = (this.RAY_DIRS.get(0).dot(this.RAY_DIRS.get(1))/(this.RAY_DIRS.get(0).mag()*this.RAY_DIRS.get(1).mag()))/2;
    }

    void setup()
    {
      for(int it = 0; it < 8; ++it)
        this.goals.add(new PVector(random(width), random(height)));
    
      for(int it = 0; it < 4; ++it)
        this.dangers.add(new PVector(random(width), random(height)));
    }

    void mousePressed(){ this.goals.add(new PVector(mouseX, mouseY)); }
    
    void update()
    {
      for(int it = 0; it < 4; ++it){
        this.goals.remove(0);
        this.goals.add(new PVector(random(width), random(height)));
      }
      for(int it = 0; it < 2; ++it){
        this.dangers.remove(0);
        this.dangers.add(new PVector(random(width), random(height)));
      }
    }

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
    }

    void conSteer(Drone drone)
    {
      for(int idx=0; idx<RAY_DIRS.size(); ++idx)
      {
         ArrayList<PVector> intrests = new ArrayList<PVector>();
         ArrayList<PVector> members = new ArrayList<PVector>();
         ArrayList<PVector> noFlyZones = new ArrayList<PVector>();
         for(PVector goal: this.goals)
         {
             float cosSim = this.RAY_DIRS.get(idx).dot(goal)/(this.RAY_DIRS.get(idx).mag()*goal.mag());
             if(cosSim <= this.SECTOR_COS_SIM) intrests.add(goal);
         }
         for(Drone other: this.drones)
         {
             if(other.pos != drone.pos)
             {
               float cosSim = this.RAY_DIRS.get(idx).dot(other.pos)/(this.RAY_DIRS.get(idx).mag()*other.pos.mag());
               if(cosSim <= this.SECTOR_COS_SIM) members.add(other.pos); 
             }
         }
         for(PVector danger: this.dangers)
         {
             float cosSim = this.RAY_DIRS.get(idx).dot(danger)/(this.RAY_DIRS.get(idx).mag()*danger.mag());
             if(cosSim <= this.SECTOR_COS_SIM) noFlyZones.add(danger); 
         }
         drone.create_context_segment(idx, intrests, noFlyZones, members);
      }
      drone.context_steering();
    }

}
