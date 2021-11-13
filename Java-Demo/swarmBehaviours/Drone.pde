class Drone {
  //main vars of any drone
  ArrayList<PVector> prev;
  PVector pos;
  PVector vel;
  PVector acc;
  final float G = 1.98;
  final float MAX_SPEED = 3;
  final static boolean DEBUG = false;
  
  //context steering specific globals
  ArrayList<ArrayList<PVector>> contextMaps = new ArrayList<ArrayList<PVector>>();
  ArrayList<PVector> currentForces = new ArrayList<PVector>();
  PVector prevForce = new PVector();
  int DIRECTIONS;
  final int GOALS = 0;
  final int DANGERS = 1;
  final int MEMBERS = 2;

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
    ArrayList<PVector> members = new ArrayList<PVector>();
    ArrayList<PVector> noFlyZone  = new ArrayList<PVector>();

    for(int it=0; it<this.DIRECTIONS; ++it)
    {
      goals.add(new PVector());
      noFlyZone.add(new PVector());
      members.add(new PVector());
    }

    this.contextMaps.add(goals);
    this.contextMaps.add(noFlyZone);
    this.contextMaps.add(members);
  }

  //update a drones position based on simple physics(velocity)  
  //update velocity based on current acceleartion and reset the acceleartion
  void update() 
  {
    if(DEBUG) System.out.println("Drone-Acceleration = ("+this.acc.x+","+this.acc.y+")");
    this.vel.add(this.acc);
    this.vel.limit(MAX_SPEED);
    if(DEBUG) System.out.println("Drone-Velocity = ("+this.vel.x+","+this.vel.y+")");
    this.pos.add(this.vel);
    this.acc.mult(0);
  }

  void borders(int h, int w) {
    while(ceil(this.pos.x)<0) this.pos.x = this.pos.x+w;
    while(ceil(this.pos.y)<0) this.pos.y = this.pos.y+h;
    while(floor(this.pos.x)>w) this.pos.x = this.pos.x-w;
    while(floor(this.pos.y)>h) this.pos.y = this.pos.y-h;
  }

  //processing specific code to draw drones
  void show(int h, int w) 
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
        line(this.pos.x,this.pos.y,this.pos.x+force.x*MAX_SPEED,this.pos.y+force.y*MAX_SPEED);
        strokeWeight(4);
        point(this.pos.x+force.x*MAX_SPEED,this.pos.y+force.y*MAX_SPEED);
      }
    }

    if(!DEBUG)
    {
      stroke(255, 0, 0);
      strokeWeight(2);
      line(this.pos.x,this.pos.y,this.pos.x+this.prevForce.x*10,this.pos.y+this.prevForce.y*10);
      strokeWeight(4);
      point(this.pos.x+this.prevForce.x*10,this.pos.y+this.prevForce.y*10);
    }

    if(!DEBUG) 
    {
      stroke(255, 0, 0);
      for(PVector danger: this.contextMaps.get(DANGERS))
      {
        strokeWeight(2);
        line(this.pos.x,this.pos.y,this.pos.x+danger.x*MAX_SPEED,this.pos.y+danger.y*MAX_SPEED);
        strokeWeight(4);
        point(this.pos.x+danger.x*MAX_SPEED,this.pos.y+danger.y*MAX_SPEED);
      }

      stroke(0, 0, 255);
      for(PVector member: this.contextMaps.get(MEMBERS))
      {
        strokeWeight(2);
        line(this.pos.x,this.pos.y,this.pos.x+member.x*MAX_SPEED,this.pos.y+member.y*MAX_SPEED);
        strokeWeight(4);
        point(this.pos.x+member.x*MAX_SPEED,this.pos.y+member.y*MAX_SPEED);
      }

      stroke(0, 255, 0);
      for(PVector goal: this.contextMaps.get(GOALS))
      {
        strokeWeight(2);
        line(this.pos.x,this.pos.y,this.pos.x+goal.x*MAX_SPEED,this.pos.y+goal.y*MAX_SPEED);
        strokeWeight(4);
        point(this.pos.x+goal.x*MAX_SPEED,this.pos.y+goal.y*MAX_SPEED);
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
  void create_context_segment(int dir, ArrayList<PVector> goals, ArrayList<PVector> noFlyZones, ArrayList<PVector> members)
  {
    //System.out.println("[Direction = "+dir+"]: "+goals.size()+" Goals & "+noFlyZone.size()+" Dangers & "+members.size()+" Members");
    PVector force = new PVector();
    for(PVector goal:goals) force.add(invGausain_Attraction(goal));
    this.contextMaps.get(GOALS).set(dir,PVector.mult(force,1/max(goals.size(),1)));
    force = new PVector();
    for(PVector member:members) force.add(gausain_AttRep(member));
    this.contextMaps.get(MEMBERS).set(dir,PVector.mult(force,1/max(members.size(),1)));
    force = new PVector();
    for(PVector danger:noFlyZones) force.add(limExp_Repulsion(danger));
    this.contextMaps.get(DANGERS).set(dir,PVector.mult(force,1/max(noFlyZones.size(),1)));
  }

  //context steering
  //update acceleration based on context steering behavior
  void context_steering(ArrayList<PVector> rayDirs, float sectorCosSim)
  {
    if(DEBUG)
    { 
      for(int it=0; it<this.DIRECTIONS; ++it)
        System.out.print("RayDir["+it+"] = ("+rayDirs.get(it).x+","+rayDirs.get(it).y+"); ");
      System.out.println("End;");
    }
    
    //calculate total force per direction
    ArrayList<PVector> forces = new ArrayList<PVector>();
    for(int idx=0; idx<this.DIRECTIONS; ++idx)
    {
      PVector force = new PVector();
      force.add(this.contextMaps.get(GOALS).get(idx).mult(1));
      force.add(this.contextMaps.get(MEMBERS).get(idx).mult(1));
      force.add(this.contextMaps.get(DANGERS).get(idx).mult(1));
      if(DEBUG && force.x!=0 && force.y!=0) System.out.println("Direction: "+idx+"; COS_SIM = "+cosine_sim(rayDirs.get(idx),force)+
      "; dir("+rayDirs.get(idx).x+","+rayDirs.get(idx).y+"); force("+force.x+","+force.y+")");
      if(cosine_sim(rayDirs.get(idx),force) < sectorCosSim) force = new PVector(); //to strong danger means no force
      else if(this.prevForce.mag()!=0)
      { 
        //less likely to switch directions
        float cosSim = cosine_sim(rayDirs.get(idx),this.prevForce);
        if(cosSim < sectorCosSim) force.mult(map(cosSim,-1.0,sectorCosSim,0.001,0.75));
      }
      forces.add(force.copy());
    }
    this.currentForces = forces;

    //select strongest force as main force direction
    int maxIdx = 0;
    float maxMag = forces.get(maxIdx).mag();
    for(int idx=1; idx<forces.size(); ++idx) 
    {
      float magnitude = forces.get(idx).mag();
      if(magnitude>maxMag) 
      {
        maxMag = magnitude;
        maxIdx = idx;
      }
    }
    PVector main_force = PVector.mult(rayDirs.get(maxIdx),MAX_SPEED);
    if(DEBUG) System.out.println("Choosen Direction: "+maxIdx);

    PVector force = new PVector();
    if(maxMag>0.5) 
    {
      //interpolate between main and stronger neighbor force
      int leftIdx = (maxIdx-1 + this.DIRECTIONS)%this.DIRECTIONS;
      int rightIdx = (maxIdx+1 + this.DIRECTIONS)%this.DIRECTIONS;
      float leftMag = forces.get(leftIdx).mag(); 
      float rightMag = forces.get(rightIdx).mag();
      int neighborIdx = (leftMag<rightMag)?rightIdx:leftIdx;
      if(DEBUG) System.out.println("neighborIdx = "+neighborIdx);
      if(DEBUG) System.out.println("maxMag: "+maxMag+"; n_mag: "+forces.get(neighborIdx).mag());
      float magnitude = forces.get(neighborIdx).mag()/maxMag;
      if(DEBUG) System.out.println("direction force("+rayDirs.get(neighborIdx).x+","+rayDirs.get(neighborIdx).y+"); mag: "+MAX_SPEED*magnitude);
      PVector secondary_force = PVector.mult(rayDirs.get(neighborIdx),MAX_SPEED*magnitude);
      force.add(secondary_force);
    }
    force.add(main_force);

    //determine force output
    force.setMag(MAX_SPEED);  
    this.prevForce = force.copy();
    this.acc.add(force);
  }

  //context steering
  //calculate repulsion force from dangers
  PVector limExp_Repulsion(PVector target)
  {
    final float LIMIT = 40;
    final float SIGMA = 3.85;

    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength;
    d = constrain(d, 0, LIMIT);
    if (d<11) strength = G * (0.5*d - LIMIT) / SIGMA; //linear_rep_limit_close
    else strength = -1*exp(-1*(G * ((LIMIT/8)*log(d) - (LIMIT/2-1.5))/SIGMA)); //exp_rep_mid_far
    force = force.normalize();
    force = force.mult(strength);
    return force.copy();
  }

  //context steering
  //calculate attraction and repulsion forces from other drones
  PVector gausain_AttRep(PVector target)
  {
    final float LIMIT = 30;
    final float SIGMA = 7.219;
    final float MEAN = 7.5;

    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = constrain(d, 0, LIMIT);
    float strength;
    //strength = G*d-10; //simple linear
    if(d<5) strength = G*(d-5); //medium_rep_close
    else if(d<14.5) strength = G*(2*d-sq(d)/10-MEAN); //high_att_mid
    else strength = exp(G *(SIGMA-d/2)); //low_att_far*/
    force = force.normalize();
    force = force.mult(strength);
    return force.copy();
  }
  
  //context steering
  //calculate attraction forces to goals
  PVector invGausain_Attraction(PVector target)
  {
    final float LIMIT = 30.0f;
    final float SIGMA = 5.0f;
    final float MEAN = LIMIT/2;

    if(DEBUG) System.out.println("target("+target.x+","+target.y+"); pos("+this.pos.x+","+this.pos.y+")");
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = constrain(d, 0.0f, LIMIT);
    float tanh_inv_gauss = (float)(Math.tanh(exp(sq(d-MEAN)/(2*sq(SIGMA)))/(SIGMA*sqrt(TWO_PI)))*10-0.25);
    float quad_eq = sq(d)-(MEAN*d)-LIMIT;
    float strength = (MEAN/10)*((G * tanh_inv_gauss) / (LIMIT/10) - quad_eq / (4*LIMIT))-(MEAN/100);
    //float strength = (G/(-3))*d+20; //simple linear
    force.normalize();
    force.mult(strength);
    return force.copy();
  }

  //att_rep behavior
  //calculate attraction force
  void primitive_attraction(PVector target, float perlimiter, int multiplier) 
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
  void primitive_repulsion(PVector target, float perlimiter, int multiplier)
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
  void linear_attraction(PVector target, int multiplier) 
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
  void complexExponential_repulsion(PVector target, float perlimiter, int a, int b)
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
  void comfy_attraction(PVector target, float perlimiter, int multiplier) 
  {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = (G*multiplier) * (d-perlimiter)/max(d,0.01f); 
    force.setMag(strength); 
    this.acc.add(force);
  }
  
  //att_rep behavior
  //calculate exponential repulsion force to repell points
  void simpleExponential_repulsion(PVector target, float perlimiter, int multiplier)
  {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = (G*multiplier) * (float)Math.exp(-(d*d)/(2*perlimiter*perlimiter));
    force.setMag(strength);
    this.acc.sub(force);
  }
};
