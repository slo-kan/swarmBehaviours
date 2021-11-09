class Drone {
  //main vars of any drone
  ArrayList<PVector> prev;
  PVector pos;
  PVector vel;
  PVector acc;
  final float G = 1.98;
  final float MAX_SPEED = 1;
  final boolean DEBUG = false;
  
  //context steering specific globals
  ArrayList<ArrayList<PVector>> contextMaps = new ArrayList<ArrayList<PVector>>();
  ArrayList<PVector> RAY_DIRS = new ArrayList<PVector>();
  PVector prevForce = new PVector();
  int DIRECTIONS;
  float SECTOR_COS_SIM; 
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
    this.SECTOR_COS_SIM = cos(PI/this.DIRECTIONS);
    ArrayList<PVector> goals = new ArrayList<PVector>();
    ArrayList<PVector> members = new ArrayList<PVector>();
    ArrayList<PVector> noFlyZone  = new ArrayList<PVector>();

    for(int it=0; it<this.DIRECTIONS; ++it)
    {
      float angle = it * TWO_PI/this.DIRECTIONS;
      this.RAY_DIRS.add(PVector.fromAngle(angle));
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
  void create_context_segment(int dir, ArrayList<PVector> goals, ArrayList<PVector> noFlyZone, ArrayList<PVector> members)
  {
    //System.out.println("[Direction = "+dir+"]: "+goals.size()+" Goals & "+noFlyZone.size()+" Dangers & "+members.size()+" Members");
    PVector force = new PVector();
    for(PVector goal:goals) force.add(invGausain_Attraction(goal));
    this.contextMaps.get(GOALS).set(dir,force.copy());
    force = new PVector();
    for(PVector member:members) force.add(gausain_AttRep(member));
    this.contextMaps.get(MEMBERS).set(dir,force.copy());
    force = new PVector();
    for(PVector danger:noFlyZone) force.add(limExp_Repulsion(danger));
    this.contextMaps.get(DANGERS).set(dir,force.copy());
  }

  //context steering
  //update acceleration based on context steering behavior
  void context_steering()
  {
    if(DEBUG)
    { 
      for(int it=0; it<this.DIRECTIONS; ++it)
        System.out.print("RayDir["+it+"] = ("+this.RAY_DIRS.get(it).x+","+this.RAY_DIRS.get(it).y+"); ");
      System.out.println("End;");
    }
    
    //calculate total force per direction
    ArrayList<PVector> forces = new ArrayList<PVector>();
    for(int idx=0; idx<this.DIRECTIONS; ++idx)
    {
      PVector force = new PVector();
      force.add(this.contextMaps.get(GOALS).get(idx).mult(1));
      //force.add(this.contextMaps.get(MEMBERS).get(idx).mult(0.5));
      //force.add(this.contextMaps.get(DANGERS).get(idx).mult(2));
      if(cosine_sim(this.RAY_DIRS.get(idx),force) < 0) force = new PVector(); //to strong danger means no force
      else if(this.prevForce.mag()!=0)
      { 
        //less likely to switch directions
        float cosSim = cosine_sim(this.RAY_DIRS.get(idx),this.prevForce);
        if(cosSim < this.SECTOR_COS_SIM) force.mult(map(cosSim,-1.0,this.SECTOR_COS_SIM,0.5,1.5));
      }
      forces.add(force.copy());
    }

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
    PVector main_force = PVector.mult(this.RAY_DIRS.get(maxIdx),MAX_SPEED);
    if(DEBUG) System.out.println("Choosen Direction: "+maxIdx);

    //interpolate between main and stronger neighbor force
    int leftIdx = (maxIdx-1 + this.DIRECTIONS)%this.DIRECTIONS;
    int rightIdx = (maxIdx+1 + this.DIRECTIONS)%this.DIRECTIONS;
    float leftMag = forces.get(leftIdx).mag(); 
    float rightMag = forces.get(rightIdx).mag();
    int neighborIdx = (leftMag<rightMag)?rightIdx:leftIdx;
    if(DEBUG) System.out.println("neighborIdx = "+neighborIdx);
    float magnitude = forces.get(neighborIdx).mag()/maxMag;
    PVector secondary_force = PVector.mult(this.RAY_DIRS.get(neighborIdx),MAX_SPEED*magnitude);

    //determine force output
    PVector force = PVector.add(main_force,secondary_force).setMag(MAX_SPEED); 
    if(DEBUG) System.out.println("ACC Force("+force.x+","+force.y+") = "+"main("+main_force.x+","+main_force.y+") + secondary("+secondary_force.x+","+secondary_force.y+")"); 
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
    return force;
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
    return force;
  }
  
  //context steering
  //calculate attraction forces to goals
  PVector invGausain_Attraction(PVector target)
  {
    final float LIMIT = 30;
    final float SIGMA = 5;
    final float MEAN = LIMIT/2;

    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = constrain(d, 0, LIMIT);
    float tanh_inv_gauss = (float)(Math.tanh(exp(sq(d-MEAN)/(2*sq(SIGMA)))/(SIGMA*sqrt(TWO_PI)))*10-0.25);
    float quad_eq = sq(d)-(MEAN*d)-LIMIT;
    float strength = (MEAN/10)*((G * tanh_inv_gauss) / (LIMIT/10) - quad_eq / (4*LIMIT))-(MEAN/100);
    //float strength = (G/(-3))*d+20; //simple linear
    force = force.normalize();
    force = force.mult(strength);
    return force;
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
