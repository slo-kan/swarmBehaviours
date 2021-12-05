class Drone {
  //main vars of any drone
  ArrayList<PVector> prev;
  PVector pos,vel,acc;
  final float G = 1.98;
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
  final float GOAL_MULT = 1.75;
  final float DANGER_MULT = 2;
  final float MEMBER_ATT_MULT = 4;
  final float MEMBER_REP_MULT = 0.5;
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
  void update() 
  {
    if(DEBUG) System.out.println("Drone-Acceleration = ("+this.acc.x+","+this.acc.y+")");
    this.vel.add(this.acc);
    this.vel.limit(this.MAX_SPEED);
    if(DEBUG) System.out.println("Drone-Velocity = ("+this.vel.x+","+this.vel.y+")");
    this.pos.add(this.vel);
    this.acc.mult(0);
  }

  void borders(int h, int w, int border_typ) {
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
  void show(int h, int w) 
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
  void create_context_segment(int dir, ArrayList<PVector> intrest_forces, ArrayList<PVector> danger_forces, ArrayList<PVector> member_atts, ArrayList<PVector> member_reps, ArrayList<PVector> alignment_forces, boolean mask_val)
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
  void context_steering(ArrayList<PVector> rayDirs, float sectorCosSim)
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
      if((cosine_sim(rayDirs.get(idx),force) < 0.0) || !memberMask.get(idx)) mask.add(false);
      else mask.add(true);
      
      //more likely to perform alignment otherwise less likely to switch directions
      float alignSim = cosine_sim(rayDirs.get(idx),alignment);
      float constrainSim = max(sectorCosSim,cosine_sim(alignment,this.vel));
      if(alignSim < constrainSim) force.mult(map(alignSim,-1.0,constrainSim,0.25,1.0));
      
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
    else if(maxIdx >= forces.size() || forces.get(maxIdx).mag()<0.1)
    this.vel.mult(0);

    //determine force output
    total_force.setMag(MAX_SPEED);  
    this.prevForce = total_force.copy();
    this.acc.add(total_force);
  }

  //context steering
  //calculate repulsion forces to other drones
  PVector linear_Repulsion(PVector target, float limit)
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
  PVector linear_Attraction(PVector target, float limit)
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
  PVector limExp_Repulsion(PVector target, float limit, float sigma, float gamma, float alpha)
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
  PVector gausain_Attraction(PVector target, float limit, float cutOff, float sigma, float gamma, float mean)
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
  PVector invGausain_Attraction(PVector target, float limit, float sigma, float mean)
  {
    PVector force = PVector.sub(target, this.pos).mult(PIXEL_METRIC_CONV);
    float d = force.mag();
    d = min(d, limit);
  
    float tanh_inv_gauss = (float)(Math.tanh(exp(sq(d-mean)/(2*sq(sigma)))/(sigma*sqrt(TWO_PI)))*10-0.25);
    float quad_eq = sq(d)-(mean*d)-limit;
    float strength = (mean/10)*((G * tanh_inv_gauss) / (limit/10) - quad_eq / (4*limit))-(mean/100);

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
