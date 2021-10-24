class Drone {
  ArrayList<PVector> prev;
  PVector pos;
  PVector vel;
  PVector acc;
  final float G = 1.98;
  

  ArrayList<ArrayList<PVector>> contextMaps = new ArrayList<ArrayList<PVector>>();
  ArrayList<PVector> RAY_DIRS;
  PVector prevForce = new PVector();
  int DIRECTIONS = 8;
  final int GOALS = 0;
  final int DANGERS = 1;
  final int MEMBERS = 2;
  final float MAX_SPEED = 0.5;

  Drone(float x, float y) 
  {
    this.pos = new PVector(x, y);
    this.prev = new ArrayList<PVector>();
    this.prev.add(new PVector(x, y));
    
    this.vel = PVector.random2D();
    this.vel.setMag(random(0, 3));
    this.acc = new PVector();
  }

  Drone(float x, float y, int directions) 
  {
    this.pos = new PVector(x, y);
    this.prev = new ArrayList<PVector>();
    this.prev.add(new PVector(x, y));
    
    this.vel = PVector.random2D();
    this.vel.setMag(random(0, 3));
    this.acc = new PVector();

    this.DIRECTIONS = directions;
    this.RAY_DIRS = new ArrayList<PVector>();
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

  void update() 
  {
    vel.add(acc);
    vel.limit(MAX_SPEED);
    pos.add(vel);
    acc.mult(0);
  }

  void show() 
  {
    stroke(255, 255);
    strokeWeight(6);
    point(this.pos.x, this.pos.y);
    
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

  void create_context_segment(int dir, ArrayList<PVector> goals, ArrayList<PVector> noFlyZone, ArrayList<PVector> members)
  {
    PVector force = new PVector();
    for(PVector goal:goals) force = force.add(inverse_gausain_attraction(goal));
    this.contextMaps.get(GOALS).set(dir,force.copy());
    force = new PVector();
    for(PVector member:members) force = force.add(gausain_attraction(member));
    this.contextMaps.get(MEMBERS).set(dir,force.copy());
    force = new PVector();
    for(PVector danger:noFlyZone) force = force.add(limited_repulsion(danger));
    this.contextMaps.get(DANGERS).set(dir,force.copy());
  }

  void context_steering()
  {
    ArrayList<PVector> forces = new ArrayList<PVector>();
    for(int idx=0; idx<this.DIRECTIONS; ++idx)
    {
      PVector force = new PVector();
      force = this.contextMaps.get(GOALS).get(idx).mult(10).add(this.contextMaps.get(MEMBERS).get(idx)).add(this.contextMaps.get(DANGERS).get(idx).mult(30));
      if(this.prevForce.mag()!=0)
      { 
        float cosSim = this.RAY_DIRS.get(idx).dot(this.prevForce)/(this.RAY_DIRS.get(idx).mag()*this.prevForce.mag());
        if(cosSim < 0) force = force.mult(cosSim+1.5);
      }
      forces.add(force);
    }

    int maxIdx = 0;
    float maxMag = forces.get(0).mag();
    for(int idx=0; idx<forces.size(); ++idx) 
      if(forces.get(idx).mag()>maxMag) 
      {
        maxMag = forces.get(idx).mag();
        maxIdx = idx;
      }

    int rNIdx = (maxIdx+1 + this.DIRECTIONS) % this.DIRECTIONS;
    int lNIdx = (maxIdx-1 + this.DIRECTIONS) % this.DIRECTIONS;
    int neighborIdx = lNIdx;
    float neighborMag = forces.get(lNIdx).mag();
    if(forces.get(rNIdx).mag()>forces.get(lNIdx).mag())
    {
      neighborIdx = rNIdx;
      neighborMag = forces.get(rNIdx).mag();
    }

    PVector force = this.RAY_DIRS.get(maxIdx).copy().setMag(MAX_SPEED);
    float mag = neighborMag/maxMag-0.5;
    if(mag>0) force = force.add(this.RAY_DIRS.get(neighborIdx).copy().setMag(MAX_SPEED*mag));
    force.setMag(MAX_SPEED);  
    this.prevForce = force;
    this.acc.add(force);
  }

  PVector limited_repulsion(PVector target)
  {
    final float LIMIT = 20;
    final float SIGMA = 3.85;

    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength;
    d = constrain(d, 0, 2*LIMIT);
    if (d>(LIMIT-5)) strength = G * (5*log(d)-(LIMIT-1.5))/3.85;
    else {
      d = constrain(d, 0, LIMIT);
      strength = G * (d - LIMIT) / SIGMA;
    }
    force.setMag(strength);
    return force;
  }

  PVector gausain_attraction(PVector target)
  {
    final float LIMIT = 30;
    final float SIGMA = 4.7;
    final float MEAN = LIMIT/2;

    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = constrain(d, 0.5, LIMIT);
    float tanh_gauss = (float)(Math.tanh(exp(sq(d-MEAN)/(2*sq(SIGMA)))/(SIGMA*sqrt(TWO_PI)))*7);
    float strength = (G * tanh_gauss * d) / LIMIT - 0.5;
    force.setMag(strength);
    return force;
  }
  
  PVector inverse_gausain_attraction(PVector target)
  {
    final float LIMIT = 30;
    final float SIGMA = 5;
    final float MEAN = LIMIT/2;

    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = constrain(d, 0.5, LIMIT);
    float tanh_inverted_gauss = (float)(Math.tanh(exp(sq(d-MEAN)/(2*sq(SIGMA)))/(SIGMA*sqrt(TWO_PI)))*10+0.75);
    float strength = (G * tanh_inverted_gauss) / (LIMIT/10) - (sq(d)+360)/(4*LIMIT);
    force.setMag(strength);
    return force;
  }

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
  
  void linear_attraction(PVector target, int multiplier) 
  {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = G*multiplier; 
    force.setMag(strength); 
    this.acc.add(force);
  }
  
  void complexExponential_repulsion(PVector target, float perlimiter, int a, int b)
  {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float c = (perlimiter*perlimiter) * (float)Math.log(b/a);
    float strength = (G*b) * (float)Math.exp(-(d*d)/c);
    force.setMag(strength);
    this.acc.sub(force);
  }
  
  void comfy_attraction(PVector target, float perlimiter, int multiplier) 
  {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = (G*multiplier) * (d-perlimiter)/max(d,0.01f); 
    force.setMag(strength); 
    this.acc.add(force);
  }
  
  void simpleExponential_repulsion(PVector target, float perlimiter, int multiplier)
  {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = (G*multiplier) * (float)Math.exp(-(d*d)/(2*perlimiter*perlimiter));
    force.setMag(strength);
    this.acc.sub(force);
  }
};
