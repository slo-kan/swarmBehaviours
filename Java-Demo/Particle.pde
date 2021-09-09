class Particle {
  PVector pos;
  ArrayList<PVector> prev;
  PVector vel;
  PVector acc;
  float G = 98;

  Particle(float x, float y) {
    pos = new PVector(x, y);
    prev = new ArrayList<PVector>();
    prev.add(new PVector(x, y));
    
    //vel = new PVector(); 
    vel = PVector.random2D();
    //vel.setMag(random(2, 5));
    acc = new PVector();
  }

  void update() {
    vel.add(acc);
    vel.limit(5);
    pos.add(vel);
    acc.mult(0);
  }
 

  void show() {
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


  void primitive_attraction(PVector target, float perlimiter, int multiplier) {
    // PVector dir = target - pos
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = constrain(d, 1, 25);
    if(d > perlimiter)
    {
      float strength = (G*multiplier) / (d * d);
      force.setMag(strength); 
      acc.add(force);
    }
  }
  
  void primitive_repulsion(PVector target, float perlimiter, int multiplier)
  {
    // PVector dir = target - pos
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = constrain(d, 1, 25);
    if(d<perlimiter)
    {
      float strength = (G*multiplier) / (d * d);
      force.setMag(strength);
      acc.sub(force);
    }
  }
  
  void linear_attraction(PVector target, int multiplier) {
    // PVector dir = target - pos
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = G*multiplier; 
    force.setMag(strength); 
    acc.add(force);
  }
  
  void complexExponential_repulsion(PVector target, float perlimiter, int a, int b)
  {
    // PVector dir = target - pos
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float c = (perlimiter*perlimiter) * (float)Math.log(b/a);
    float strength = (G*b) * (float)Math.exp(-(d*d)/c);
    force.setMag(strength);
    acc.sub(force);
  }
  
  void comfy_attraction(PVector target, float perlimiter, int multiplier) {
    // PVector dir = target - pos
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = (G*multiplier) * (d-perlimiter)/max(d,0.01f); 
    force.setMag(strength); 
    acc.add(force);
  }
  
  void simpleExponential_repulsion(PVector target, float perlimiter, int multiplier)
  {
    // PVector dir = target - pos
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = (G*multiplier) * (float)Math.exp(-(d*d)/(2*perlimiter*perlimiter));
    force.setMag(strength);
    acc.sub(force);
  }
};
