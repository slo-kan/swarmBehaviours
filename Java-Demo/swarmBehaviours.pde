import java.util.ArrayList;

ArrayList<PVector> attractors = new ArrayList<PVector>();
ArrayList<PVector> repellPoints = new ArrayList<PVector>();
ArrayList<Particle> particles = new ArrayList<Particle>();
float comfy_dist = 17;
float perlimiter = 64.25;
int ticks = 0;

void setup() {
  size(2400, 1400);
  
  for(int it=0; it<100; ++it)
     particles.add(new Particle(random(width), random(height)));

  for(int it = 0; it < 8; ++it)
     attractors.add(new PVector(random(width), random(height)));
  
  for(int it = 0; it < 4; ++it)
     repellPoints.add(new PVector(random(width), random(height)));
}

void newAttractorsAndRepellPoints(){
  for(int it = 0; it < 4; ++it){
     attractors.remove(0);
     attractors.add(new PVector(random(width), random(height)));
  }
  for(int it = 0; it < 2; ++it){
     repellPoints.remove(0);
     repellPoints.add(new PVector(random(width), random(height)));
  }
}
//void mousePressed() { attractors.add(new PVector(mouseX, mouseY)); }

void primitive_attRep(Particle particle)
{
  for(PVector attractor:this.attractors)
     particle.primitive_attraction(attractor,comfy_dist,10);
  for(PVector repellPoint:this.repellPoints)
     particle.primitive_repulsion(repellPoint,comfy_dist,20);
  for(Particle otherPart:this.particles)
     if(otherPart!=particle) particle.primitive_attraction(otherPart.pos,2*comfy_dist,1);
  for(Particle otherPart:this.particles)
     if(otherPart!=particle) particle.primitive_repulsion(otherPart.pos,1.2*comfy_dist,5);  
}

void advanced_attRep(Particle particle)
{
  for(PVector attractor:this.attractors)
     particle.comfy_attraction(attractor,comfy_dist,10);
  for(PVector repellPoint:this.repellPoints)
     particle.simpleExponential_repulsion(repellPoint,perlimiter,200);
  for(Particle otherPart:this.particles)
     if(otherPart!=particle) particle.linear_attraction(otherPart.pos,1);
  for(Particle otherPart:this.particles)
     if(otherPart!=particle) particle.complexExponential_repulsion(otherPart.pos,perlimiter,1,5);
}

void draw() {
  background(51);
  
  if(ticks==112)
  {
    ticks = 0;
    newAttractorsAndRepellPoints();
  }

  stroke(0, 255, 0);
  strokeWeight(8);
  for (PVector attractor:this.attractors)
    point(attractor.x, attractor.y);
    
  stroke(0, 0, 255);
  strokeWeight(8);
  for (PVector repellPoint:this.repellPoints)
    point(repellPoint.x, repellPoint.y);
  
  stroke(255);
  strokeWeight(4);
  for(Particle particle:this.particles) 
  {
    //primitive_attRep(particle);
    advanced_attRep(particle);
    
    particle.update();
    particle.show();
  }
  ++ticks;
}
