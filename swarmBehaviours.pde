import java.util.ArrayList;

ArrayList<PVector> attractors = new ArrayList<PVector>();
ArrayList<Particle> particles = new ArrayList<Particle>();
float perlimiter = 8;
int ticks = 0;

void setup() {
  size(2400, 1400);
  
  for(int it=0; it<100; ++it)
     particles.add(new Particle(random(width), random(height)));

  for(int it = 0; it < 8; ++it)
     attractors.add(new PVector(random(width), random(height)));
}

void newAttractors(){
  for(int it = 0; it < 4; ++it){
     attractors.remove(0);
     attractors.add(new PVector(random(width), random(height)));
  }
}

//void mousePressed() { attractors.add(new PVector(mouseX, mouseY)); }

void draw() {
  background(51);
  
  if(ticks==28)
  {
    ticks = 0;
    newAttractors();
  }

  stroke(0, 255, 0);
  for (PVector attractor:this.attractors)
    point(attractor.x, attractor.y);
  
  stroke(255);
  strokeWeight(4);
  for(Particle particle:this.particles) 
  {
    for(PVector attractor:this.attractors)
      particle.attracted(attractor,perlimiter);
    for(PVector attractor:this.attractors)
      particle.repulsed(attractor,perlimiter,1);
    for(Particle otherPart:this.particles)
       if(otherPart!=particle) particle.repulsed(otherPart.pos,2*perlimiter,4);
    
    particle.update();
    particle.show();
  }
  ++ticks;
}
