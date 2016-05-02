import traer.physics.*;

static int NUM_POINTS = 10;
static int PARTICLE_SIZE = 30;

ParticleSystem physics;
Particle[] particles = new Particle[NUM_POINTS];

void setup() {
 
  size(1600, 900); // largescreen
  //size(1680, 1050); // fullscreen
  smooth();
  
  physics = new ParticleSystem(0.0, 0.0);

  for(int i=0; i<NUM_POINTS; i++) {
    particles[i] = physics.makeParticle(1.0, (int)random(0, width), (int)random(0, height), 0.0);
  }
  
  for(int i=1; i<NUM_POINTS; i++) {
    physics.makeAttraction(particles[i-1], particles[i], -10000, 10);
  }
  
}

void draw() {
  
  background(255);
  
  physics.tick();
  
  fill(0, 255, 0);
  for(int i=0; i<NUM_POINTS; i++) {
    ellipse(particles[i].position().x(), particles[i].position().y(), PARTICLE_SIZE, PARTICLE_SIZE);
  }
  
  // handle the boundaries
  for(int i=0; i<NUM_POINTS; i++) {
    handleBoundaryCollisions(particles[i]);
  }
  
}

void handleBoundaryCollisions( Particle p ) {
  if ( p.position().x() < 0 || p.position().x() > width ) {
    p.velocity().set(-0.9*p.velocity().x(), p.velocity().y(), 0);
    //p.setVelocity( -0.9*p.velocity().x(), p.velocity().y(), 0 );
  }
  if ( p.position().y() < 0 || p.position().y() > height ) {
    p.velocity().set(p.velocity().x(), -0.9*p.velocity().y(), 0);
    //p.setVelocity( p.velocity().x(), -0.9*p.velocity().y(), 0 );
  }
  //p.moveTo( constrain( p.position().x(), 0, width ), constrain( p.position().y(), 0, height ), 0 ); 
  p.position().set( constrain( p.position().x(), 0, width ), constrain( p.position().y(), 0, height ), 0 ); 
}