//This is a modified version of the Flocking example by Daniel Shiffman listed on the Processing website http://www.processing.org/examples/flocking.html
//This version assigns a leader that has more 'weight' in the flock.
//The leader tends not to follow the flock but rather set the pace for it.



Flock flock;
int LEADERWEIGHT = 10000;
int LIEUTENANTWEIGHT = LEADERWEIGHT /3 ; // Not used yet
int simulationtime =0;
int countOfLeaders = 0;
int reignLength = 500;
float maxSpeedRegular = 3;
float maxSpeedLeader = 4.01;

float maxForceRegular = 0.09;
float maxForceLeader = 0.1;

float max=0;


void setup() {
  size(1080, 720);
  flock = new Flock();
  // Add an initial set of boids into the system
  for (int i = 0; i < 150; i++) {
    flock.addBoid(new Boid(width/2, height/2));
  }
}

void draw() {
  background(50);
  flock.run();
  simulationtime++;
}

// Add a new boid into the System
void mousePressed() {
  flock.addBoid(new Boid(mouseX, mouseY));
}



// The Boid class

class Boid {

  PVector location;
  PVector velocity;
  PVector acceleration;
  PVector gravityCenter;
  float r;   // size of the plane
  float maxforce;    // Maximum steering force
  float maxspeed;    // Maximum speed
  boolean leader; 
  float mass;

  Boid(float x, float y) 
  {
    acceleration = new PVector(0, 0);

    // This is a new PVector method not yet implemented in JS
    // velocity = PVector.random2D();

    // Leaving the code temporarily this way so that this example runs in JS
    float angle = random(TWO_PI);
    velocity = new PVector(cos(angle), sin(angle));
    location = new PVector(x, y);
    maxspeed = maxSpeedRegular;
    maxforce = maxForceRegular;
    mass = random(2, 7);
    r = 2.0 * mass/4;
  }


  void abdicate()
  {
    if (this.leader == true)
    {
      this.leader = false;
      countOfLeaders--;
      this.maxspeed = maxSpeedRegular;
      this.maxforce = maxForceRegular;
    }
  }

  void elect()
  {
    if (this.leader == false)
    {
      this.leader = true;
      countOfLeaders++;
      this.maxspeed = maxSpeedLeader;
      this.maxforce = maxForceLeader;
    }
  }

  void run(ArrayList<Boid> boids) {

    if (countOfLeaders<5)
    {
      elect();
    }
    else
      if (simulationtime % reignLength == 0)
      {
        abdicate();
      }

    flock(boids);
    update();
    borders();
    render();
  }


  void applyForce(PVector force) {
    // We could add mass here if we want A = F / M - UPDATE: we just did.
    acceleration.add(force);
    acceleration.mult(1/mass);
  }

  // We accumulate a new acceleration each time based on three rules
  void flock(ArrayList<Boid> boids) {
    PVector sep = separate(boids);   // Separation
    PVector ali = align(boids);      // Alignment
    PVector coh = cohesion(boids);   // Cohesion
    // Arbitrarily weight these forces
    sep.mult(1.8);
    ali.mult(1.5);
    coh.mult(1.0);
    // Add the force vectors to acceleration
    applyForce(ali);
    applyForce(sep); // the leader tries to maintain a distance from other other boids
    applyForce(coh);
  }

  // Method to update location
  void update() {
    // Update velocity
    velocity.add(acceleration);
    // Limit speed
    velocity.limit(maxspeed);
    location.add(velocity);
    // Reset accelertion to 0 each cycle
    acceleration.mult(0);
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, location);  // A vector pointing from the location to the target
    // Scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);

    // Above two lines of code below could be condensed with new PVector setMag() method
    // Not using this method until Processing.js catches up
    // desired.setMag(maxspeed);

    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);

    steer.limit(maxforce);  // Limit to maximum steering force
    return steer;
  }

  void render() {
    // Draw a triangle rotated in the direction of velocity
    float theta = velocity.heading2D() + radians(90);
    // heading2D() above is now heading() but leaving old syntax until Processing.js catches up

    if (this.leader == true) // Leader is red
    {
      stroke(255, 0, 0);
      fill(255, 0, 0);
    }
    else
    {
      stroke(255);
      fill(200, 100);
    }
    pushMatrix();
    translate(location.x, location.y);
    rotate(theta);
    beginShape(TRIANGLES);
    vertex(0, -r*2);
    vertex(-r, r*2);
    vertex(r, r*2);
    endShape();
    popMatrix();
  }

  // Wraparound
  void borders() {
    if (location.x < -r) location.x = width+r;
    if (location.y < -r) location.y = height+r;
    if (location.x > width+r) location.x = -r;
    if (location.y > height+r) location.y = -r;
  }

  // Separation
  // Method checks for nearby boids and steers away
  PVector separate (ArrayList<Boid> boids) {
    float desiredseparation = 25.0f;
    PVector steer = new PVector(0, 0, 0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);

      if (other.leader == true)
      {
        desiredseparation = 50.0f; // try to stick closer to the leader than you would to other boids.
      }
      else
      {
        desiredseparation = 100.0f;
      }
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor

        PVector diff = PVector.sub(location, other.location);
        diff.normalize();
        diff.div(d);        // Weight by distance
        steer.add(diff);
        count++;            // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // First two lines of code below could be condensed with new PVector setMag() method
      // Not using this method until Processing.js catches up
      // steer.setMag(maxspeed);

      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    return steer;
  }

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  PVector align (ArrayList<Boid> boids) {
    float neighbordist = 50;
    PVector sum = new PVector(0, 0);
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.velocity);
        count++;
        if (other.leader == true)
        {
          for (int i=0; i<LEADERWEIGHT;i++)
          {
            sum.add(other.velocity);
            count++;
          }
        }
      }
    }
    if (count > 0) {
      sum.div((float)count);
      // First two lines of code below could be condensed with new PVector setMag() method
      // Not using this method until Processing.js catches up
      // sum.setMag(maxspeed);

      // Implement Reynolds: Steering = Desired - Velocity
      sum.normalize();
      sum.mult(maxspeed);
      PVector steer = PVector.sub(sum, velocity);
      steer.limit(maxforce);
      return steer;
    } 
    else {
      return new PVector(0, 0);
    }
  }

  // Cohesion
  // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
  PVector cohesion (ArrayList<Boid> boids) {
    float neighbordist = 50;
    PVector sum = new PVector(0, 0);   // Start with empty vector to accumulate all locations
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.location); // Add location
        count++;
        if (other.leader == true)
        {
          for (int i=0; i<LEADERWEIGHT;i++)
          {
            sum.add(other.location);
            count++;
          }
        }
      }
    }
    if (count > 0) {
      sum.div(count);
      return seek(sum);  // Steer towards the location
    } 
    else {
      return new PVector(0, 0);
    }
  }
}




// The Flock (a list of Boid objects)

class Flock {
  ArrayList<Boid> boids; // An ArrayList for all the boids

    Flock() 
  {
    boids = new ArrayList<Boid>(); // Initialize the ArrayList
  }

  void run() 
  {
    for (Boid b : boids)
    {
      b.run(boids);  // Passing the entire list of boids to each boid individually
    }
  }

  void addBoid(Boid b) 
  {
    boids.add(b);
  }
}

