/// In this file, you will have to implement seek and waypoint-following
/// The relevant locations are marked with "TODO"

class Crumb
{
  PVector position;
  color c;
  Crumb(PVector position, color c)
  {
    this.position = position;
    this.c = c;
  }
  void draw()
  {
    fill(c);
    noStroke();
    circle(this.position.x, this.position.y, CRUMB_SIZE);
  }
}

class Boid
{
  Crumb[] crumbs = {};
  int last_crumb;
  float acceleration;
  float rotational_acceleration;
  KinematicMovement kinematic;
  PVector target;
  FlightPlan flightPlan;

  Boid(PVector position, float heading, float max_speed, float max_rotational_speed, float acceleration, float rotational_acceleration)
  {
    this.kinematic = new KinematicMovement(position, heading, max_speed, max_rotational_speed);
    this.last_crumb = millis();
    this.acceleration = acceleration;
    this.rotational_acceleration = rotational_acceleration;
  }

  void update(float dt)
  {

    if (target != null)
    {
      // TODO: Implement seek here

      // If the flight plan has not been calibrated yet, calibrate it. Initial step done once per flightplan
      if (!flightPlan.calibrated()) {
        flightPlan.calibrate(dt);
      }

      // General variables about the frame/settings
      final int SLOW_SPEED = 5;
      final int SLOW_RADIUS = 30; // The radius around the goal that we want to start slowing down in. Arbitrarily chosen
      final int END_RADIUS = 10;  // The radius around the goal that we consider as within the goal. Arbitrarily chosen
      float frame_max_acceleration = this.acceleration * dt;  // The maximum amount of acceleration we can do this frame
      float frame_rot_acceleration = this.rotational_acceleration * dt;  // The maximum amount of rotational acceleration we can do this frame

      // Variables related to the boid and target
      PVector target_vector = PVector.sub(kinematic.position, target);      // A vector from the boid to the target
      float distance_to_target = target_vector.mag();                        // The distance from the boid to the target
      float target_heading = atan2(target.y - kinematic.position.y, target.x - kinematic.position.x);  // The heading (radians) that the boid is from the target
      float direction_change = normalize_angle_left_right(target_heading - this.kinematic.heading); // The amount of radians we want to change our heading by (-PI to PI)

      // Visible elements drawn on the screen for more understanding
      fill(255, 153);
      circle(target.x, target.y, 2*SLOW_RADIUS);    // Draw the slowing radius
      fill(255, 100);
      circle(target.x, target.y, 2*END_RADIUS);     // Draw the goal radius
      line(kinematic.position.x, kinematic.position.y, target.x, target.y); // Draw line from boid to target
      text("Speed: " + this.kinematic.getSpeed(), 0, 10);



      /*
          General movement is handled in the following sections. It is broken up into two sections:
       1. Velocity
       2. Rotational velocity
       
       The first section only cares about changing the velocity, and the second the rotational velocity
       */


      if (distance_to_target < END_RADIUS) {
        // We are within the target range

        if ( flightPlan.hasNext()) {
          // If we have a target after this current one, select the next target. This way we don't slow down more than we want to and instead rely on the flightplan to decrease our speed to the necessary amount
          flightPlan.incrementTarget();
          this.target = flightPlan.getCurrentTarget();
        } else if (this.kinematic.getSpeed() >= 0.001) {
          // Prioritize stopping velocity over stopping rotation
          // Slow down as fast as we can
          this.kinematic.increaseSpeed(-min(this.kinematic.getSpeed(), frame_max_acceleration), 0);
        } else if (this.kinematic.getRotationalVelocity() >= 0.001) {
          // Slow down rotation as fast as we can
          this.kinematic.increaseSpeed(0, -min(this.kinematic.getRotationalVelocity(), frame_rot_acceleration));
        } else {
          // This is the last target and we've effectively stopped
          print("mission accomplished!\n");  // Print out success

          flightPlan.incrementTarget();      // Increment the flightPlan to the next target
          this.target = null;  // This is the last target, set target to null
        }
      } else if (distance_to_target < SLOW_RADIUS && !flightPlan.hasNext()) {
        // We are within the slowing radius and this is the last target. Attempt to slow to 5 speed as fast as possible. Speed of 5 is arbitrarily chosen
        if (this.kinematic.getSpeed() > SLOW_SPEED) {
          this.kinematic.increaseSpeed(-constrain(this.kinematic.getSpeed(), -frame_max_acceleration, frame_max_acceleration), 0);
        }
      } else {
        // We are nowhere near the goal, this is where the main acceleration is done

        Path current_path = flightPlan.getCurrentPath();  // Find what path we are on
        PVector desired_speed_v = current_path.getDesiredSpeed(current_path.getT(this.kinematic.position)); // Find the desired speed of the boid for this path given the current position
        float speed_error = max(desired_speed_v.y, SLOW_SPEED) - this.kinematic.getSpeed();                                  // Find the difference between the speed we want to be going and our current speed. If speed_error is positive, we want to be going faster than we currently are and vice versa
        kinematic.increaseSpeed(constrain(speed_error, -frame_max_acceleration, frame_max_acceleration), 0);  // Adjust our speed to match the speed_error, constrained to our maximum acceleration minimums

        // The rest of this block is for visual displays
        //println("max acc: " + frame_max_acceleration);  // Log the maximum acceleration allowed this frame
        //println("real acc: " + constrain(speed_error, -frame_max_acceleration, frame_max_acceleration));  // Log the acceleration taken this frame
        //println("speed error: " + speed_error); 
        // println("target: ", this.target);// Log the speed_error

        //this.crumbs = (Crumb[])append(this.crumbs, new Crumb(new PVector(desired_speed_v.x, 550 - constrain(speed_error, -frame_max_acceleration, frame_max_acceleration)*1000), color(255, 0, 0)));  // Acceleration this frame
        //this.crumbs = (Crumb[])append(this.crumbs, new Crumb(new PVector(desired_speed_v.x, 550 - frame_max_acceleration*1000), color(0, 255, 0)));  // Max Acc border
        //this.crumbs = (Crumb[])append(this.crumbs, new Crumb(new PVector(desired_speed_v.x, 550 + frame_max_acceleration*1000), color(0, 255, 0)));  // Min Acc border

        //this.crumbs = (Crumb[])append(this.crumbs, new Crumb(desired_speed_v, color(255, 255, 255)));  // Desired speed
        //this.crumbs = (Crumb[])append(this.crumbs, new Crumb(new PVector(desired_speed_v.x, this.kinematic.getSpeed()), color(155, 155, 155)));  // Actual speed
        //this.crumbs = (Crumb[])append(this.crumbs, new Crumb(new PVector(desired_speed_v.x, 500 - speed_error), color(255, 0, 0)));        // Error
        //stroke(155);
        //line(0, 500, 800, 500);
      }

      // Handle angle
      /*
       Angle is handled in a similar way to speed, but it is much easier.
       direction_change is the left to right amount (in radians) that we want to change our heading by.
       We map this to our maximum rotational speed this frame.
       Due to this mapping, if the target is directly behind the boid, it will want to rotate at the maximum speed possible this frame.
       If the target is quite close to the current heading, the boid will want to rotate slower.
       
       By using (desired_rot_speed - this.kinematic.getRotationalVelocity()) we don't have to deal with overshooting or manually setting values for how much to turn.
       Map creates a linear curve leading to 0 heading difference from both sides and by setting our goal to (desired - actual) we just follow that curve.
       */
      float desired_rot_speed = map(direction_change, -PI, PI, -this.kinematic.max_rotational_speed, this.kinematic.max_rotational_speed);
      kinematic.increaseSpeed(0, constrain(desired_rot_speed - this.kinematic.getRotationalVelocity(), -frame_rot_acceleration, frame_rot_acceleration)*5);
    }

    // place crumbs, do not change
    if (LEAVE_CRUMBS && (millis() - this.last_crumb > CRUMB_INTERVAL))
    {
      this.last_crumb = millis();
      this.crumbs = (Crumb[])append(this.crumbs, new Crumb(this.kinematic.position, color(255, 255, 255)));
      if (this.crumbs.length > MAX_CRUMBS)
        this.crumbs = (Crumb[])subset(this.crumbs, 1);
    }

    // do not change
    this.kinematic.update(dt);

    draw();
  }

  void draw()
  {
    for (Crumb c : this.crumbs)
    {
      c.draw();
    }

    fill(255);
    noStroke();
    float x = kinematic.position.x;
    float y = kinematic.position.y;
    float r = kinematic.heading;
    circle(x, y, BOID_SIZE);
    // front
    float xp = x + BOID_SIZE*cos(r);
    float yp = y + BOID_SIZE*sin(r);

    // left
    float x1p = x - (BOID_SIZE/2)*sin(r);
    float y1p = y + (BOID_SIZE/2)*cos(r);

    // right
    float x2p = x + (BOID_SIZE/2)*sin(r);
    float y2p = y - (BOID_SIZE/2)*cos(r);
    triangle(xp, yp, x1p, y1p, x2p, y2p);
    
    if (this.target != null) {
       circle(this.flightPlan.getLastTarget().x, this.flightPlan.getLastTarget().y, 20); 
    }
  }

  void seek(PVector target)
  {
    /*
     When only one target is specified waypoints is empty. Since we want to use waypoints in either follow() or seek() we add the target to waypoints in seek().
     To accomodate for follow() being called, that method removes the last waypoint in the list so it can be re-added here
     */
     // Using the aStar function to set a list of waypoints based off astar search 
    ArrayList<PVector> pathFollowing = new ArrayList<PVector>();
    pathFollowing = map.aStar(map.getNodeClosestToPoint(this.kinematic.getPosition()), map.getNodeClosestToPoint(target));
     
    //waypoints = nm.aStar(kinematic.position, target);44
    flightPlan = new FlightPlan(this, pathFollowing);
    this.target = flightPlan.getCurrentTarget();  // Assign the target to the initial target on the FlightPlan
    waypoints = new ArrayList<PVector>();
  }

}

/*
 This class is the crux of the flight plan idea. Since all waypoints are known at the time of calling seek() or follow(),
 they could be integrated into some class that contains both the waypoints and a greater amount of information such as the angles between each edge.
 Ultimately the motivation was the idea that we could precompute a desired speed for the boid at any point along the path and simply instruct
 the boid to get as close to that desired speed as possible. Precalculating this information might enable behavior that is more informed about what to do
 and can path find in a more efficient manner.
 
 This class contains the collection of edges that make up the overall path, but actual implementation of determining the desired speed is relegated to the Path class below.
 
 */
class FlightPlan {
  ArrayList<Path> paths;  // The list of paths contained in the overall flight plan
  int pathIndex;          // the current path we are on
  boolean calibrated;    // true if we have called FlightPlan.calibrate() to determine the maximum speed along each Path

  /*
    Constructor for flightplan. Generates paths and determines speeds around corners
   
   @param b             the boid that owns this flightplan
   @param waypoints     the waypoints to include in this flightplan
   
   SIDE-EFFECTS: sets this.paths, and assigns values to the created paths. Starts this.pathIndex at 0
   */
  FlightPlan(Boid b, ArrayList<PVector> waypoints) {
    paths = new ArrayList<Path>();
    // Iterate through waypoints, creating paths based on the starting and ending points, assigning start/end speeds of zero by default
    for (int point_index = 0; point_index < waypoints.size(); point_index++) {
      PVector point = waypoints.get(point_index);
      PVector previousPoint = point_index == 0 ? b.kinematic.position : waypoints.get(point_index - 1);
      paths.add(new Path(previousPoint, 0, point, 0, b.kinematic.max_speed));
    }

    // For every consecutive pair of paths, determine the desired speed at each corner. Calibration must be called after this step and not before
    for (Path currPath : this.paths) {
      if (this.paths.indexOf(currPath) == 0) {
        continue;
      }
      Path prevPath = this.paths.get(this.paths.indexOf(currPath) - 1);

      println(180 - degrees(PVector.angleBetween(prevPath.getPath(), currPath.getPath())));
      float corner_angle = 180 - degrees(PVector.angleBetween(prevPath.getPath(), currPath.getPath())); // Find the inner angle of the corner
      float hard_cap = constrain(sqrt(2*1*currPath.getDistance()), 0, BILLY_MAX_SPEED);        // Hard cap on the speed of the corner. Figure out the theoretical maximum speed we could be going at this corner and still come to a dead stop at the next waypoint. Constrain to the boids max movement speed as well
      float corner_speed = map(corner_angle, 0, 180, 0, hard_cap);        // Limit the speed based on the angle of the corner. If the angle is a straight through 180 degrees allow full speed, if an about face then come to a complete stop. Interpolate between the two. The main purpose of this is to avoid overshoot on a tighter angle
      println("corner_speed: " + corner_speed);

      // Set corresponding start and end speeds
      prevPath.setEndSpeed(corner_speed);
      currPath.setStartSpeed(corner_speed);
    }
    pathIndex = 0;
  }

  // Get the current target of the boid, the end of the current path
  PVector getCurrentTarget() {
    return paths.get(pathIndex).getEnd();
  }

  // Get the current path of the flightplan
  Path getCurrentPath() {
    return paths.get(pathIndex);
  }

  // Get the last target of the flightplan, the starting point of the current path. Currently not used by anything
  PVector getLastTarget() {
    return paths.get(paths.size() - 1).getEnd();
  }

  // Increment the path the flightplan is on
  int incrementTarget() {
    return ++pathIndex;
  }

  // Check if the flightplan is done, if there are no more paths
  boolean isComplete() {
    return pathIndex >= paths.size();
  }

  // Check if there are more paths
  boolean hasNext() {
    return pathIndex + 1 < paths.size();
  }

  boolean calibrated() {
    return this.calibrated;
  }

  /*
    Go through each path and calibrate them for maximum speed
   
   @param dt     the dt value for the frame calibrate() is called on
   
   SIDE-EFFECT: modifies the values of the Path's in this.paths, sets this.calibrated to true
   */
  void calibrate(float dt) {
    for (Path path : paths) {
      path.calibrate(dt);
    }
    this.calibrated = true;
  }
}

/*
 General Information:
 This helper class defines an edge of the graph obtained by drawing the lines between the waypoints
 It's purpose is to store data about this path so that further information can be calculated and so information about this path can be easily obtained
 
 Implementation details:
 The current iteration of Path employs a Quadratic Bezier Curve to inform the desired speed. The motivation was that when decelerating to the goal the
 boid ought to have a planned path that abides by the maximum acceleration speed. If the planned path does not follow these limits, the boid would very
 likely overshoot the goal by 'believing' it can stop sooner than it can actually stop.
 
 Initially a linear speed curve was used, but this resulted in lethargic acceleration (34828d6).
 The idea behind the use of a bezier curve was that it would maintain a higher speed longer, or at the very least look more fluid.
 Somewhat rough checks are done to ensure the acceleration along the curve is not outside of the maximum allowed values (the maximum speed along the curve
 is decreased until an allowable curve is generated)(1d12722).
 This option was further modified so that prior to the apex of the bezier curve the maximum speed allowed is the value at the apex of the curve. After the
 apex of the curve, the boid begins following the bezier curve. This was done with the idea that the speed at the apex is the maximum speed that the boid can
 slow down to the goal speed from, and so it does not matter what acceleration is done prior to the apex so long as the speed does not exceed that apex value.
 
 NOTE: POTENTIAL TODO for all calculations regarding the bezier curve, p2 is considered to be the midpoint between start and end. I'm unsure, but this may be a naive assumption
 that could cause some error in performance. It may be trivial error, but if issues arise some thought should be put into this.
 */
class Path {
  PVector start;      // The starting location of the path
  float start_speed;  // The desired speed of the boid at the start of the path
  PVector end;        // The ending location of the path
  float end_speed;    // The desired speed of the boid at the end of the path. This value is
  PVector path;        // A vector from the start to the end of the path
  float distance;    // The distance of the path
  float max_speed;      // The maximum speed that is allowed to occur along this path
  float t_of_highest_speed;    // The t value at the point of the highest speed

  /*
    Constructor for the Path class
   
   @param start         a PVector that is the point that the path starts at
   @param start_speed   speed at which the boid should be at the start of the path
   @param end           a PVector that is the point that the path ends at
   @param end_speed     speed at which the boid shoul dbe at the end of the path
   @param max_speed     the maximum speed that should occur along the path. Note: this is no guarantee that the path will reach this speed, just that the speed will never exceed this speed
   */
  Path(PVector start, float start_speed, PVector end, float end_speed, float max_speed) {
    this.start = start;
    this.start_speed = start_speed;
    this.end = end;
    this.end_speed = end_speed;
    this.path = PVector.sub(end, start);
    this.distance = this.path.mag();
    this.max_speed = max_speed;
  }

  PVector getStart() {
    return start;
  }
  float getStartSpeed() {
    return start_speed;
  }
  PVector getEnd() {
    return end;
  }
  float getEndSpeed() {
    return end_speed;
  }
  PVector getPath() {
    return path;
  }
  float getDistance() {
    return distance;
  }

  void setEndSpeed(float end_speed) {
    this.end_speed = end_speed;
  }

  void setStartSpeed(float start_speed) {
    this.start_speed = start_speed;
  }

  /*
    Function to get the t-value of the boid along the path. The t-value is essentially the progress of completion of the boid along the path.
   0.0 < t < 1.0 where 0.0 means the boid is at the start of the path and 1.0 means the boid is at the end. This value is used for calculations
   involving the bezier curve. NOTE: the returned t-value is not strictly the progress along the path,  as it is based only on the distance to the end.
   
   @param currentPos     the current position of the boid
   @return               a float value that is between 0 and 1 representing path completion
   */
  float getT(PVector currentPos) {
    return PVector.dist(currentPos, this.start) / this.distance;
  }

  /*
   Calculates and returns what speed the boid should have at a given point along the path
   
   @param t     a float value 0 < t < 1 that represents completion along the path
   @return      a PVector where PVector.y is the desired speed and PVector.x is the distance along the path that speed corresponds to
   */
  PVector getDesiredSpeed(float t) {
    float actual_t = t;
    t += .001;  // small increment because passing a t-value of 0.0 causes issues with the calculation



    // If the current t value is lower than the t value of the apex of the curve, default to the t-value of the highest speed of the curve
    if (t < this.t_of_highest_speed) {
      t = this.t_of_highest_speed;
    }
    PVector p1 = new PVector(0, this.start_speed);
    PVector p2 = new PVector(this.distance / 2, this.max_speed);
    PVector p3 = new PVector(this.distance, this.end_speed);

    PVector pd1 = PVector.add(PVector.mult(p1, 1 - t), PVector.mult(p2, t));
    PVector pd2 = PVector.add(PVector.mult(p2, 1 - t), PVector.mult(p3, t));

    PVector pf = PVector.add(PVector.mult(pd1, 1 - t), PVector.mult(pd2, t));
    //this.drawCurve(p1, p2, p3);

    pf.x = this.distance * actual_t; // correct to use real value of t incase we're in the first half of the trip

    return pf;
  }

  /*
   A function that finds the apex/highest speed of the bezier curve. Used to determine at what point the boid should switch to bezier deceleration
   
   @return     a PVector that is the point of the apex of the curve. PVector.x is the distance it occurs at along the path and PVector.y is the speed at that point.
   */
  PVector findHighestPoint() {
    PVector p1 = new PVector(0, this.start_speed);
    PVector p2 = new PVector(this.distance / 2, this.max_speed);
    PVector p3 = new PVector(this.distance, this.end_speed);

    PVector highest = new PVector(0, 0); // A variable to store the highest point found yet

    // Iterate through the curve at 0.01 increments and record the highest found point. Note: Due to the inclusion of non-zero start/end speeds we cannot
    // just search from 0.0 to 0.5, as the apex may not be at the centerpoint
    for (float t = 0.0; t < 1.0; t += 0.01) {
      PVector pd1 = PVector.add(PVector.mult(p1, 1 - t), PVector.mult(p2, t));
      PVector pd2 = PVector.add(PVector.mult(p2, 1 - t), PVector.mult(p3, t));

      PVector pf = PVector.add(PVector.mult(pd1, 1 - t), PVector.mult(pd2, t));

      PVector acc_vector = PVector.add(PVector.mult(PVector.sub(p2, p1), 2 * (1 - t)), PVector.mult(PVector.sub(p3, p2), 2 * t)); // Vector containing the acceleration at the given point, calculated using the derivative formula
      float acc = atan2(acc_vector.y, acc_vector.x);

      if (pf.y > highest.y) {
        highest = new PVector(t, pf.y);
      }
    }

    return highest;
  }

  /*
   This function calibrates the path, which entails finding the maximum allowed speed along the path and the highest point of the curve.
   The maximum speed is determined by finding the first value which does not contain an acceleration outside of our estimated frame acceleration
   
   @param dt     the change in time since the last frame. This value is just one value of dt, and so in reality during execution
   of pathfinding the real value of dt may be slightly different. Boid acceleration is constrained on a per frame basis, so this diversion
   of what is planned vs what can be done will not cause 'illegal' movement, but will create error in the speed of the boid
   
   As a consequence, while this method does not return anything it contains two SIDE-EFFECTS:
   1. it overrides max_speed to be the maximum speed that keeps the acceleration within bounds
   2. it assigns t_of_highest_speed to the t-value that corresponds to the maximum speed
   
   NOTE: when finding the max_speed it only considers the maximum speed at the midpoint, but then finds the highest point overall.
   NOTE: currently the acceleration threshold is set to 0.12, I believe this was found by looking at frame_max_acceleration values during actual executions but it is somewhat arbitrary.
   Using dt to estimate frame_max_acc was attempted but yielded oddly low results, possibly due to dt being calculated when nothing else is occuring and dt is therefore much smaller.
   To that end, dt is not currently used.
   */
  void calibrate(float dt) {
    float temp_speed;

    PVector p1 = new PVector(0, this.start_speed); // Starting speed of the path

    PVector p3 = new PVector(this.distance, this.end_speed);

    // Continue until either a valid max_speed is found or temp_speed is negative.
    // NOTE: it is assumed that there will be a temp_speed that will abide the acc value, but if there is one that is sufficiently low it will likely be painfully slow
    for (temp_speed = BILLY_MAX_SPEED; temp_speed > 0.0; temp_speed -= 0.1) {
      PVector p2 = new PVector(this.distance / 2, temp_speed);

      if (maxAccelerationIsBelow(p1, p2, p3, 0.12)) {
        break;
      }
    }

    this.max_speed = temp_speed;
    println("max_speed: " + temp_speed);

    this.t_of_highest_speed = findHighestPoint().x;
  }

  /*
      Function that tests to see if for a given bezier curve there is an acceleration that is larger than a given value.
   
   @param p1          the first point of a quadratic bezier curve
   @param p2          the second point of a quadratic bezier curve
   @param p3          the third point of a quadratic bezier curve
   @param max_acc     the maximum allowed acceleration on this curve
   @return            true if the max_acc is not exceeded, false otherwise
   
   POTENTIAL TODO: Look into formula based way to find maximum acceleration, second derivative? Efficiency and accuracy improvements would likely be negligible
   */
  boolean maxAccelerationIsBelow(PVector p1, PVector p2, PVector p3, float max_acc) {
    // For every temp_speed we examine in 0.01 increments the curve generated and search for an acceleration outside of what we can do
    for (float t = 0.05; t < 1.0; t += 0.01) {
      PVector acc_vector = PVector.add(PVector.mult(PVector.sub(p2, p1), 2 * (1 - t)), PVector.mult(PVector.sub(p3, p2), 2 * t));
      float acc = atan2(acc_vector.y, acc_vector.x);


      // If the acceleration value at this point in the curve is too high, lower temp_speed and try again
      if (abs(acc) > max_acc) {
        return false;
      }
    }

    return true;
  }

  /*
    A function to draw a quadratic bezier curve based on the three points. Used only for illustrative purposes
   */
  void drawCurve(PVector p1, PVector p2, PVector p3) {
    // from https://processing.org/reference/bezierTangent_.html
    //float ft = 0.25;
    PVector cp0 = p1;
    PVector cp1 = PVector.add(p1, PVector.mult(PVector.sub(p2, p1), 0.66));
    PVector cp2 = PVector.add(p3, PVector.mult(PVector.sub(p2, p3), 0.66));
    PVector cp3 = p3;

    bezier(cp0.x, cp0.y, cp1.x, cp1.y, cp2.x, cp2.y, cp3.x, cp3.y);

    ////println("tangent: " + bezierTangent(cp0.x, cp1.x, cp2.x, cp3.x, ft));

    //float x = bezierPoint(cp0.x, cp1.x, cp2.x, cp3.x, t);
    //float y = bezierPoint(cp0.y, cp1.y, cp2.y, cp3.y, t);
    //// Get the tangent points
    //float tx = bezierTangent(cp0.x, cp1.x, cp2.x, cp3.x, t);
    //float ty = bezierTangent(cp0.y, cp1.y, cp2.y, cp3.y, t);
    ////println("tx, ty: " + tx + ", " + ty);
    //// Calculate an angle from the tangent points
    //float a = atan2(ty, tx);
    //println("a " + a);
    //a += PI;
    //stroke(255, 102, 0);
    //line(x, y, cos(a)*120 + x, sin(a)*120 + y);
  }
}
