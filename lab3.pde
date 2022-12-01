/// You do not need to change anything in this file, but you can
/// For example, if you want to add additional options controllable by keys
/// keyPressed would be the place for that.
import java.util.Random;

ArrayList<PVector> waypoints = new ArrayList<PVector>();
int lastt;
Random rand = new Random();
Map map = new Map();
boolean show_graph = false;
boolean auto_run = false;
Boid billy;

void setup() {
  size(800, 600);
  randomSeed(0);
  map.generate(-2);
  billy = new Boid(BILLY_START, BILLY_START_HEADING, BILLY_MAX_SPEED, BILLY_MAX_ROTATIONAL_SPEED, BILLY_MAX_ACCELERATION, BILLY_MAX_ROTATIONAL_ACCELERATION);
}


void keyPressed()
{
  if (key == 'g')
  {
    map.generate(rand.nextInt());
  } else if (key == 'n')
  {
    show_graph = !show_graph;
  } else if( key == 'r') {
      auto_run = !auto_run;
  }
}

void mousePressed() {
   if(mouseButton == LEFT) {
      billy.seek(new PVector(mouseX, mouseY));
   }
}


void draw() {
  background(0);

  float dt = (millis() - lastt)/1000.0;
  lastt = millis();

  if (show_graph) {
    stroke(255, 0, 0);
    for (Node n : map.map.stream().flatMap(row -> row.stream()).collect(Collectors.toList())) {
      circle(n.center.x, n.center.y, 5);
      for (Node neighbor : n.neighbors) {
        line(n.center.x, n.center.y, neighbor.center.x, neighbor.center.y);
      }
    }
  }
  
  if (auto_run && billy.target == null) {
     billy.seek(map.getRandomNode().center); 
  }

  map.update(dt);
  billy.update(dt);
}
