/// You do not need to change anything in this file, but you can
/// For example, if you want to add additional options controllable by keys
/// keyPressed would be the place for that.
import java.util.Random;

ArrayList<PVector> waypoints = new ArrayList<PVector>();
int lastt;
Random rand = new Random();

Map map = new Map();

void setup() {
  size(800, 600);
  randomSeed(0);
  map.generate(-2);
}


void keyPressed()
{
    if (key == 'g')
    {
       map.generate(rand.nextInt());
    }
}


void draw() {
  background(0);

  float dt = (millis() - lastt)/1000.0;
  lastt = millis();
  
  map.update(dt);  
}
