import java.util.Random;
import java.util.Objects;
import java.util.HashSet;

class Wall
{
  PVector start;
  PVector end;
  PVector normal;
  PVector direction;
  float len;

  Wall(PVector start, PVector end)
  {
    this.start = start;
    this.end = end;
    direction = PVector.sub(this.end, this.start);
    len = direction.mag();
    direction.normalize();
    normal = new PVector(-direction.y, direction.x);
  }

  // Return the mid-point of this wall
  PVector center()
  {
    return PVector.mult(PVector.add(start, end), 0.5);
  }

  void draw()
  {
    strokeWeight(3);
    line(start.x, start.y, end.x, end.y);
    if (SHOW_WALL_DIRECTION)
    {
      PVector marker = PVector.add(PVector.mult(start, 0.2), PVector.mult(end, 0.8));
      circle(marker.x, marker.y, 5);
    }
  }
   
  @Override
    boolean equals(Object other) {
    if (this == other) {
      return true;
    } else if (!this.getClass().equals(other.getClass())) {
      return false;
    } else {
      Wall other_wall = (Wall) other;
      return (this.start.equals(other_wall.start) && this.end.equals(other_wall.end)) || (this.start.equals(other_wall.end) && this.end.equals(other_wall.start));
    }
  }
  
  @Override
  int hashCode() {
    return Objects.hash(this.center()); // I recognize this is not a great way to do this, but for the moment it works.
  }  
}

class Node
{
  PVector center;
  boolean visited = false;
  ArrayList<Node> neighbors = new ArrayList<Node>();

  Node(PVector center) {
    this.center = center;
  }

  public void visit() {
    this.visited = true;
  }

  public void addNeighbor(Node neighbor) {
    neighbors.add(neighbor);
  }
}


class Map
{
  HashSet<Wall> walls;
  ArrayList<Node> nodes;
  ArrayList<ArrayList<Node>> map;
  int cellsWide;
  int cellsTall;

  Map()
  {

    walls = new HashSet<Wall>();
    nodes = new ArrayList<Node>();
  }



  void generate(int which)
  {
    cellsWide = 800 / GRID_SIZE;
    cellsTall = 600 / GRID_SIZE;
    println(cellsWide + " by " + cellsTall);


    walls.clear();

    Random rand = new Random(which);

    /**
      Initialize Node Map
    **/
    map = new ArrayList<ArrayList<Node>>();

    // Initialize map to the correct size
    for (int row = 0; row < cellsTall; row++) {
      map.add(new ArrayList<Node>(cellsWide));
    }

    // Populate map with nodes
    for (int row = 0; row < cellsTall; row++) {
      for (int col = 0; col < cellsWide; col++) {
        int centerOffset = GRID_SIZE / 2;
        int adjustedColPos = col * GRID_SIZE;
        int adjustedRowPos = row * GRID_SIZE;
        PVector nodeCenter = new PVector(adjustedColPos + centerOffset, adjustedRowPos + centerOffset);
        map.get(row).add(new Node(nodeCenter));
        
        // Add the walls, hashset will prevent duplicates
        walls.add(new Wall(new PVector(adjustedColPos, adjustedRowPos), new PVector(adjustedColPos + GRID_SIZE, adjustedRowPos)));
        walls.add(new Wall(new PVector(adjustedColPos, adjustedRowPos), new PVector(adjustedColPos, adjustedRowPos + GRID_SIZE)));
        walls.add(new Wall(new PVector(adjustedColPos + GRID_SIZE, adjustedRowPos + GRID_SIZE), new PVector(adjustedColPos + GRID_SIZE, adjustedRowPos)));
        walls.add(new Wall(new PVector(adjustedColPos + GRID_SIZE, adjustedRowPos + GRID_SIZE), new PVector(adjustedColPos, adjustedRowPos + GRID_SIZE)));
      }
    }

    //
    
    ArrayList<Wall> wallFrontier = new ArrayList<Wall>(); // TODO: figure out how to populate
    //for (int row = 0; row < cellsTall; row++) {
    //  for (int col = 0; col < cellsWide; col++) {
    //    Node curr = map.get(row).get(col);
    //  }
    //}
    
    
    int startCell = rand.nextInt(cellsWide * cellsTall);
  }

  void update(float dt)
  {
    draw();
  }

  void draw()
  {
    stroke(255);
    strokeWeight(3);
    for (Wall w : walls)
    {
      w.draw();
    }

    stroke(255, 0, 0);
    for (ArrayList<Node> row : map) {
      for (Node n : row) {
         circle(n.center.x, n.center.y, 10);
      }
    }
    
    for (Wall wall : walls) {
       wall.draw(); 
    }
    println(walls.size());
  }
}
