import java.util.Random; //<>// //<>//
import java.util.Objects;
import java.util.HashSet;
import java.util.HashMap;
import java.util.Arrays;
import java.util.stream.Collectors;
import java.util.Set;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Comparator;
import java.util.Collections;

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

  // The equals and hashCode methods are taken from Lab1 for the use of a HashSet
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
    return Objects.hash(this.center());
  }
}

static class Node
{
  PVector center;
  boolean visited = false;
  ArrayList<Node> neighbors = new ArrayList<Node>();
  ArrayList<Wall> walls = new ArrayList<>();

  Node(PVector center, ArrayList<Wall> walls) {
    this.center = center;
    this.walls = walls;
  }

  public void visit() {
    this.visited = true;
  }

  // Add another node to this nodes neighbor list
  public void addNeighbor(Node neighbor) {
    neighbors.add(neighbor);
  }

  // Have two nodes add each other as neighbors
  static void associate(Node a, Node b) {
    a.addNeighbor(b);
    b.addNeighbor(a);
  }

  public ArrayList<Wall> getWalls() {
    return this.walls;
  }
}


class Map
{
  HashSet<Wall> walls;
  ArrayList<Node> nodes;
  ArrayList<ArrayList<Node>> map;
  int cellsWide;
  int cellsTall;
  HashMap<PVector, Node> coordsToNode;  // Hashmap that can be used to find a node from it's center position

  Map()
  {

    walls = new HashSet<Wall>();
    nodes = new ArrayList<Node>();
    coordsToNode = new HashMap<PVector, Node>();
  }



  void generate(int which)
  {
    cellsWide = width / GRID_SIZE;
    cellsTall = height / GRID_SIZE;


    walls.clear();

    Random rand = new Random(which);

    /**
     Initialize Node Map
     **/
    map = new ArrayList<ArrayList<Node>>();
    coordsToNode = new HashMap<PVector, Node>();
    walls = new HashSet<Wall>();

    // Initialize map to the correct size
    for (int row = 0; row < cellsTall; row++) {
      map.add(new ArrayList<Node>(cellsWide));
    }

    // Populate map with nodes
    for (int row = 0; row < cellsTall; row++) {
      for (int col = 0; col < cellsWide; col++) {
        int centerOffset = GRID_SIZE / 2;
        int adjustedColPos = col * GRID_SIZE;  // X position of left side of grid cell
        int adjustedRowPos = row * GRID_SIZE;  // Y position of top side of grid cell
        PVector nodeCenter = new PVector(adjustedColPos + centerOffset, adjustedRowPos + centerOffset);

        // Create walls bordering this node
        Wall up = new Wall(new PVector(adjustedColPos, adjustedRowPos), new PVector(adjustedColPos + GRID_SIZE, adjustedRowPos));
        Wall left = new Wall(new PVector(adjustedColPos, adjustedRowPos), new PVector(adjustedColPos, adjustedRowPos + GRID_SIZE));
        Wall right = new Wall(new PVector(adjustedColPos + GRID_SIZE, adjustedRowPos + GRID_SIZE), new PVector(adjustedColPos + GRID_SIZE, adjustedRowPos));
        Wall down = new Wall(new PVector(adjustedColPos + GRID_SIZE, adjustedRowPos + GRID_SIZE), new PVector(adjustedColPos, adjustedRowPos + GRID_SIZE));


        walls.addAll(new ArrayList<>(Arrays.asList(up, left, right, down))); // Add the walls, hashset will prevent duplicates

        Node newNode = new Node(nodeCenter, new ArrayList<>(Arrays.asList(up, left, right, down))); // Create node and tell it which walls it has

        map.get(row).add(newNode); // Add node to map

        coordsToNode.put(nodeCenter, newNode); // Add node to hashmap
      }
    }


    /*
     Maze Generation using Prim's Algorithm
     */

    // Setup start node
    Node startNode = map.get(rand.nextInt(cellsTall)).get(rand.nextInt(cellsWide));  // The node we will start with
    startNode.visit();                                                               // Declare our start node visited

    // Setup frontier
    Set<Wall> wallFrontier = new HashSet<Wall>();  // Our frontier of walls
    wallFrontier.addAll(startNode.getWalls());     // Add the start nodes walls to the frontier

    // While there are unvisited nodes in the map
    while (map.stream().flatMap(row -> row.stream()).filter(n -> n.visited == false).collect(Collectors.toList()).size() > 0) {
      wallFrontier = pruneWallFrontier(wallFrontier); // Prune the frontier to exclude border walls and walls we can no longer remove

      // Find a valid wall
      Wall chosenWall = new ArrayList<>(wallFrontier).get(rand.nextInt(wallFrontier.size())); // Choose a wall from the frontier
      ArrayList<Node> wallNeighbors = getWallNodes(chosenWall, coordsToNode);                 // Get the neighbors of the wall

      // Get our new node and visit it
      Node freshlyVisitedNode = wallNeighbors.get(0).visited ? wallNeighbors.get(1) : wallNeighbors.get(0); // Figure out which neighbor of the wall we haven't visited yet
      freshlyVisitedNode.visit();  // Visit the neighbor

      wallFrontier.addAll(freshlyVisitedNode.getWalls());      // Add the new node's walls to frontier
      wallFrontier.remove(chosenWall); // Remove the wall we came from. Because wallFrontier is a set, we don't have to worry about the wall we added above being duplicate
      walls.remove(chosenWall);  // Remove the wall from our map

      Node.associate(wallNeighbors.get(0), wallNeighbors.get(1));  // Associate the nodes to each other now that the wall is gone
    }
  }

  // Get the nodes a wall seperates
  ArrayList<Node> getWallNodes(Wall wall, HashMap<PVector, Node> coordsToNode) {
    // Figure out if the wall is vertical or horizontal
    boolean verticalWall = Float.compare(wall.normal.x, 0.0) != 0 ;
    boolean horizontalWall = Float.compare(wall.normal.y, 0.0) != 0 ;


    // Find center of the nodes we're looking for
    PVector node1Coord = null;
    PVector node2Coord = null;

    if (horizontalWall) {
      //println("wall " + wall.center() + " is h");
      node1Coord = PVector.sub(wall.center(), new PVector(0, GRID_SIZE / 2));
      node2Coord = PVector.add(wall.center(), new PVector(0, GRID_SIZE / 2));
    } else if (verticalWall) {
      //println("wall " + wall.center() + " is v");
      node1Coord = PVector.sub(wall.center(), new PVector(GRID_SIZE / 2, 0));
      node2Coord = PVector.add(wall.center(), new PVector(GRID_SIZE / 2, 0));
    } else {
      // Floating point number comparison is oh so fun
      println("PAIN PAIN PAIN");
      println(wall.normal);
    }

    // Find nodes based on their center
    Node node1 = coordsToNode.get(node1Coord);
    Node node2 = coordsToNode.get(node2Coord);

    return new ArrayList<>(Arrays.asList(node1, node2));
  }

  Set<Wall> pruneWallFrontier(Set<Wall> walls) {
    // Prune the wallFrontier to exclude borders. Filter wallFrontier to only include walls which have two non null node neighbors
    Set<Wall> prunedWalls = walls.stream().filter(
      wall -> getWallNodes(wall, coordsToNode).stream().filter(
      node -> node != null).collect(Collectors.toList())
      .size() == 2).collect(Collectors.toSet());

    // Prune the wallFrontier to exclude walls we can not remove. Filter wallFrontier to only include walls which have 1 visited and 1 non-visited node
    prunedWalls = prunedWalls.stream().filter(
      wall -> getWallNodes(wall, coordsToNode).stream().filter(
      node -> node.visited).collect(Collectors.toList())
      .size() == 1).collect(Collectors.toSet());

    return prunedWalls;
  }

  Node getRandomNode() {
    return map.get(rand.nextInt(cellsTall)).get(rand.nextInt(cellsWide));
  }

  Node getNode(int i, int j) {
    i = constrain(i, 0, cellsTall - 1);
    j = constrain(j, 0, cellsWide - 1);
    return map.get(i).get(j);
  }

  Node getNodeClosestToPoint(PVector point) {
    return map.stream().flatMap(row -> row.stream()).sorted((n1, n2) -> Float.valueOf(PVector.dist(n1.center, point)).compareTo(PVector.dist(n2.center, point))).collect(Collectors.toList()).get(0);
  }

  ArrayList<PVector> aStar(Node start, Node goal) {
    HashMap<Node, Float> costs = new HashMap<>();
    costs.put(start, 0.0);
    HashMap<Node, Node> previous = new HashMap<>();
    PriorityQueue<QueueItem> queue = new PriorityQueue<QueueItem>(new QueueItemComparator());
    queue.add(new QueueItem(start, 0, 0 + PVector.dist(start.center, goal.center)));

    while (queue.size() > 0) {
      QueueItem curr_qi = queue.poll();
      Node curr_node = curr_qi.node;
      if (curr_node.center.equals(goal.center)) {
        break;
      } else {
        for (Node neighbor : curr_node.neighbors) {
          Float new_neighbor_weight = costs.get(curr_node) + 1;

          if (!costs.containsKey(neighbor) || new_neighbor_weight < costs.get(neighbor)) {
            costs.put(neighbor, new_neighbor_weight);
            previous.put(neighbor, curr_node);
            queue.add(new QueueItem(neighbor, 1, 1 + PVector.dist(neighbor.center, goal.center)));
          }
        }
      }
    }


    ArrayList<PVector> waypoints = new ArrayList<PVector>();
    waypoints.add(goal.center);
    Node temp = goal;
    while (temp != start) {
      waypoints.add(previous.get(temp).center);
      temp = previous.get(temp);
    }
    Collections.reverse(waypoints);
    return waypoints;
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

    //for (ArrayList<Node> row : map) {
    //  for (Node n : row) {
    //    circle(n.center.x, n.center.y, 10);
    //  }
    //}
  }
}

class QueueItem {
  Node node;
  float base_value;
  float heuristic_value;

  QueueItem(Node n, float bv, float hv) {
    this.node = n;
    this.base_value = bv;
    this.heuristic_value = hv;
  }

  float getSumValue() {
    return base_value + heuristic_value;
  }
}

class QueueItemComparator implements Comparator<QueueItem> {
  public int compare(QueueItem q1, QueueItem q2) {
    if (q1.getSumValue() < q2.getSumValue()) {
      return -1;
    } else if (q1.getSumValue() > q2.getSumValue()) {
      return 1;
    }

    return 0;
  }
}
