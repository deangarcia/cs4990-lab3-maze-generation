import java.util.Random; //<>// //<>// //<>//
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
  PVector center;                                      // The center of the cell
  boolean visited = false;                              // Has the cell been visited yet?
  ArrayList<Node> neighbors = new ArrayList<Node>();    // Which nodes does this node neighbor?
  ArrayList<Wall> walls = new ArrayList<>();            // Which walls surround this node

  Node(PVector center, ArrayList<Wall> walls) {
    this.center = center;
    this.walls = walls;
  }

  // Simple mutator, set visited to true
  public void visit() {
    this.visited = true;
  }

  // Add another node to this nodes neighbor list
  public void addNeighbor(Node neighbor) {
    neighbors.add(neighbor);
  }

  // Static method to have two nodes add each other as neighbors
  static void associate(Node a, Node b) {
    a.addNeighbor(b);
    b.addNeighbor(a);
  }

  // Get this nodes neighbors
  public ArrayList<Wall> getWalls() {
    return this.walls;
  }
}


class Map
{
  HashSet<Wall> walls;                  // The walls of the map
  ArrayList<ArrayList<Node>> map;      // 2D arraylist of the nodes
  int cellsWide, cellsTall;              // Dimensions of the maze
  HashMap<PVector, Node> coordsToNode;  // Hashmap that can be used to find a node from it's center position

  Map()
  {

    walls = new HashSet<Wall>();
    coordsToNode = new HashMap<PVector, Node>();
  }



  void generate(int which)
  {
    cellsWide = width / GRID_SIZE;  // Calculate the width of the map
    cellsTall = height / GRID_SIZE;  // Calculate the height of the map
    Random rand = new Random(which); // Create a random object

    /**
     Initialize Node Map
     **/
    map = new ArrayList<ArrayList<Node>>();    
    coordsToNode = new HashMap<PVector, Node>();
    walls = new HashSet<Wall>();

    // Initialize map with the correct dimensions
    for (int row = 0; row < cellsTall; row++) {
      map.add(new ArrayList<Node>(cellsWide));
    }

    // Populate map with nodes
    for (int row = 0; row < cellsTall; row++) {
      for (int col = 0; col < cellsWide; col++) {
        float centerOffset = (float) GRID_SIZE / 2;      // Calculate the offset from the side of a cell to the center of the cell
        int adjustedColPos = col * GRID_SIZE;  // X position of left side of grid cell
        int adjustedRowPos = row * GRID_SIZE;  // Y position of top side of grid cell
        PVector nodeCenter = new PVector(adjustedColPos + centerOffset, adjustedRowPos + centerOffset);  // The center of this node cell

        // Create walls bordering this node
        Wall up = new Wall(new PVector(adjustedColPos, adjustedRowPos), new PVector(adjustedColPos + GRID_SIZE, adjustedRowPos));
        Wall left = new Wall(new PVector(adjustedColPos, adjustedRowPos), new PVector(adjustedColPos, adjustedRowPos + GRID_SIZE));
        Wall right = new Wall(new PVector(adjustedColPos + GRID_SIZE, adjustedRowPos + GRID_SIZE), new PVector(adjustedColPos + GRID_SIZE, adjustedRowPos));
        Wall down = new Wall(new PVector(adjustedColPos + GRID_SIZE, adjustedRowPos + GRID_SIZE), new PVector(adjustedColPos, adjustedRowPos + GRID_SIZE));


        walls.addAll(new ArrayList<>(Arrays.asList(up, left, right, down))); // Add the walls, hashset will prevent duplicates

        Node newNode = new Node(nodeCenter, new ArrayList<>(Arrays.asList(up, left, right, down))); // Create node and tell it which walls it has

        map.get(row).add(newNode); // Add node to map

        coordsToNode.put(nodeCenter, newNode); // Add node to hashmap so we can find this node from it's center
      }
    }

    
    /*
     Maze Generation using Prim's Algorithm
     */

    // Choose a random node to start from
    Node startNode = map.get(rand.nextInt(cellsTall)).get(rand.nextInt(cellsWide));  // The node we will start with
    startNode.visit();                                                               // Declare our start node visited

    // Setup frontier
    Set<Wall> wallFrontier = new HashSet<Wall>();  // Our frontier of walls that tracks which ones we can remove
    wallFrontier.addAll(startNode.getWalls());     // Add the start nodes walls to the frontier

    // While there are unvisited nodes in the map
    while (map.stream().flatMap(row -> row.stream()).filter(n -> n.visited == false).collect(Collectors.toList()).size() > 0) {
      wallFrontier = pruneWallFrontier(wallFrontier); // Prune the frontier to exclude border walls and walls we can no longer remove

      // Find a valid wall
      Wall chosenWall = new ArrayList<>(wallFrontier).get(rand.nextInt(wallFrontier.size())); // Choose a wall from the frontier
      ArrayList<Node> wallNeighbors = getWallNodes(chosenWall, coordsToNode);                 // Get the node neighbors of the wall

      // Get our new node and visit it
      Node freshlyVisitedNode = wallNeighbors.get(0).visited ? wallNeighbors.get(1) : wallNeighbors.get(0); // Figure out which neighbor of the wall we haven't visited yet
      freshlyVisitedNode.visit();  // Visit the neighbor

      wallFrontier.addAll(freshlyVisitedNode.getWalls());      // Add the new node's walls to frontier. Because wallFrontier is a set, we don't have to worry about any of the walls we add being duplicates
      wallFrontier.remove(chosenWall); // Remove the wall we came from from the frontier
      walls.remove(chosenWall);  // Remove the wall from our final map

      Node.associate(wallNeighbors.get(0), wallNeighbors.get(1));  // Associate the nodes to each other now that the wall is gone
    }
  }


  /*
    A wall separates two nodes, this method can be used to find the two nodes a given wall separates
    It takes the given wall and a hashmap that maps from node centers to the corresponding node
    It returns an arraylist containing the neighbor nodes
  */
  ArrayList<Node> getWallNodes(Wall wall, HashMap<PVector, Node> coordsToNode) {
    // Figure out if the wall is vertical or horizontal
    boolean verticalWall = Float.compare(wall.normal.x, 0.0) != 0 ;
    boolean horizontalWall = Float.compare(wall.normal.y, 0.0) != 0 ;


    // Find center of the nodes we're looking for
    PVector node1Coord = null;
    PVector node2Coord = null;

    if (horizontalWall) {
      //println("wall " + wall.center() + " is h");
      node1Coord = PVector.sub(wall.center(), new PVector(0, (float) GRID_SIZE / 2));
      node2Coord = PVector.add(wall.center(), new PVector(0, (float) GRID_SIZE / 2));
    } else if (verticalWall) {
      //println("wall " + wall.center() + " is v");
      node1Coord = PVector.sub(wall.center(), new PVector((float) GRID_SIZE / 2, 0));
      node2Coord = PVector.add(wall.center(), new PVector((float) GRID_SIZE / 2, 0));
    } else {
      // Floating point number comparison is oh so fun
      println("PAIN PAIN PAIN");
      println(wall.normal);
    }

    // Find nodes based on their center. If no node is found at that position (the wall is a border wall) it will return null
    Node node1 = coordsToNode.get(node1Coord);
    Node node2 = coordsToNode.get(node2Coord);

    return new ArrayList<>(Arrays.asList(node1, node2));
  }
  
  /*
    Method to prune the wall frontier by removing walls we can't consider for removal (walls on the border or walls we can't remove without creating a cycle)
    Takes in a set of walls and returns a set of walls
  */
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

  // Method to get a random node from the map
  Node getRandomNode() {
    return map.get(rand.nextInt(cellsTall)).get(rand.nextInt(cellsWide));
  }

  // Method to get a node based on it's index in the map
  // Not currently used
  Node getNode(int i, int j) {
    i = constrain(i, 0, cellsTall - 1);
    j = constrain(j, 0, cellsWide - 1);
    return map.get(i).get(j);
  }

  // Method to return the node closest to a given point on the screen
  Node getNodeClosestToPoint(PVector point) {
    // Stream the map and sort it by distance to point, then get the closest one
    return map.stream().flatMap(row -> row.stream()).sorted((n1, n2) -> Float.valueOf(PVector.dist(n1.center, point)).compareTo(PVector.dist(n2.center, point))).collect(Collectors.toList()).get(0);
  }

  // A* path finding algorithm
  // Takes in the starting node and ending node
  // Returns a path of PVectors (the node centers) to the goal
  ArrayList<PVector> aStar(Node start, Node goal) {
    HashMap<Node, Float> costs = new HashMap<>();  // Track costs
    costs.put(start, 0.0);
    HashMap<Node, Node> previous = new HashMap<>();  // Track how we reached a node
    PriorityQueue<QueueItem> queue = new PriorityQueue<QueueItem>(new QueueItemComparator());  // The queue
    queue.add(new QueueItem(start, 0, 0 + PVector.dist(start.center, goal.center)));
    
    // While there are nodes
    while (queue.size() > 0) {
      QueueItem curr_qi = queue.poll();
      Node curr_node = curr_qi.node;
      
      
      if (curr_node.center.equals(goal.center)) {
        break; // If we've found our goal, break
      } else {
        for (Node neighbor : curr_node.neighbors) {
          Float new_neighbor_weight = costs.get(curr_node) + 1; // The actual weight of reaching the node

          if (!costs.containsKey(neighbor) || new_neighbor_weight < costs.get(neighbor)) {
            costs.put(neighbor, new_neighbor_weight);
            previous.put(neighbor, curr_node);
            queue.add(new QueueItem(neighbor, new_neighbor_weight, PVector.dist(neighbor.center, goal.center)));
          }
        }
      }
    }

    // Backtrack to find our path
    ArrayList<PVector> waypoints = new ArrayList<PVector>();
    waypoints.add(goal.center);
    Node temp = goal;
    while (temp != start) {
      waypoints.add(previous.get(temp).center);
      temp = previous.get(temp);
    }
    Collections.reverse(waypoints);
    return minimizePath(waypoints); // Prune the waypoints a bit
  }

  // Method to prune the returned astar path minimally.
  // Removes any waypoints that are in a straight line from the previous and next waypoints
  // Returns a list of waypoints where waypoints only exist at the start, corners, and end
  ArrayList<PVector> minimizePath(ArrayList<PVector> path) {
    ArrayList<PVector> prunedPath = new ArrayList<>();
    prunedPath.add(path.get(0));  // Add the first waypoint
    if (path.size() > 2) {
      for (int i = 1; i < path.size() - 1; i++) {
        PVector prev = path.get(i - 1);
        PVector curr = path.get(i);
        PVector next = path.get(i + 1);

        // If this waypoint doesn't share the same x value with the previous and next or it doesn't share the y value with previous and next, include it
        if (!(Float.valueOf(prev.x).equals(curr.x) && Float.valueOf(curr.x).equals(next.x)) && !(Float.valueOf(prev.y).equals(curr.y) && Float.valueOf(curr.y).equals(next.y))) {
          prunedPath.add(curr);
        }
      }
    }
    prunedPath.add(path.get(path.size() - 1)); // Add the last waypoint
    return prunedPath;
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
    //    text("" + n.center, n.center.x, n.center.y);
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
