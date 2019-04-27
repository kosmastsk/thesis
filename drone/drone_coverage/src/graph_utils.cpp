#include "drone_coverage/graph_utils.h"

namespace drone_coverage
{
Graph generateGraph(ros::NodeHandle nh, std::vector<octomath::Pose6D> points)
{
  ROS_INFO("Generating graph...\n");
  // https://www.technical-recipes.com/2015/getting-started-with-the-boost-graph-library/

  // Create the edges between nodes that their distance is smaller than 1.5m and they are visible between them
  // writing out the edges in the graph
  std::vector<Edge> edges;
  std::vector<double> weights;

  double rfid_range;
  nh.param<double>("/rfid/range", rfid_range, 1);

  for (int i = 0; i < points.size(); i++)
  {
    for (int j = 0; j < points.size(); j++)
    {
      if (i == j)
        continue;  // point with itself

      // Check distance
      double distance = points.at(i).distance(points.at(j));

      if (distance < 0.75 * rfid_range)
      {
        // Save edge and weight
        edges.push_back(Edge(i, j));
        weights.push_back(distance);
      }
    }
  }

  ROS_INFO("Graph has been created\n");
  Graph g(edges.begin(), edges.end(), weights.begin(), points.size());
  return g;
}

std::vector<octomath::Pose6D> hillClimbingBase(ros::NodeHandle nh, Graph graph, std::vector<octomath::Pose6D> points)
{
  ROS_INFO("Hill climbing is running....\n");
  // Better use int vector, than Pose6D
  std::vector<int> order, new_order;
  order.resize(points.size());

  // Fill in the vector
  for (std::vector<int>::const_iterator it = order.begin(); it != order.end(); ++it)
    order.at(it - order.begin()) = it - order.begin();

  // Create a property map for the graph
  boost::property_map<Graph, boost::edge_weight_t>::type weightmap = get(boost::edge_weight, graph);
  std::vector<vertex_descriptor> p(boost::num_vertices(graph));
  std::vector<double> d(boost::num_vertices(graph));

  double init_distance = calculateCost(graph, order, p, d);
  ROS_INFO("Initial distance is %f\n", init_distance);

  double best_distance = init_distance;
  std::vector<int> best_order = order;

  int current_node, next_node;

  int iterations, restarts;
  double desired_distance;
  nh.param<int>("/hill_climbing/iterations", iterations, 25);
  nh.param<int>("/hill_climbing/restarts", restarts, 5);
  nh.param<double>("/hill_climbing/goal", desired_distance, 2);

  // Restarts
  for (int rs = 0; rs < restarts; rs++)
  {
    current_node = 0;  //    getRandomNumber(0, order.size() - 1);

    // Order must contain more than 2 points to make a comparison
    while (order.size() > 2)
    {
      ROS_DEBUG("Current node %d", current_node);
      // Find where in order are the current and the next goal
      std::vector<int>::iterator it = std::find(order.begin(), order.end(), current_node);
      int current_index = std::distance(order.begin(), it);
      int next_index;

      new_order.push_back(current_node);
      order.erase(order.begin() + current_index);

      // [....., max index that makes this condition true , size()-2 , size()-1]
      if (current_index < order.size() - 2)
      {
        next_index = current_index;
        // We have erased the current node from order, so the size becomes one less, and the index is the same
      }
      else
        next_index = getRandomNumber(0, order.size() - 1);

      next_node = order.at(next_index);

      vertex_descriptor start, goal;
      start = boost::vertex(current_node, graph);
      goal = boost::vertex(next_node, graph);
      boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                               boost::predecessor_map(&p[0]).distance_map(&d[0]));
      double distance = d[next_node];

      if (distance > desired_distance)
      {
        // Apply hill climbing to the next node to find a best solution
        // Swap next index with a random index
        int iter = 0;
        do
        {
          iter++;
          int random_node = order.at(int(getRandomNumber(0, order.size() - 1)));
          if (random_node == current_node || random_node == next_node)
            continue;

          vertex_descriptor start, goal;
          start = boost::vertex(current_node, graph);
          goal = boost::vertex(random_node, graph);
          boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                                   boost::predecessor_map(&p[0]).distance_map(&d[0]));
          double new_distance = d[random_node];

          if (new_distance < distance)
          {
            distance = new_distance;
            next_node = random_node;
          }
          if (new_distance < desired_distance)
            break;

        } while (iter < iterations && distance > desired_distance);
      }

      current_node = next_node;
    }
    // Add the last two points
    new_order.push_back(order.back());
    new_order.push_back(order.front());

    order.resize(new_order.size());
    order.swap(new_order);
    new_order.clear();

    double total_distance = calculateCost(graph, order, p, d);
    ROS_INFO("distance for restart #%d : %f\n", rs, total_distance);

    if (total_distance < best_distance)
    {
      best_distance = total_distance;
      best_order = order;
    }
  }  // rs

  // Order the Pose6D points according ot the order vector
  ROS_INFO("Keeping the order with the lowest total distance...\n");
  return reorderPoints(points, best_order);
}
/*
std::vector<octomath::Pose6D> hillClimbingBase(ros::NodeHandle nh, Graph graph, std::vector<octomath::Pose6D> points)
{
  // Better use int vector, than Pose6D
  std::vector<int> order;
  order.resize(points.size());

  for (std::vector<int>::const_iterator it = order.begin(); it != order.end(); ++it)
    order.at(it - order.begin()) = it - order.begin();

  // Generate a random solution
  // Shuffle points except the first point
  // ROS_INFO("Shuffling points....\n");
  // std::random_shuffle(++order.begin(), order.end());

  // https://www.boost.org/doc/libs/1_42_0/libs/graph/example/dijkstra-example.cpp
  // Create a property map for the graph
  boost::property_map<Graph, boost::edge_weight_t>::type weightmap = get(boost::edge_weight, graph);
  std::vector<vertex_descriptor> p(boost::num_vertices(graph));
  std::vector<double> d(boost::num_vertices(graph));

  //  Simulated Annealing algorithm
  // https://www.codeproject.com/Articles/26758/Simulated-Annealing-Solving-the-Travelling-Salesma
  bool runSA;
  nh.param<bool>("/simulated_annealing/run", runSA, 0);
  if (runSA)
    order = simulatedAnnealing(nh, graph, order, p, d);
  else
    order = hillClimbing(nh, graph, order, p, d);

  // Order the Pose6D points according ot the order vector
  return reorderPoints(points, order);
}
*/

std::vector<octomath::Pose6D> reorderPoints(std::vector<octomath::Pose6D> points, std::vector<int> order)
{
  std::vector<octomath::Pose6D> ordered_points;
  for (int i = 0; i < order.size(); i++)
  {
    ordered_points.push_back(points.at(order.at(i)));
  }

  points = ordered_points;

  return ordered_points;
}

double calculateCost(Graph graph, std::vector<int> order, std::vector<vertex_descriptor> p, std::vector<double> d)
{
  // ros::WallTime startTime = ros::WallTime::now();
  double cost = 0;

  for (int i = 0; i < order.size() - 1; i++)
  {
    // Source vertex
    vertex_descriptor start = boost::vertex(order.at(i), graph);

    vertex_descriptor goal = boost::vertex(order.at(i + 1), graph);
    boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                             boost::predecessor_map(&p[0]).distance_map(&d[0]));

    cost += d[order.at(i + 1)];
  }
  // double dt = (ros::WallTime::now() - startTime).toSec();
  // ROS_INFO_STREAM("Calculating cost took " << dt << " seconds.");
  return cost;
}

double calculateWeight(Graph graph, std::vector<int> order, int index, std::vector<vertex_descriptor> p,
                       std::vector<double> d)
{
  double cost = 0;
  vertex_descriptor start, goal;
  for (int i = -1; i < 1; i++)
  {
    start = boost::vertex(order.at(index + i), graph);
    goal = boost::vertex(order.at(index + i + 1), graph);
    boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                             boost::predecessor_map(&p[0]).distance_map(&d[0]));
    cost += d[order.at(index + i + 1)];
  }

  return cost;
}

double calculateDiff(Graph graph, std::vector<int> prev_order, std::vector<int> next_order,
                     std::vector<vertex_descriptor> p, std::vector<double> d, int first, int second)
{
  // ros::WallTime startTime = ros::WallTime::now();
  // Change calculateCost to calculate the differences and not the whole value
  double diff = 0;

  //  Subtract previous weights that do not exist anymore
  diff -= calculateWeight(graph, prev_order, first, p, d);
  diff -= calculateWeight(graph, prev_order, second, p, d);

  // Add new values
  diff += calculateWeight(graph, next_order, first, p, d);
  diff += calculateWeight(graph, next_order, second, p, d);

  // double dt = (ros::WallTime::now() - startTime).toSec();
  // ROS_INFO_STREAM("Calculating cost took " << dt << " seconds.");
  return diff;
}

double getRandomNumber(double i, double j)  // This function generates a random number between
{
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> distribution(i, j);
  return double(distribution(generator));
}

double getProbability(double difference, double temperature)
// This function finds the probability of how bad the new solution is
{
  return exp(-1 * difference / temperature);
}

std::vector<int> getNextOrder(std::vector<int> order, int first_index, int second_index)
{
  std::iter_swap(order.begin() + first_index, order.begin() + second_index);

  return order;
}

/*
std::vector<int> hillClimbing(ros::NodeHandle nh, Graph graph, std::vector<int> order, std::vector<vertex_descriptor> p,
                              std::vector<double> d)
{
  ROS_INFO("Hill climbing is running....\n");

  ros::WallTime startTime = ros::WallTime::now();
  // Load from Parameter Server
  int iterations;
  double goal;
  nh.param<int>("/hill_climbing/iterations", iterations, 1000);
  nh.param<double>("/hill_climbing/goal", goal, 80);

  double init_distance = calculateCost(graph, order, p, d);
  double distance = init_distance;

  int iter = 0;
  while (iter < iterations && distance > goal)
  {
    int first = int(getRandomNumber(1, order.size() - 1));
    int second = int(getRandomNumber(1, order.size() - 1));
    std::vector<int> next_order = getNextOrder(order, first, second);
    // adjacent_difference
    // IDEA
    double new_distance = distance + calculateDiff(graph, order, next_order, p, d, first, second);

    if (new_distance < distance)
    {
      order = next_order;
      distance = new_distance;
      iter++;
      std::cout << "swapping " << first << " with " << second << std::endl;
      ROS_INFO("iteration : %d\n", iter);
    }
  }

  double shortest_distance = distance;

  ROS_INFO("Initial VS shortest distance: %f, %f\n", init_distance, shortest_distance);
  ROS_INFO("%d iterations needed\n", iter);

  double dt = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO_STREAM("Hill Climbing took " << dt << " seconds.");

  return order;
}

std::vector<int> simulatedAnnealing(ros::NodeHandle nh, Graph graph, std::vector<int> order,
                                    std::vector<vertex_descriptor> p, std::vector<double> d)
{
  ROS_INFO("Hill climbing with Simulated Annealing is running....\n");

  ros::WallTime startTime = ros::WallTime::now();
  // Load from Parameter Server
  double temperature, cooling_rate, absolute_temperature;
  nh.param<double>("/simulated_annealing/temperature", temperature, 1000);
  nh.param<double>("/simulated_annealing/cooling_rate", cooling_rate, 0.999);
  nh.param<double>("/simulated_annealing/absolute_temperature", absolute_temperature, 0.00001);

  double delta_distance = 0;

  double init_distance = calculateCost(graph, order, p, d);
  double distance = init_distance;

  while (temperature > absolute_temperature)
  {
    ROS_INFO("temperature: %f\n", temperature);
    int first = int(getRandomNumber(1, order.size() - 1));
    int second = int(getRandomNumber(1, order.size() - 1));
    std::vector<int> next_order = getNextOrder(order, first, second);
    delta_distance = calculateDiff(graph, order, next_order, p, d, first, second);

    if ((delta_distance < 0) || (distance > 0 && getProbability(delta_distance, temperature) > getRandomNumber(0, 1)))
    {
      order = next_order;
      distance += delta_distance;
    }

    temperature *= cooling_rate;
  }
  double shortest_distance = distance;

  ROS_INFO("Initial VS shortest distance: %f, %f\n", init_distance, shortest_distance);

  double dt = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO_STREAM("Simulated Annealing took " << dt << " seconds.");

  return order;
}
*/

}  // namespace drone_coverage
