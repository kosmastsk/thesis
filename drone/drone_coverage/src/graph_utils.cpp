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

std::vector<octomath::Pose6D> hillClimbing(ros::NodeHandle nh, Graph graph, std::vector<octomath::Pose6D> points)
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
    order = simulatedAnnealing(nh, graph, order, weightmap, p, d);
  else
    order = hillClimbing(nh, graph, order, weightmap, p, d);

  // Order the Pose6D points according ot the order vector
  return reorderPoints(points, order);
}

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

double calculateCost(Graph graph, std::vector<int> order, WeightMap weightmap, std::vector<vertex_descriptor> p,
                     std::vector<double> d)
{
  ros::WallTime startTime = ros::WallTime::now();

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

  double dt = (ros::WallTime::now() - startTime).toSec();
  // ROS_INFO_STREAM("Calculating cost took " << dt << " seconds.");
  return cost;
}

double calculateDiff(Graph graph, std::vector<int> prev_order, std::vector<int> next_order, WeightMap weightmap,
                     std::vector<vertex_descriptor> p, std::vector<double> d, int first, int second)
{
  ros::WallTime startTime = ros::WallTime::now();

  // Change calculateCost to calculate the differences and not the whole value
  // Make it faster
  double cost = 0;
  vertex_descriptor start, goal;

  //  Subtract previous weights that do not exist anymore
  start = boost::vertex(prev_order.at(first - 1), graph);
  goal = boost::vertex(prev_order.at(first), graph);
  boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                           boost::predecessor_map(&p[0]).distance_map(&d[0]));
  cost -= d[prev_order.at(first)];

  start = boost::vertex(prev_order.at(first), graph);
  goal = boost::vertex(prev_order.at(first + 1), graph);
  boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                           boost::predecessor_map(&p[0]).distance_map(&d[0]));
  cost -= d[prev_order.at(first + 1)];

  start = boost::vertex(prev_order.at(second - 1), graph);
  goal = boost::vertex(prev_order.at(second), graph);
  boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                           boost::predecessor_map(&p[0]).distance_map(&d[0]));
  cost -= d[prev_order.at(second)];

  start = boost::vertex(prev_order.at(second), graph);
  goal = boost::vertex(prev_order.at(second + 1), graph);
  boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                           boost::predecessor_map(&p[0]).distance_map(&d[0]));
  cost -= d[prev_order.at(second + 1)];

  // Add new values
  start = boost::vertex(next_order.at(first - 1), graph);
  goal = boost::vertex(next_order.at(first), graph);
  boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                           boost::predecessor_map(&p[0]).distance_map(&d[0]));
  cost += d[next_order.at(first)];

  start = boost::vertex(next_order.at(first), graph);
  goal = boost::vertex(next_order.at(first + 1), graph);
  boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                           boost::predecessor_map(&p[0]).distance_map(&d[0]));
  cost += d[next_order.at(first + 1)];

  start = boost::vertex(next_order.at(second - 1), graph);
  goal = boost::vertex(next_order.at(second), graph);
  boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                           boost::predecessor_map(&p[0]).distance_map(&d[0]));
  cost += d[next_order.at(second)];

  start = boost::vertex(next_order.at(second), graph);
  goal = boost::vertex(next_order.at(second + 1), graph);
  boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                           boost::predecessor_map(&p[0]).distance_map(&d[0]));
  cost += d[next_order.at(second + 1)];

  double dt = (ros::WallTime::now() - startTime).toSec();
  // ROS_INFO_STREAM("Calculating cost took " << dt << " seconds.");
  return cost;
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

std::vector<int> getNextOrder(std::vector<int> order, int& first_index, int& second_index)
{
  first_index = int(getRandomNumber(1, order.size() - 1));
  second_index = int(getRandomNumber(1, order.size() - 1));

  std::iter_swap(order.begin() + first_index, order.begin() + second_index);

  return order;
}

std::vector<int> hillClimbing(ros::NodeHandle nh, Graph graph, std::vector<int> order,
                              boost::property_map<Graph, boost::edge_weight_t>::type weightmap,
                              std::vector<vertex_descriptor> p, std::vector<double> d)
{
  ROS_INFO("Hill climbing is running....\n");

  ros::WallTime startTime = ros::WallTime::now();
  // Load from Parameter Server
  int iterations;
  double goal;
  nh.param<int>("/hill_climbing/iterations", iterations, 1000);
  nh.param<double>("/hill_climbing/goal", goal, 80);

  double init_distance = calculateCost(graph, order, weightmap, p, d);
  double distance = init_distance;

  int iter = 0;

  while (iter < iterations && distance > goal)
  {
    ROS_INFO("iteration : %d\n", iter);
    int first = 0, second = 0;
    std::vector<int> next_order = getNextOrder(order, first, second);

    double new_distance = distance + calculateDiff(graph, order, next_order, weightmap, p, d, first, second);

    if (new_distance < distance)
    {
      order = next_order;
      distance = new_distance;
    }
    iter++;
  }

  double shortest_distance = distance;

  ROS_INFO("Initial VS shortest distance: %f, %f\n", init_distance, shortest_distance);
  ROS_INFO("%d iterations needed\n", iter);

  double dt = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO_STREAM("Hill Climbing took " << dt << " seconds.");

  return order;
}

std::vector<int> simulatedAnnealing(ros::NodeHandle nh, Graph graph, std::vector<int> order,
                                    boost::property_map<Graph, boost::edge_weight_t>::type weightmap,
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

  double init_distance = calculateCost(graph, order, weightmap, p, d);
  double distance = init_distance;

  while (temperature > absolute_temperature)
  {
    ROS_INFO("temperature: %f\n", temperature);
    int first = 0, second = 0;
    std::vector<int> next_order = getNextOrder(order, first, second);
    delta_distance = calculateDiff(graph, order, next_order, weightmap, p, d, first, second);

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

}  // namespace drone_coverage
