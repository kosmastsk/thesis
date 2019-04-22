#include "drone_coverage/graph_utils.h"

namespace drone_coverage
{
Graph generateGraph(std::vector<octomath::Pose6D> points)
{
  ROS_INFO("Generating graph...\n");
  // https://www.technical-recipes.com/2015/getting-started-with-the-boost-graph-library/

  // Create the edges between nodes that their distance is smaller than 1.5m and they are visible between them
  // writing out the edges in the graph
  std::vector<Edge> edges;
  std::vector<double> weights;

  double rfid_range;
  ros::NodeHandle nh;
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

std::vector<octomath::Pose6D> hillClimbing(Graph graph, std::vector<octomath::Pose6D> points)
{
  // Better use int vector, than Pose6D
  std::vector<int> order;
  order.resize(points.size());
  for (std::vector<int>::const_iterator it = order.begin(); it != order.end(); ++it)
    order.at(it - order.begin()) = it - order.begin();

  ROS_INFO("Shuffling points....\n");

  // Generate a random solution
  // Shuffle points except the first point
  // std::random_shuffle(++order.begin(), order.end());

  // https://www.boost.org/doc/libs/1_42_0/libs/graph/example/dijkstra-example.cpp
  // Create a property map for the graph
  boost::property_map<Graph, boost::edge_weight_t>::type weightmap = get(boost::edge_weight, graph);
  std::vector<vertex_descriptor> p(boost::num_vertices(graph));
  std::vector<double> d(boost::num_vertices(graph));

  //  Simulated Annealing algorithm
  // https://www.codeproject.com/Articles/26758/Simulated-Annealing-Solving-the-Travelling-Salesma

  ROS_INFO("Hill climbing with Simulated Annealing is running....\n");

  double temperature = 10;  // 1000;
  double delta_distance = 0;
  double cooling_rate = 0.999;
  double absolute_temperature = 1;  // 0.00001;

  double distance = calculateCost(graph, order, weightmap, p, d);
  double init_distance = distance;
  while (temperature > absolute_temperature)
  {
    ROS_INFO("temperature: %f\n", temperature);

    std::vector<int> next_order = getNextOrder(order);
    delta_distance = calculateCost(graph, next_order, weightmap, p, d) - distance;
    if ((delta_distance < 0) || (distance > 0 && getProbability(delta_distance, temperature) > getRandomNumber(0, 1)))
    {
      order = next_order;
      distance += delta_distance;
    }

    temperature *= cooling_rate;
  }
  double shortest_distance = distance;

  ROS_INFO("Initial VS shortest distance: %f, %f\n", init_distance, shortest_distance);

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

double calculateCost(Graph graph, std::vector<int> order,
                     boost::property_map<Graph, boost::edge_weight_t>::type weightmap, std::vector<vertex_descriptor> p,
                     std::vector<double> d)
{
  double cost = 0;

  for (int i = 0; i < order.size() - 1; i++)
  {
    // Source vertex
    vertex_descriptor s = boost::vertex(order.at(i), graph);

    boost::dijkstra_shortest_paths(graph, s, boost::predecessor_map(&p[0]).distance_map(&d[0]));

    cost += d[order.at(i + 1)];
  }

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

std::vector<int> getNextOrder(std::vector<int> order)
{
  int first_random_index = getRandomNumber(1, order.size() - 1);
  int second_random_index = getRandomNumber(1, order.size() - 1);

  std::iter_swap(order.begin() + first_random_index, order.begin() + second_random_index);

  return order;
}

}  // namespace drone_coverage
