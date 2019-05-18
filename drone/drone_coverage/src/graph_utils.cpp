#include "drone_coverage/graph_utils.h"

namespace drone_coverage
{
double distanceXY(const Point_xy i, const Point_xy j)
{
  double dist_x = i.first - j.first;
  double dist_y = i.second - j.second;
  return sqrt(dist_x * dist_x + dist_y * dist_y);
}

Graph generateGraph(ros::NodeHandle nh, std::vector<Point_xy> points)
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
      double distance = distanceXY(points.at(i), points.at(j));

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

std::vector<Point_xy> calculateOptimalPath(ros::NodeHandle nh, Graph graph, std::vector<Point_xy> points,
                                           octomap::OcTree* octomap)
{
  ROS_INFO("Calculating the optimal path....\n");
  // Better use int vector, than Pose6D
  std::vector<int> order, new_order, best_order;
  order.resize(points.size());

  // Fill in the vector
  for (std::vector<int>::const_iterator it = order.begin(); it != order.end(); ++it)
    order.at(it - order.begin()) = it - order.begin();

  // Create a property map for the graph
  boost::property_map<Graph, boost::edge_weight_t>::type weightmap = get(boost::edge_weight, graph);
  std::vector<vertex_descriptor> p(boost::num_vertices(graph));
  std::vector<double> d(boost::num_vertices(graph));

  double total_distance = calculateCost(graph, order, p, d);
  ROS_INFO("Initial distance is %f\n", total_distance);

  best_order = order;
  double best_distance = total_distance;

  int current_node;

  int iterations, restarts;
  double desired_distance;
  nh.param<int>("/hill_climbing/iterations", iterations, 25);
  nh.param<int>("/hill_climbing/restarts", restarts, 5);
  nh.param<double>("/hill_climbing/goal", desired_distance, 2);

  // Restarts
  for (int rs = 0; rs < restarts; rs++)
  {
    current_node = 0;

    // vector order must contain more than 2 points to make a comparison
    while (order.size() > 2)
    {
      ROS_DEBUG("Current node %d", current_node);
      // Find where in order are the current and the next goal
      std::vector<int>::iterator it = std::find(order.begin(), order.end(), current_node);
      int current_index = std::distance(order.begin(), it);

      new_order.push_back(current_node);
      order.erase(order.begin() + current_index);

      // [....., max index that makes this condition true , size()-2 , size()-1]
      vertex_descriptor start;

      // Check the next node
      if (std::find(order.begin(), order.end(), current_node + 1) != order.end())
      {
        start = boost::vertex(current_node, graph);
        boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                                 boost::predecessor_map(&p[0]).distance_map(&d[0]));
        double near_distance = d[current_node + 1];

        octomath::Vector3 view_point(points.at(current_node).first, points.at(current_node).second, 1);
        octomath::Vector3 point_to_test(points.at(current_node + 1).first, points.at(current_node + 1).second, 1);

        if (near_distance < desired_distance && checkIfVisible(view_point, point_to_test, octomap))
        {
          current_node = current_node + 1;
          continue;
        }
      }

      // Check the previous node
      if (std::find(order.begin(), order.end(), current_node - 1) != order.end())
      {
        start = boost::vertex(current_node, graph);
        boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                                 boost::predecessor_map(&p[0]).distance_map(&d[0]));
        double near_distance = d[current_node - 1];

        octomath::Vector3 view_point(points.at(current_node).first, points.at(current_node).second, 1);
        octomath::Vector3 point_to_test(points.at(current_node - 1).first, points.at(current_node - 1).second, 1);

        if (near_distance < desired_distance && checkIfVisible(view_point, point_to_test, octomap))
        {
          current_node = current_node - 1;
          continue;
        }
      }

      // If none of the previous if statements continued......
      // Apply hill climbing to the next node to find a best solution
      // Swap next index with a random index
      int iter = 0;
      double distance = DBL_MAX;
      // provide a value, in case none of the points satisfies the conditions needed do
      int next_node = order.at(int(getRandomNumber(0, order.size() - 1)));
      do
      {
        int random_node = order.at(int(getRandomNumber(0, order.size() - 1)));
        if (random_node == current_node)
          continue;

        start = boost::vertex(current_node, graph);
        boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                                 boost::predecessor_map(&p[0]).distance_map(&d[0]));
        double new_distance = d[random_node];

        octomath::Vector3 view_point(points.at(current_node).first, points.at(current_node).second, 1);
        octomath::Vector3 point_to_test(points.at(random_node).first, points.at(random_node).second, 1);

        if (new_distance < distance && checkIfVisible(view_point, point_to_test, octomap))
        {
          distance = new_distance;
          next_node = random_node;
        }
        iter++;
      } while (iter < iterations && distance > desired_distance);

      current_node = next_node;
    }

    // Add the last two points
    new_order.push_back(order.front());
    new_order.push_back(order.back());

    total_distance = calculateCost(graph, new_order, p, d);
    ROS_INFO("distance for restart %d/%d : %f \n", rs + 1, restarts, total_distance);

    if (total_distance < best_distance)
    {
      best_distance = total_distance;
      best_order = new_order;
    }

    // Keeping always the order with the lowest cost, in the beginning of each restart
    order = best_order;
    new_order.clear();

  }  // rs

  // Order the Pose6D points according ot the order vector
  ROS_INFO("Keeping the order with the lowest total distance: %f...\n", best_distance);
  return reorderPoints(points, best_order);
}

std::vector<Point_xy> reorderPoints(std::vector<Point_xy> points, std::vector<int> order)
{
  std::vector<Point_xy> ordered_points;
  for (int i = 0; i < order.size(); i++)
  {
    ordered_points.push_back(points.at(order.at(i)));
  }

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

    boost::astar_search_tree(graph, start, boost::astar_heuristic<Graph, double>(),
                             boost::predecessor_map(&p[0]).distance_map(&d[0]));

    cost += d[order.at(i + 1)];
  }
  // double dt = (ros::WallTime::now() - startTime).toSec();
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

std::vector<int> getNextOrder(std::vector<int> order, int first_index, int second_index)
{
  std::iter_swap(order.begin() + first_index, order.begin() + second_index);
  return order;
}

bool checkIfVisible(const octomap::point3d view_point, const octomap::point3d point_to_test, octomap::OcTree* octomap)
{
  // Get all nodes in a line
  octomap::KeyRay key_ray;

  octomap->computeRayKeys(view_point, point_to_test, key_ray);

  const octomap::OcTreeKey& point_to_test_key = octomap->coordToKey(point_to_test);

  // Now check if there are any unknown or occupied nodes in the ray,
  // except for the point_to_test key.
  for (octomap::OcTreeKey key : key_ray)
  {
    if (key != point_to_test_key)
    {
      octomap::OcTreeNode* node = octomap->search(key);

      if (node != NULL && octomap->isNodeOccupied(node))
        return false;
    }
  }
  return true;
}

}  // namespace drone_coverage
