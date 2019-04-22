#ifndef GRAPH_UTILS_HEADER
#define GRAPH_UTILS_HEADER

#include <cmath>
#include <algorithm>  // std::random_shuffle
#include <utility>    // for std::pair
#include <vector>
#include <chrono>  // for random numbers
#include <ctime>   // for random numbers

#include <ros/ros.h>

#include <octomap_msgs/conversions.h>  // for octomath

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>

namespace drone_coverage
{
struct found_goal
{
};  // exception for termination

// Boost Graph typedefs
typedef std::pair<int, int> Edge;
typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty>
    Graph;
typedef boost::property_map<Graph, boost::edge_weight_t>::type WeightMap;
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;

// Functions
Graph generateGraph(ros::NodeHandle nh, std::vector<octomath::Pose6D> points);
std::vector<octomath::Pose6D> hillClimbing(ros::NodeHandle nh, Graph graph, std::vector<octomath::Pose6D> points);
std::vector<octomath::Pose6D> reorderPoints(std::vector<octomath::Pose6D> points, std::vector<int> order);
double calculateCost(Graph graph, std::vector<int> order,
                     boost::property_map<Graph, boost::edge_weight_t>::type weightmap, std::vector<vertex_descriptor> p,
                     std::vector<double> d);

double getRandomNumber(double i, double j);
double getProbability(double difference, double temperature);
std::vector<int> getNextOrder(std::vector<int> order, int& first_index, int& second_index);

std::vector<int> hillClimbing(ros::NodeHandle nh, Graph graph, std::vector<int> order,
                              boost::property_map<Graph, boost::edge_weight_t>::type weightmap,
                              std::vector<vertex_descriptor> p, std::vector<double> d);
std::vector<int> simulatedAnnealing(ros::NodeHandle nh, Graph graph, std::vector<int> order,
                                    boost::property_map<Graph, boost::edge_weight_t>::type weightmap,
                                    std::vector<vertex_descriptor> p, std::vector<double> d);

}  // namespace drone_coverage

#endif
