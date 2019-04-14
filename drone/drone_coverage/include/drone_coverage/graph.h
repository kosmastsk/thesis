#ifndef GRAPH_HEADER
#define GRAPH_HEADER

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <octomap/octomap.h>

namespace drone_coverage
{
// data structure to store graph edges
struct Edge
{
  octomath::Pose6D src, dest;
  float weight;
};

// Destination and weight to go there
typedef std::pair<octomath::Pose6D, float> Pair;

class Graph
{
public:
  // construct a vector of vectors of Pairs to represent an adjacency list
  std::vector<std::vector<Pair>> _adj_list;
  std::vector<octomath::Pose6D> _nodes;

  // Graph Constructor
  Graph(std::vector<Edge> const& edges, int n);
  ~Graph();

  void printGraph(Graph const& graph, int n);
};

}  // namespace drone_coverage

#endif
