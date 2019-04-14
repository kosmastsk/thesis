#include "drone_coverage/graph.h"

namespace drone_coverage
{
Graph::Graph(std::vector<Edge> const& edges, int n)
{
  // resize the vector to N elements of type vector<Pair>
  _adj_list.resize(n);

  // add edges to the directed graph
  for (auto& edge : edges)
  {
    octomath::Pose6D src = edge.src;
    octomath::Pose6D dest = edge.dest;
    float weight = edge.weight;

    // Check if src exists in the vector
    std::vector<octomath::Pose6D>::iterator it = std::find(_nodes.begin(), _nodes.end(), src);

    if (it != _nodes.end())
    {
      // Node found, calculate index
      int index = std::distance(_nodes.begin(), it);
      // insert at the end
      _adj_list[index].push_back(std::make_pair(dest, weight));
    }

    // Uncomment below line for undirected graph
    // adjList[dest].push_back(make_pair(src, weight));
  }
}

Graph::~Graph()
{
  ROS_INFO("Graph object has been destroyed\n");
}

// print adjacency list representation of graph
void Graph::printGraph(Graph const& graph, int n)
{
  ROS_INFO("Print all neighboring vertices of each vertex\n");

  for (int i = 0; i < n; i++)
  {
    // print all neighboring vertices of given vertex
    for (Pair v : graph._adj_list[i])
      ROS_INFO("[ src(%f, %f, %f), dest(%f, %f, %f), weight:%f ]\n", _nodes[i].x(), _nodes[i].y(), _nodes[i].z(),
               v.first.x(), v.first.y(), v.first.z(), v.second);
  }
}

}  // namespace drone_coverage
