#include <iostream>
#include <vector>
#include <queue>
#include <bits/stdc++.h>

using namespace std;

// This class represents a graph.
class Graph {
public:
  // Constructor.
  Graph(int n) {
    vertices_ = n;
    edges_ = vector<vector<int>>(vertices_, vector<int>());
  }

  // Add an edge to the graph.
  void add_edge(int u, int v, int w) {
    edges_[u].push_back(v);
    edges_[v].push_back(u);
    weights_[u][v] = w;
    weights_[v][u] = w;
  }

  // Get the number of vertices.
  int num_vertices() const {
    return vertices_;
  }

  // Get the adjacency list for a vertex.
  const vector<int> &adj(int u) const {
    return edges_[u];
  }

  // Get the weight of an edge.
  int weight(int u, int v) const {
    return weights_[u][v];
  }

private:
  int vertices_;
  vector<vector<int>> edges_;
  vector<vector<int>> weights_;
};

// This function finds the shortest path from the source vertex `s` to the destination vertex `d` using the A* algorithm.
// The distance between two vertices `u` and `v` is stored in the array `distances`.
// The path from `s` to `v` is stored in the array `predecessors`.
void a_star(const Graph &g, int s, int d, vector<int> &distances, vector<int> &predecessors) {
  // Initialize the distances and predecessors.
  for (int v = 0; v < g.num_vertices(); v++) {
    distances[v] = INT_MAX;
    predecessors[v] = -1;
  }

  // Set the distance to the source vertex to 0.
  distances[s] = 0;

  // Create a priority queue to store the vertices that are still to be processed.
  priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> queue;
  queue.push({0, s});

  // While the queue is not empty, do the following:
  while (!queue.empty()) {
    // Pop the vertex with the minimum distance from the queue.
    int u = queue.top().second;
    queue.pop();

    // For each neighbor `v` of `u`, do the following:
    for (int v : g.adj(u)) {
      // If the distance to `v` through `u` is less than the current distance to `v`, do the following:
      if (distances[v] > distances[u] + g.weight(u, v)) {
        distances[v] = distances[u] + g.weight(u, v);
        predecessors[v] = u;
        queue.push({distances[v], v});
      }
    }
  }
}
