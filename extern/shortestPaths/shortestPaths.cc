#include "shortestPaths.hh"
#include <iostream>
#include "../dijkstra/dijkstra.hh"

int minKey(const Eigen::ArrayXi& key, const Eigen::ArrayXi& mstSet) {
  // Initialize min value
  int min = INT_MAX, min_index;

  for (int v = 0; v < mstSet.size(); v++)
    if (mstSet[v] == false && key[v] < min)
      min = key[v], min_index = v;

  return min_index;
}

void traverseDepthFirst(const Eigen::ArrayXi& parent, long long index, std::vector<size_t>& path, bool _traverseBack) {
  path.push_back(index);
  for (long long i = parent.size() - 1; i >= 0; i--) {
    if (index == parent[i])
      traverseDepthFirst(parent, i, path, _traverseBack);
  }
  if (_traverseBack && parent[index] != -1)
    path.push_back(parent[index]);
}

// A utility function to print the constructed MST stored in parent[]
void printMST(const Eigen::ArrayXi& parent, int n, const Eigen::MatrixXi& graph) {
  std::cout << parent << std::endl;
  printf("Edge   Weight\n");
  for (int i = 1; i < graph.cols(); i++)
    printf("%d - %d    %d \n", parent[i], i, graph(i, parent[i]));
}

// Function to construct and print MST for a graph represented using adjacency
// matrix representation
std::vector<size_t> primMST(const Eigen::MatrixXi& graph, size_t startNode) {
  int V = graph.cols();
  // int graph[V][V]
  Eigen::ArrayXi parent = Eigen::ArrayXi::Zero(V); // Array to store constructed MST
  Eigen::ArrayXi key = Eigen::ArrayXi::Zero(V);   // Key values used to pick minimum weight edge in cut
  Eigen::ArrayXi mstSet = Eigen::ArrayXi::Zero(V);  // To represent set of vertices not yet included in MST

  // Initialize all keys as INFINITE
  for (int i = 0; i < V; i++)
    key[i] = INT_MAX, mstSet[i] = false;

  // Always include first 1st vertex in MST.
  key[startNode] = 0;     // Make key 0 so that this vertex is picked as first vertex
  parent[startNode] = -1; // First node is always root of MST

  // The MST will have V vertices
  for (int count = 0; count < V - 1; count++) {
    // Pick thd minimum key vertex from the set of vertices
    // not yet included in MST
    int u = minKey(key, mstSet);

    // Add the picked vertex to the MST Set
    mstSet[u] = true;

    // Update key value and parent index of the adjacent vertices of
    // the picked vertex. Consider only those vertices which are not yet
    // included in MST
    for (int v = 0; v < V; v++)

      // graph[u][v] is non zero only for adjacent vertices of m
      // mstSet[v] is false for vertices not yet included in MST
      // Update the key only if graph[u][v] is smaller than key[v]
      if (graph(u, v) && mstSet[v] == false && graph(u, v) <  key[v])
        parent[v]  = u, key[v] = graph(u, v);
  }

  // print the constructed MST
  // printMST(parent, V, graph);

  std::vector<size_t> path;
  if (1 == 1) { // tree depth first traverse
    traverseDepthFirst(parent, startNode, path, true);
  }
  else {
    traverseDepthFirst(parent, startNode, path, false);
  }

  // igl::dijkstra_compute_paths

  return path;
}