#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>

using namespace std;

const int INF = numeric_limits<int>::max();
// Graph class definition
class Graph {
    int V; // Number of vertices
    vector<vector<pair<int, int>>> adj; // Adjacency list (node, weight)

public:
    Graph(int V) {
        this->V = V;
        adj.resize(V);
    }

    void addEdge(int u, int v, int weight) {
        adj[u].push_back({v, weight});
        adj[v].push_back({u, weight}); // Undirected graph
    }

    // Dijkstra's Algorithm
    void dijkstra(int src) {
        vector<int> dist(V, INF);
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;

        dist[src] = 0;
        pq.push({0, src});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            for (auto &edge : adj[u]) {
                int v = edge.first;
                int weight = edge.second;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                }
            }
        }

        displayDistances(src, dist);
    }

    // Bellman-Ford Algorithm
    void bellmanFord(int src) {
        vector<int> dist(V, INF);
        dist[src] = 0;

        for (int i = 0; i < V - 1; i++) {
            for (int u = 0; u < V; u++) {
                for (auto &edge : adj[u]) {
                    int v = edge.first, weight = edge.second;
                    if (dist[u] != INF && dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                    }
                }
            }
        }

        displayDistances(src, dist);
    }

    // Display shortest distances
    void displayDistances(int src, vector<int> &dist) {
        cout << "Shortest path from node " << src << ":\n";
        for (int i = 0; i < V; i++) {
            cout << "To " << i << " -> Distance: " << (dist[i] == INF ? -1 : dist[i]) << endl;
        }
    }
};

int main() {
    int V, E;
    cout << "Enter number of vertices and edges: ";
    cin >> V >> E;

    Graph g(V);

    cout << "Enter edges (u v weight):\n";
    for (int i = 0; i < E; i++) {
        int u, v, weight;
        cin >> u >> v >> weight;
        g.addEdge(u, v, weight);
    }

    int choice, src;
    cout << "Choose algorithm:\n1. Dijkstra\n2. Bellman-Ford\nEnter choice: ";
    cin >> choice;
    cout << "Enter source node: ";
    cin >> src;

    if (choice == 1)
        g.dijkstra(src);
    else if (choice == 2)
        g.bellmanFord(src);
    else
        cout << "Invalid choice!";

    return 0;
}
