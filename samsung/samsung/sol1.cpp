#include<bits/stdc++.h>

using namespace std;

const int INF = 1e9;

struct Edge {
    int destination;
    int time;
};

vector<int> dijkstra(int start, int n, vector<vector<Edge>>& graph) {
    vector<int> dist(n + 1, INF);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    dist[start] = 0;
    pq.push({0, start});
    
    while (!pq.empty()) {
        int d = pq.top().first;
        int node = pq.top().second;
        pq.pop();
        
        if (d > dist[node]) continue;
        
        for (const Edge& edge : graph[node]) {
            int next = edge.destination;
            int time = edge.time;
            if (dist[node] + time < dist[next]) {
                dist[next] = dist[node] + time;
                pq.push({dist[next], next});
            }
        }
    }
    return dist;
}

int main() {
    int T;
    cin >> T;
    for (int t = 1; t <= T; t++) {
        int N, E, M;
        cin >> N >> E >> M;

        vector<vector<Edge>> graph(N + 1);
        
        for (int i = 0; i < E; i++) {
            int u, v, h;
            cin >> u >> v >> h;
            graph[u].push_back({v, h});
        }
        
        vector<pair<int, int>> deliveries(M);
        for (int i = 0; i < M; i++) {
            int start, dest;
            cin >> start >> dest;
            deliveries[i] = {start, dest};
        }
        
        vector<vector<int>> all_pairs_dist(N + 1, vector<int>(N + 1, INF));
        for (int i = 1; i <= N; i++) {
            all_pairs_dist[i] = dijkstra(i, N, graph);
        }
        
        int min_time = INF;
        
        for (int mask = 0; mask < (1 << M); mask++) {
            vector<pair<int, int>> robot1_deliveries, robot2_deliveries;
            
            for (int i = 0; i < M; i++) {
                if (mask & (1 << i)) {
                    robot1_deliveries.push_back(deliveries[i]);
                } else {
                    robot2_deliveries.push_back(deliveries[i]);
                }
            }
            
            int robot1_time = 0, robot2_time = 0;
            
            auto calculate_path_time = [&](vector<pair<int, int>>& deliveries) {
                int time = 0, current_pos = 1; 
                for (auto& delivery : deliveries) {
                    int start = delivery.first, dest = delivery.second;
                    time += all_pairs_dist[current_pos][start];
                    time += all_pairs_dist[start][dest];
                    current_pos = dest;
                }
                if (!deliveries.empty()) {
                    time += all_pairs_dist[current_pos][1];
                }
                return time;
            };
            
            robot1_time = calculate_path_time(robot1_deliveries);
            robot2_time = calculate_path_time(robot2_deliveries);
            
            int total_time = max(robot1_time, robot2_time);
            min_time = min(min_time, total_time);
        }
        
        cout << "Case #" << t << ": " << min_time << endl;
    }
    return 0;
}
