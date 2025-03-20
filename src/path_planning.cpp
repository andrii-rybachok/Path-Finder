#include "rclcpp/rclcpp.hpp"
#include "max_ros/srv/path_request.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <limits>

#include <memory>

using namespace std;

struct node
{
    int id;
    int x;
    int y;
    char loc1[4];
    char loc2[4];
    char loc3[4];
};

struct connection
{
    int idA;
    int idB;
};

vector<node> nodes;
vector<connection> connections;
vector<vector<float>> adjacency;

struct instruction
{
    float angle;
    float distance;
};

bool setDataFromArrays(vector<node> *dataNodes, vector<connection> *dataConnections)
{
    // Define nodes
    vector<node> tempNodes = {
        {0, 0, 0, "EV2", "EV3", "PAS"},
        {1, 85, 0, "HH", "NUL", "NUL"},
        {2, 85, 25, "EV1", "AL", "NUL"},
        {3, 85, 105, "ML", "NUL", "NUL"},
        {4, 85, 230, "LIB", "NH", "NUL"},
        {5, 160, 25, "TC", "NUL", "NUL"},
        {6, 185, 180, "NUL", "NUL", "NUL"},
        {7, 205, 105, "NUL", "NUL", "NUL"},
        {8, 220, 25, "SCH", "NUL", "NUL"},
        {9, 280, 105, "NUL", "NUL", "NUL"},
        {10, 280, 160, "RCH", "NUL", "NUL"},
        {11, 355, 160, "NUL", "NUL", "NUL"},
        {12, 385, 160, "DWE", "NUL", "NUL"},
        {13, 450, 160, "CPH", "NUL", "NUL"},
        {14, 360, 225, "NUL", "NUL", "NUL"},
        {15, 290, 225, "E2", "NUL", "NUL"},
        {16, 175, 250, "NUL", "NUL", "NUL"},
        {17, 120, 250, "NUL", "NUL", "NUL"},
        {18, 45, 330, "STC", "NUL", "NUL"},
        {19, 300, 290, "PYH", "NUL", "NUL"},
        {20, 300, 370, "E3", "NUL", "NUL"},
        {21, 300, 450, "EIT", "NUL", "NUL"},
        {22, 270, 450, "NUL", "NUL", "NUL"},
        {23, 270, 540, "DC", "NUL", "NUL"},
        {24, 165, 375, "B1", "B2", "ESC"},
        {25, 165, 420, "NUL", "NUL", "NUL"},
        {26, 165, 495, "C2", "NUL", "NUL"},
        {27, 165, 540, "NUL", "NUL", "NUL"},
        {28, 100, 495, "MC", "NUL", "NUL"},
        {29, 40, 495, "NUL", "NUL", "NUL"},
        {30, 40, 550, "SLC", "NUL", "NUL"},
        {31, 40, 650, "NUL", "NUL", "NUL"},
        {32, 165, 650, "NUL", "NUL", "NUL"},
        {33, 220, 650, "M3", "NUL", "NUL"},
        {34, 385, 650, "GSC", "NUL", "NUL"},
        {35, 220, 760, "CSB", "NUL", "NUL"},
        {36, 40, 740, "BMH", "EXP", "NUL"},
        {37, 170, 800, "ERC", "NUL", "NUL"},
        {38, 220, 800, "NUL", "NUL", "NUL"},
        {39, 370, 800, "COM", "NUL", "NUL"}};

    // Define connections
    vector<connection> tempConnections = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {2, 5}, {5, 6}, {6, 7}, {7, 8}, {7, 9}, {9, 10}, {10, 11}, {11, 12}, {12, 13}, {11, 14}, {14, 15}, {15, 16}, {16, 17}, {17, 18}, {17, 4}, {15, 19}, {19, 20}, {20, 21}, {21, 22}, {22, 23}, {10, 16}, {6, 16}, {16, 24}, {24, 25}, {25, 26}, {26, 27}, {27, 23}, {28, 25}, {28, 26}, {28, 29}, {29, 30}, {30, 31}, {31, 32}, {32, 27}, {32, 33}, {33, 34}, {33, 35}, {31, 36}, {36, 37}, {37, 38}, {38, 35}, {38, 39}};

    // Populate the vectors
    *dataNodes = tempNodes;
    *dataConnections = tempConnections;

    return true;
}

// Reads data file, storing nodes and connections in vectors
bool loadGraphFromData(string sFilename, vector<node> *nodes, vector<connection> *connections)
{
    ifstream f(sFilename);
    if (!f.is_open())
    {
        return false;
    }

    while (!f.eof())
    {
        // Assumes line length is less than 64
        char line[64];
        f.getline(line, 64);

        stringstream s;
        s << line;

        char junk;

        if (line[0] == 'n')
        {
            node n;
            s >> junk >> n.id >> n.x >> n.y >> n.loc1 >> n.loc2 >> n.loc3;
            n.loc1[3] = '\0';
            n.loc2[3] = '\0';
            n.loc3[3] = '\0';
            (*nodes).push_back(n);
        }

        if (line[0] == 'c')
        {
            connection c;
            s >> junk >> c.idA >> c.idB;
            (*connections).push_back(c);
        }
    }
    return true;
}

// Returns id of node which has building name
// Note name must be 4 bytes (including theyre ending "\0")
// If searching for a two character name, ensure character 3 is "\0"
int findNode(char name[4], vector<node> nodes)
{
    for (int i = 0; i < static_cast<int>(nodes.size()); i++)
    {
        if (equal(name, name + 4, nodes[i].loc1) || equal(name, name + 4, nodes[i].loc2) || equal(name, name + 4, nodes[i].loc3))
            return nodes[i].id;
    }
    return -1;
}

// Returns the distance between two nodes
float findDistance(node n1, node n2)
{
    int dx = abs(n1.x - n2.x);
    int dy = abs(n1.y - n2.y);
    return sqrtf(dx * dx + dy * dy);
}

// Returns the angle (deg) of the path between two nodes relative to the y axis
float findAngle(node start, node end)
{
    float dy = end.y - start.y;
    float angle = (180 / 3.14159265358979323846f) * acosf(dy / findDistance(start, end));
    if (start.x > end.x)
        return angle + 180;
    return angle;
}

// Generats an nxn adjacency matrix where n is the number of nodes
// The entry in any cell coordinates x,y is the distance between node x and node y
// A value of zero indicates no connection is present
vector<vector<float>> adjacencyMatrix(vector<node> nodes, vector<connection> connections)
{
    vector<vector<float>> result(nodes.size(), vector<float>(nodes.size(), 0));
    for (int i = 0; i < static_cast<int>(connections.size()); i++)
    {
        float dist = findDistance(nodes[connections[i].idA], nodes[connections[i].idB]);
        result[connections[i].idA][connections[i].idB] = dist;
        result[connections[i].idB][connections[i].idA] = dist;
    }
    return result;
}

vector<instruction> generateInstructions(vector<int> path, vector<node> nodes)
{
    vector<instruction> result;
    for (int i = 0; i < static_cast<int>(path.size()) - 1; i++)
    {
        instruction s;
        s.angle = findAngle(nodes[path[i]], nodes[path[i + 1]]);
        s.distance = findDistance(nodes[path[i]], nodes[path[i + 1]]);
        result.push_back(s);
    }
    return result;
}

int findClosestNode(vector<node> nodes, int x, int y)
{
    int closest = -1;
    float closestDist = numeric_limits<float>::infinity();
    for (int i = 0; i < static_cast<int>(nodes.size()); i++)
    {
        float dist = findDistance(nodes[i], {0, x, y, "", "", ""});
        if (dist < closestDist)
        {
            closest = i;
            closestDist = dist;
        }
    }
    return closest;
}

vector<int> dijkstra(vector<vector<float>> adjacency, int start, int end)
{
    vector<int> path;
    vector<float> dist(adjacency.size(), numeric_limits<float>::infinity());
    vector<int> prev(adjacency.size(), -1);
    vector<bool> visited(adjacency.size(), false);
    priority_queue<pair<float, int>, vector<pair<float, int>>, greater<pair<float, int>>> pq;

    dist[start] = 0;

    pq.push(make_pair(0, start));

    while (!pq.empty())
    {
        int u = pq.top().second;
        pq.pop();
        if (u == end)
            break;
        if (visited[u])
            continue;
        visited[u] = true;

        for (int v = 0; v < static_cast<int>(adjacency.size()); v++)
        {
            if (adjacency[u][v] != 0)
            {
                float alt = dist[u] + adjacency[u][v];
                if (alt < dist[v])
                {
                    dist[v] = alt;
                    prev[v] = u;
                    pq.push(make_pair(alt, v));
                }
            }
        }
    }

    for (int at = end; at != -1; at = prev[at])
    {
        path.push_back(at);
    }

    reverse(path.begin(), path.end());

    return path;
}

string pathToString(vector<int> path, vector<node> nodes)
{
    string result = "";
    for (int i = 0; i < static_cast<int>(path.size()); i++)
    {
        result += nodes[path[i]].loc1;
        if (i != static_cast<int>(path.size()) - 1)
            result += " -> ";
    }
    return result;
}

string nodesToString(vector<node> nodes)
{
    string result = "";
    for (int i = 0; i < static_cast<int>(nodes.size()); i++)
    {
        result += nodes[i].loc1;
        if (i != static_cast<int>(nodes.size()) - 1)
            result += ", ";
    }
    return result;
}

void returnPath(const std::shared_ptr<max_ros::srv::PathRequest::Request> request,
                std::shared_ptr<max_ros::srv::PathRequest::Response> response)
{
    int start = findClosestNode(nodes, request->currentx, request->currenty);

    char goal_array[4];
    for (int i = 0; i < 3; i++)
    {
        goal_array[i] = request->goal[i];
    }

    goal_array[3] = '\0';

    int end = findNode(goal_array, nodes);

    adjacency = adjacencyMatrix(nodes, connections);

    vector<int> path = dijkstra(adjacency, start, end);

    vector<int> path_array_x(path.size());
    vector<int> path_array_y(path.size());

    for (int i = 0; i < static_cast<int>(path.size()); i++)
    {
        path_array_x[i] = nodes[path[i]].x;
        path_array_y[i] = nodes[path[i]].y;
    }

    response->pathx = path_array_x;
    response->pathy = path_array_y;

    string goal_string = "";
    for (int i = 0; i < 3; i++)
    {
        goal_string += goal_array[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: currentx: %d"
                                              " currenty: %d goal: %s",
                request->currentx, request->currenty, goal_string.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back path: %s", pathToString(path, nodes).c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    setDataFromArrays(&nodes, &connections);

    // loadGraphFromData("data.txt", &nodes, &connections);
    adjacency = adjacencyMatrix(nodes, connections);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("path_planning");

    rclcpp::Service<max_ros::srv::PathRequest>::SharedPtr service =
        node->create_service<max_ros::srv::PathRequest>("path_planning", &returnPath);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to path plan.");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Nodes: %s", nodesToString(nodes).c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connections: %d", static_cast<int>(connections.size()));

    rclcpp::spin(node);
    rclcpp::shutdown();
}

