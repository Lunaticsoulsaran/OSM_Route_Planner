#include "route_planner.h"
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

using std::cout;
using std::vector;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
	start_node= &m_Model.FindClosestNode(start_x,start_y);
    end_node= &m_Model.FindClosestNode(end_x,end_y);
    //cout<<"siuces";
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

   // cout<< end_node->distance(*start_node) << "\n";
    //cout<<"success9";
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    //cout<<"success8";
    current_node->FindNeighbors();
    for(RouteModel::Node *Current_Neighbour : current_node->neighbors)
    {
        Current_Neighbour->parent=current_node;
        Current_Neighbour->h_value=CalculateHValue(Current_Neighbour);
        Current_Neighbour->g_value=current_node->g_value+current_node->distance(*Current_Neighbour);
        open_list.push_back(Current_Neighbour);
        Current_Neighbour->visited=true;

    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

    //cout<<"success7";
    sort(open_list.begin(),open_list.end(),[](const auto &First,const auto &Second)
    {return First->h_value+First->g_value<Second->h_value+Second->g_value;});

    RouteModel::Node *Lowest_sum = open_list.front();
    open_list.erase(open_list.begin());
    return Lowest_sum;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    //cout<<"sucess10";
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    while(current_node->parent)
    {
        distance += current_node->parent->distance(*current_node);
        path_found.push_back(*current_node);
        current_node=current_node->parent;
    }
    path_found.push_back(*current_node);

    // TODO: Implement your solution here.
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    reverse(path_found.begin(),path_found.end());
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    //cout<<"success6";
    RouteModel::Node *current_node = nullptr;

    open_list.push_back(start_node);
    start_node->visited=true;
    while(open_list.size()>0)
    {
        current_node=NextNode();
        if(current_node==end_node)
        {
            m_Model.path=ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
    return;

    // TODO: Implement your solution here.
}