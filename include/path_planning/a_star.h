#pragma once

#include "priority_queue.h"

#include <boost/optional.hpp>

#include <algorithm>
#include <map>
#include <vector>

namespace path_planning
{

// Represent a node with associated A* costs.
template <typename Node, typename Cost>
struct CostedNode
{
    // A node in the graph.
    Node node;

    // The cost to go to this node.
    Cost cost_to_go;

    // The heuristic cost for this node.
    Cost heuristic_cost;
};

// Equality operator, compares the nodes themselves.
template <typename Node, typename Cost>
bool operator==(const CostedNode<Node, Cost> &lhs, const CostedNode<Node, Cost> &rhs)
{
    return lhs.node == rhs.node;
}

// Less than operator, compares the total costs of the nodes.
template <typename Node, typename Cost>
bool operator<(const CostedNode<Node, Cost> &lhs, const CostedNode<Node, Cost> &rhs)
{
    return (lhs.cost_to_go + lhs.heuristic_cost) < (rhs.cost_to_go + rhs.heuristic_cost);
}

// Computes the optimal path from `start` to `goal`.
//
// A number of functors must be provided:
// Neighbor: provides the neighbors of a given node.
// Travel: computes the cost to travel between two neighbors.
// Heuristic: computes the heuristic cost to travel from a node to the goal.
//
// If no path can be found, a disengaged optional is returned.
template <typename Node, typename NeighborFunctor, typename TravelFunctor, typename HeuristicFunctor>
boost::optional<std::vector<Node>> a_star(const Node &start, const Node &goal, NeighborFunctor neighbor_functor,
                                          TravelFunctor travel_functor, HeuristicFunctor heuristic_functor)
{
    // The cost type that we're using.
    using Cost =
        typename std::remove_reference<decltype(travel_functor(std::declval<Node>(), std::declval<Node>()))>::type;

    // The node in the closed set.
    std::set<Node> closed_set;

    // A priority queue of the nodes in the open set.
    PriorityQueue<CostedNode<Node, Cost>> open_set;

    // A map between each node and the node that said node can be reached from the with lowest cost.
    std::map<Node, Node> came_from;

    // Initialize the open set with the start node.
    open_set.upsert(CostedNode<Node, Cost>{start, Cost{}, heuristic_functor(start)});

    while (!open_set.empty())
    {
        auto node = open_set.pop();

        if (node.node == goal)
        {
            // We've found a path to the goal.
            std::vector<Node> path;

            // Walk backwards from the goal to the start.
            auto current_node = node.node;
            while (current_node != start)
            {
                path.emplace_back(current_node);
                current_node = came_from.at(current_node);
            }
            path.emplace_back(std::move(current_node));

            // Reverse to get the path from the start to the goal.
            std::reverse(std::begin(path), std::end(path));

            return boost::make_optional(std::move(path));
        }

        // Iterate over the neighbors of the current node.
        auto neighbors = neighbor_functor(node.node);
        for (const auto &neighbor : neighbors)
        {
            if (closed_set.find(neighbor) != std::end(closed_set))
            {
                // This neighbor is already in the closed set.
                continue;
            }

            // Compute the cost to go for this neighbor via the current node.
            auto cost_to_go = node.cost_to_go + travel_functor(node.node, neighbor);

            // Compute the heuristic cost from this neighbor to the goal.
            auto heuristic_cost = heuristic_functor(neighbor);

            // Upsert the costed neighbor into the open set.
            CostedNode<Node, Cost> costed_neighbor{neighbor, std::move(cost_to_go), std::move(heuristic_cost)};
            const bool did_insert = open_set.upsert(std::move(costed_neighbor));
            if (did_insert)
            {
                // We inserted the neighbor into the open set, meaning that this is either the first time
                // we've
                // visited it, or the path via the current node is lower cost.
                came_from[neighbor] = node.node;
            }
        }

        // This node is now closed.
        closed_set.emplace(std::move(node.node));
    }

    return boost::none;
}

} // namespace path_planning
