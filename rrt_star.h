//
// Created by mab on 26.10.22.
//

#ifndef SRC_RRT_STAR_H
#define SRC_RRT_STAR_H

#include "rrt.h"

const double kNeighborhoodRadius = 1.;


// =====================================================================================================================
//    RRT* TEMPLATE CLASS
// =====================================================================================================================
template <typename Config>
class RRT_Star: public RRT<Config> {
public:
    using RRT<Config>::RRT;

    std::vector<Node<Config>> run(int=1);

private:
    void getNeighborhood(Node<Config> &input_node, std::vector<Node<Config> *> &neighborhood);

    void extendFromCheapestNeighbor(std::vector<Node<Config>*> &neighborhood, Node<Config> &input_node);

    void rewire(Node<Config> &new_node, std::vector<Node<Config>*> &neighborhood);
};

// =====================================================================================================================
//    RRT* TEMPLATE CLASS - DEFINITIONS
// =====================================================================================================================
template <typename Config>
std::vector<Node<Config>> RRT_Star<Config>::run(int n_iterations) {
    // reserve memory for the tree nodes saved in nodes and add start as first node
    this->nodes.reserve(n_iterations + 1);
    this->nodes.template emplace_back(this->start_node.getConfig());

    //std::vector<Node<Config>*> neighborhood;
    std::vector<Node<Config>*> neighborhood;
    double goal_sample_rate = kGoalSampleRate;
    int goal_node_index = -1;

    for (int i=0; i<n_iterations; i++) {
        neighborhood.clear();

        // sample random configuration or goal_config node
        Node<Config> sample_node = this->sampleNode(goal_sample_rate);


        // get a reference of the nearest node in nodes in regard to the sampled config
        Node<Config> &nearest_node = this->getNearestNode(sample_node);

        // extend nearest_node towards sample node with a limiting step size -> new_node
        // calculate path and cost of extension for new_node
        // save new_node in nodes. update parent and children of new_node and nearest_node
        auto [path, cost] = this->steer(nearest_node, sample_node, kExtensionStepSize, false);
        Node<Config> new_node(path.back());

        // As long goal is not connected
        if (goal_sample_rate && this->getDistance(new_node, this->goal_node) <= kExtensionStepSize) {
            std::cout << "found goal in " << i << " iterations" << std::endl;
            new_node = this->goal_node;
            goal_node_index = i + 1;
            goal_sample_rate = 0;
        }

        // get vector of pointers to the nearest neighbor nodes
        getNeighborhood(new_node, neighborhood);
        assert(!neighborhood.empty());

        extendFromCheapestNeighbor(neighborhood, new_node);


        rewire(this->nodes.back(), neighborhood);

    }
    return this->getFinalPath(this->nodes[goal_node_index]);
}


template <typename Config>
void RRT_Star<Config>::getNeighborhood(Node<Config> &input_node, std::vector<Node<Config>*> &neighborhood) {
    double distance=0;

    for (Node<Config> &node: this->nodes) {
        distance = this->getDistance(input_node, node);
        if (distance > kNeighborhoodRadius) continue;
        neighborhood.push_back(&node);
    }
}


template <typename Config>
void RRT_Star<Config>::extendFromCheapestNeighbor(std::vector<Node<Config>*> &neighborhood, Node<Config> &input_node) {
    double min_cost = kDoubleMax;
    Node<Config> *cheapest_neighbor = nullptr;

    for (Node<Config> *neighbor: neighborhood) {
        auto [path, cost] = this->steer(*neighbor, input_node, kDoubleMax, false);
        if (cost < min_cost) {
            min_cost = cost;
            cheapest_neighbor = neighbor;
        }
    }

    auto [path, cost] = this->steer(*cheapest_neighbor, input_node);
    this->nodes.push_back(input_node);
    cheapest_neighbor->connectToChild(this->nodes.back(), path, cost);
}


template <typename Config>
void RRT_Star<Config>::rewire(Node<Config> &new_node, std::vector<Node<Config>*> &neighborhood) {
    double cost = 0, cost_diff = 0;

    for (Node<Config> *neighbor: neighborhood) {
        auto [path, cost] = this->steer(new_node, *neighbor);

        if (new_node.getCost() + cost >= neighbor->getCost()) continue;

        neighbor->reconnectToNewParent(new_node, path, cost);
    }
}

#endif //SRC_RRT_STAR_H
