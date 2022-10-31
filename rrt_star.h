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

    void extendAndRewire(Config &input_config, std::vector<Node<Config> *> &neighborhood);

    void extendFromCheapestNeighbor(std::vector<Node<Config>*> &neighborhood, Node<Config> &input_node);

    void propagateCostToChildren(Node<Config> &node, double new_cost);

    void rewire(Node<Config> &new_node, std::vector<Node<Config>*> &neighborhood);
};

// =====================================================================================================================
//    RRT* TEMPLATE CLASS - DEFINITIONS
// =====================================================================================================================
template <typename Config>
std::vector<Node<Config>> RRT_Star<Config>::run(int n_iterations) {
    // reserve memory for the tree nodes saved in nodes and add start as first node
    this->nodes.reserve(n_iterations + 1);
    this->nodes.push_back(this->start_node);
    this->nodes.push_back(this->goal_node);
    std::vector<Node<Config>*> neighborhood;

    for (int i=0; i<n_iterations; i++) {
        neighborhood = {};

        //std::cout << "Iteration: " << i << std::endl;
        // sample random configuration or goal_config node
        Node<Config> sample_node = this->sampleNode();

        // get a reference of the nearest node in nodes in regard to the sampled config
        Node<Config> &nearest_node = this->getNearestNode(sample_node);

        // extend nearest_node towards sample node with a limiting step size -> new_node
        // calculate path and cost of extension for new_node
        // save new_node in nodes. update parent and children of new_node and nearest_node
        auto [path, cost] = this->steer(nearest_node, sample_node, kExtensionStepSize, false);
        Node<Config> new_node(path.back());

        if (this->getDistance(new_node, this->goal_node) <= kExtensionStepSize) continue;

        // get vector of pointers to the nearest neighbor nodes
        std::vector<Node<Config>*> neighborhood;
        getNeighborhood(new_node, neighborhood);
        assert(!neighborhood.empty());

        extendFromCheapestNeighbor(neighborhood, new_node);


        // todo: rewire
        //      calculate cost of connection from new node to neighbor
        //      if new node cost + cost of connection < neighbor cost
        //          neighparent = neighbor parent
        //          set neighbor parent = new_node
        //          update neighbor cost
        //          add cost_difference to children (and propagate diff to their children)
        //          if neighbor + cost of connection to old parent < parent cost: do it again
        rewire(this->nodes.back(), neighborhood);
    }
    return this->getFinalPath(this->nodes[1]);
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
    input_node.parent = cheapest_neighbor;
    input_node.cost = cheapest_neighbor->cost + min_cost;
    input_node.path = path;
    cheapest_neighbor->children.push_back(&input_node);
}


template <typename Config>
void RRT_Star<Config>::rewire(Node<Config> &new_node, std::vector<Node<Config>*> &neighborhood) {
    double cost = 0, cost_diff = 0;

    for (Node<Config> *neighbor: neighborhood) {
        auto [path, cost] = this->steer(new_node, *neighbor);

        if (new_node.cost + cost < neighbor->cost) {
            new_node.children.push_back(neighbor);
            neighbor->parent = &new_node;
            neighbor->path = path;
            cost_diff = neighbor->cost - (new_node.cost + cost);
            propagateCostToChildren(*neighbor, cost_diff);
        }
    }
}


template <typename Config>
void RRT_Star<Config>::propagateCostToChildren(Node<Config> &node, double cost_diff) {
    for (Node<Config> *child: node.children) {
        child->cost += cost_diff;
        propagateCostToChildren(*child, cost_diff);
    }
}
//template <typename Config>
//void RRT_Star::extendNode(Node<Config> &exte)
// extendNode
//    findNearestNeighbours -> return neighborhood somehow for later use
//    for neighbor:
//        calculateSteeringCostNeighborToInput
//        trackMinCost
//    ConnectToCheapestNeighbor
//
//template <typename Config>
//void RRT_Star::rewire(Node &input_node, std::vector<Node<Config>>* neighborhood) {
//    for neigbor in neighbourhodd:
//        calculateCostOfNeighborToInput
//        rewireIfCheaper
//}

#endif //SRC_RRT_STAR_H
