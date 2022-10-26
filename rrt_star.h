//
// Created by mab on 26.10.22.
//

#ifndef SRC_RRT_STAR_H
#define SRC_RRT_STAR_H

#include "rrt.h"

const double kNeighborhoodRadius = 0.5;


// =====================================================================================================================
//    RRT* TEMPLATE CLASS
// =====================================================================================================================
template <typename Config>
class RRT_Star: public RRT<Config> {
public:
    using RRT<Config>::RRT;
    void run(int=1);
private:
    void getNeighborhood(Config &input_config, std::vector<Node<Config>*> &neighborhood);
    void extendAndRewire(Config &input_config, std::vector<Node<Config>*> &neighborhood);


    };


// =====================================================================================================================
//    RRT* TEMPLATE CLASS - DEFINITIONS
// =====================================================================================================================
template <typename Config>
void RRT_Star<Config>::run(int n_iterations) {
    Config sampled_config, intermediate_config;
    std::vector<Node<Config>*> neighborhood;

    this->nodes.reserve(n_iterations + 1);
    this->nodes.emplace_back(this->start);

    for (int i=0; i<n_iterations; i++) {
        this->sampleConfig(sampled_config);
        Node<Config> &nearest_node = this->getNearestNode(sampled_config);
        this->getIntermediateConfig(nearest_node.config, sampled_config, intermediate_config);

        getNeighborhood(intermediate_config, neighborhood);
        extendAndRewire(intermediate_config, neighborhood);
        //extendNode(*neighborhood[0]), intermediate_config);
        //this->extendNode(nearest_node, intermediate_config);
    }
}


template <typename Config>
void RRT_Star<Config>::getNeighborhood(Config &input_config, std::vector<Node<Config>*> &neighborhood) {
    double distance, min_distance=std::numeric_limits<double>::max();

    for (Node<Config> &node: this->nodes) {
        distance = this->normalizedSquaredDistance(input_config, node.config);
        if (distance > kNeighborhoodRadius) continue;
        neighborhood.push_back(&node);
    }
}


template <typename Config>
void RRT_Star<Config>::extendAndRewire(Config &input_config, std::vector<Node<Config>*> &neighborhood) {
    std::vector<double> cost(neighborhood.size());
    double min_cost = std::numeric_limits<double>::max();
    int cheapest_node_idx = 0;

    for (int i=0; i<neighborhood.size(); i++) {
        cost[i] = this->normalizedDistance(neighborhood[i]->config, input_config);

        if (cost[i] + neighborhood[i]->cost > min_cost) continue;

        min_cost = cost[i] + neighborhood[i]->cost;
        cheapest_node_idx = i;
    }

    this->nodes.emplace_back(input_config);
    this->nodes.back().cost = min_cost;
    this->nodes.back().parent = neighborhood[cheapest_node_idx];
    neighborhood[cheapest_node_idx]->children.push_back(&(this->nodes.back()));

    for (int i=0; i<neighborhood.size(); i++) {
        if (this->nodes.back().cost + cost[i] > neighborhood[i]->cost) continue;
        neighborhood[i]->cost = this->nodes.back().cost + cost[i];
        //neighborhood[i]->parent->children remove this child
        neighborhood[i]->parent = &(this->nodes.back());
        this->nodes.back().children.push_back(neighborhood[i]);
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
