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

    void run(int= 1);

private:
    void getNeighborhood(Config &input_config, std::vector<Node<Config> *> &neighborhood);

    void extendAndRewire(Config &input_config, std::vector<Node<Config> *> &neighborhood);

    void extendFromCheapestNeighbor(std::vector<Node<Config>*> &neighborhood, Config &input_config);

    void propagateCostToChildren(Node<Config> &node, double new_cost);

    void rewire(std::vector<Node<Config>*> &neighborhood);
};

// =====================================================================================================================
//    RRT* TEMPLATE CLASS - DEFINITIONS
// =====================================================================================================================
template <typename Config>
void RRT_Star<Config>::run(int n_iterations) {
    Config sampled_config, new_config;
    std::vector<Node<Config>*> neighborhood;

    this->nodes.reserve(n_iterations + 1);
    this->nodes.emplace_back(this->start_config);

    for (int i=0; i<n_iterations; i++) {
        neighborhood = {};

        //std::cout << "Iteration: " << i << std::endl;
        // sample random configuration or goal_config node
        this->sampleConfig(sampled_config);

        // get a reference of the nearest node in nodes in regard to the sampled config
        Node<Config> &nearest_node = this->getNearestNode(sampled_config);

        //todo: make steering function
        this->getIntermediateConfig(nearest_node.config, sampled_config, new_config);

        //todo: check if intermediate config is not in collision space

        // get vector of pointers to the nearest neighbor nodes
        getNeighborhood(new_config, neighborhood);
        assert(!neighborhood.empty());
        if (i==999999) {
            std::cout << "New Config: " << new_config.transpose() << std::endl;
            for (Node<Config> *node: neighborhood) {
                node->printConfig();
                std::cout << "-->" << this->steer(new_config, node->config) << std::endl;
            }
        }

        // todo: calculate cost of connection from neighbors to new config
        //      make new node and emplace back to nodes
        //      make cheapest neighbor to parent of new node
        //      pushback new node to neighbor children
        //      set cost of new node
        extendFromCheapestNeighbor(neighborhood, new_config);


        // todo: rewire
        //      calculate cost of connection from new node to neighbor
        //      if new node cost + cost of connection < neighbor cost
        //          neighparent = neighbor parent
        //          set neighbor parent = new_node
        //          update neighbor cost
        //          add cost_difference to children (and propagate diff to their children)
        //          if neighbor + cost of connection to old parent < parent cost: do it again
        rewire(neighborhood);
    }
}


template <typename Config>
void RRT_Star<Config>::getNeighborhood(Config &input_config, std::vector<Node<Config>*> &neighborhood) {
    double distance=0;

    for (Node<Config> &node: this->nodes) {
        distance = this->distanceFunction(input_config, node.config);
        if (distance > kNeighborhoodRadius) continue;
        neighborhood.push_back(&node);
    }
}


template <typename Config>
void RRT_Star<Config>::extendFromCheapestNeighbor(std::vector<Node<Config>*> &neighborhood, Config &input_config) {
    double min_cost = std::numeric_limits<double>::max();
    double cost = 0;
    Node<Config> *cheapest_neighbor = nullptr;

    for (Node<Config> *neighbor: neighborhood) {
        cost = this->steer(neighbor->config, input_config);
        if (cost < min_cost) {
            min_cost = cost;
            cheapest_neighbor = neighbor;
        }
    }

    this->nodes.emplace_back(input_config);
    Node<Config> &new_node = this->nodes.back();

    new_node.parent = cheapest_neighbor;
    cheapest_neighbor->children.push_back(&new_node);
    new_node.cost = cheapest_neighbor->cost + min_cost;
}


template <typename Config>
void RRT_Star<Config>::rewire(std::vector<Node<Config>*> &neighborhood) {
    double cost = 0, cost_diff = 0;
    Node<Config> &new_node = this->nodes.back();

    for (Node<Config> *neighbor: neighborhood) {
        cost = this->steer(new_node.config, neighbor->config);

        if (new_node.cost + cost < neighbor->cost) {
            new_node.children.push_back(neighbor);
            neighbor->parent = &new_node;
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
