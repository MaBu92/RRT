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
    this->nodes[0].id = 0;

    //std::vector<Node<Config>*> neighborhood;
    std::vector<Node<Config>*> neighborhood;
    double goal_sample_rate = kGoalSampleRate;
    int goal_node_index = -1;

    for (int i=0; i<n_iterations; i++) {
//std::cout << i << std::endl;
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
        new_node.id = i+1;

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


        // todo: rewire
        //      calculate cost of connection from new node to neighbor
        //      if new node cost + cost of connection < neighbor cost
        //          neighparent = neighbor parent
        //          set neighbor parent = new_node
        //          update neighbor cost
        //          add cost_difference to children (and propagate diff to their children)
        //          if neighbor + cost of connection to old parent < parent cost: do it again

//        std::cout << "---------- started" << std::endl;
//        std::vector<int> seen;
//        checkConnection(this->nodes[0], seen);
//        std::cout << "---------- finished" << std::endl;
//        std::cout << "---------- started2" << std::endl;
//        checkParentChild(this->nodes[0]);
//        std::cout << "---------- finished2" << std::endl;

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
    if (new_node.id == 915 || new_node.id ==141 || new_node.id ==281 || new_node.id ==484 || new_node.id ==865) {
        std::cout << "dummy" << std::endl;
    }
    double cost = 0, cost_diff = 0;

    for (Node<Config> *neighbor: neighborhood) {
        if (neighbor->id == 915 || neighbor->id ==141 || neighbor->id ==281 || neighbor->id ==484 || neighbor->id ==865) {
            std::cout << "dummy" << std::endl;
        }
        auto [path, cost] = this->steer(new_node, *neighbor);

        if (new_node.getCost() + cost >= neighbor->getCost()) continue;

        neighbor->reconnectToNewParent(new_node, path, cost);
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

template <typename Config>
void checkConnection(Node<Config> &node, std::vector<int> &seen) {
    std::cout << "curr node: " << node.id << std::endl;

    if (std::find(seen.begin(), seen.end(), node.id) == seen.end()) {
        seen.push_back(node.id);
    } else {
        std::cout << "found a reconecting node" << std::endl;
        std::cout << "id: " << node.id << std::endl;
        throw ("jochen");
    }
    for (Node<Config> *child: node.getChildren()) {
        checkConnection(*child, seen);
    }
}

template <typename Config>
void checkParentChild(Node<Config> &node) {
    std::cout << "curr node: " << node.id << std::endl;

    for (Node<Config> *child: node.getChildren()) {
        if (child->getParent()->id != node.id) {
            std::cout << "child: " << child->id;
            throw ("jochen2");
        }
        checkParentChild(*child);

    }
}

#endif //SRC_RRT_STAR_H
