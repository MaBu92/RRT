#ifndef SRC_RRT_H
#define SRC_RRT_H

#include <vector>
#include <iostream>
#include <random>
#include <functional>
#include "eigen3/Eigen/Dense"

#include "node.h"
#include "config_space.h"


const int kNumIterations = 1;
const double kGoalSampleRate = 0.5;
const double kExtensionStepSize = 0.5;

using ConfigXYYaw = Eigen::Array<double, 3, 1>;
using ConfigXY = Eigen::Array<double, 2, 1>;

// =====================================================================================================================
//    CONFIGURATION SPACE TEMPLATE STRUCT
// =====================================================================================================================
//template <typename Config>
//struct ConfigSpace {
//    Config min;
//    Config max;
//
//    ConfigSpace (double min_x, double max_x, double min_y, double max_y, double min_yaw, double max_yaw)
//        : min(min_x, min_y, min_yaw), max(max_x, max_y, max_yaw) {}
//    ConfigSpace (Config min_, Config max_) : min(min_), max(max_) {}
//};


// =====================================================================================================================
//    NODE TEMPLATE STRUCT
// =====================================================================================================================
//template <typename Config>
//struct Node {
//    double cost = 0;
//    std::vector<Node<Config>*> children;
//    Node* parent = nullptr;
//
//    Node(Config config_)
//        : config(config_) {
//        children.reserve(5);
//    }
//    Config config;
//
//    void printConfig() {std::cout << "Config: (" << config.transpose() << ")" << std::endl;}
//};


// =====================================================================================================================
//    EUKLIDEAN DISTANCE 2D
// =====================================================================================================================
template <typename Config>
double euklideanDistance2D(const Config &config1, const Config &config2) {
    Config unit_config_2d = Config::Zero();
    unit_config_2d[0] = 1;
    unit_config_2d[1] = 1;
    return sqrt(((config2 - config1) * unit_config_2d).pow(2).sum());
}

template <>
double euklideanDistance2D(const ConfigXY &config1, const ConfigXY &config2) {
    return sqrt(((config2 - config1)).pow(2).sum());
}


template <typename Config>
double squaredEuklideanDistance2D(const Config &config1, const Config &config2) {
    Config unit_config_2d = Config::Zero();
    unit_config_2d[0] = 1;
    unit_config_2d[1] = 1;
    return ((config2 - config1) * unit_config_2d).pow(2).sum();
}

template <>
double squaredEuklideanDistance2D(const ConfigXY &config1, const ConfigXY &config2) {
    return (config2 - config1).pow(2).sum();
}

// =====================================================================================================================
//    RRT TEMPLATE CLASS
// =====================================================================================================================
template <typename Config>
class RRT {
public:
    RRT() {}

    RRT(Config start_config, Config goal_config, ConfigSpace<Config> config_space_)
        : start_node(start_config), goal_node(goal_config), config_space(config_space_) {}
    void run(int=1);
    void printNodes() {
        for (Node<Config> &node: nodes) node.printConfig();
    }

protected:
    Config start_node, goal_node;
    ConfigSpace<Config> config_space;
    std::vector<Node<Config>> nodes;

    Node<Config> sampleNode();
    Node<Config>& getNearestNode(Node<Config> &input_node);
    Node<Config> getIntermediateNode(Node<Config> &start, Node<Config> &goal, double stepsize);
    void extendNode(Node<Config> to_extend, Node<Config> extend_with);
    double steer(Config &start_config, Config &goal_config);
};


// =====================================================================================================================
//    RRT TEMPLATE CLASS - DEFINITIONS
// =====================================================================================================================

template <typename Config>
void RRT<Config>::run(int n_iterations) {
    /*
     * arguments:
     *      - n_iterations: Number of iterations/nodes with which the tree is expanded.
     *
     * Extend the tree by placing nodes between a sampled node (random or goal_config) and the
     * closest node in the tree (relative to the sample node). The new node is placed by
     * taking a step from the nearest node into the direction of the sampled node
     * ...
     */

    Config new_config;

    // reserve memory for the tree nodes saved in nodes and add start as first node
    nodes.reserve(n_iterations + 1);
    nodes.emplace_back(start_node);

    for (int i=0; i<n_iterations; i++) {
        // sample a configuration randomly from config space or use goal configuration and save it in sampled_config
        Node<Config> sample_node = sampleNode();

        // get a reference to the closest node in the tree with respect to the sample node
        Node<Config> &nearest_node = getNearestNode(sample_node);

        // make a step from nearest_node towards sampled node with a limited step size ans save the resulting
        // configuration in new_config
        Node<Config> new_node = getIntermediateNode(nearest_node, sample_node, kExtensionStepSize);

        // save new_node in nodes. update parent and children of new_node and nearest_node
        extendNode(nearest_node, new_node);
    }
}

template <typename Config>
Node<Config> RRT<Config>::sampleNode() {
    // get a random value between 0 and 1
    double random_double = (double) rand() / RAND_MAX;

    // return goal sample with probability kGoalSample
    if (kGoalSampleRate > random_double) return goal_node;

    // return a random config within config space
    return Node<Config>(config_space.getRandomConfig());
}

template <typename Config>
Node<Config>& RRT<Config>::getNearestNode(Node<Config> &input_node) {
    double distance=0, min_distance=std::numeric_limits<double>::max();
    Node<Config>* nearest_node = nullptr;

    // find nearest node in nodes
    for (Node<Config> &node: nodes) {
        distance = distanceFunction(input_node.config, node.config);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_node = &node;
        }
    }
    return *nearest_node;
}

template <typename Config>
Node<Config> RRT<Config>::getIntermediateNode(Node<Config> &start, Node<Config> &goal, double stepsize) {
    Config direction = (goal.config - start.config);
    double magnitude = start.getDistance(goal);

    double step_factor = std::min(1., stepsize / magnitude);

    Config new_config = direction * step_factor + start.config;
    return Node<Config>(new_config);
};


template <typename Config>
void RRT<Config>::extendNode(Node<Config> to_extend, Node<Config> extend_with) {
    extend_with.setParent(to_extend);
    nodes.push_back(extend_with);
    to_extend.addChild(nodes.back());
}

//template <typename Config>
//double RRT<Config>::steer(Config &start_config, Config &goal_config) {
//    double cost;
//
//    cost = this->normalizedDistance(start_config, goal_config);
//    return cost;
//}

#endif

