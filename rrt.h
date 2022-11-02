#ifndef SRC_RRT_H
#define SRC_RRT_H

#include <vector>
#include <iostream>
#include <random>
#include <functional>
#include <tuple>
#include "eigen3/Eigen/Dense"

#include "node.h"
#include "config_space.h"


const double kGoalSampleRate = 0.5;
const double kExtensionStepSize = 0.5;
const double kPathResolution = 0.05;
const double kDoubleMax = std::numeric_limits<double>::max();

using ConfigXYYaw = Eigen::Array<double, 3, 1>;
using ConfigXY = Eigen::Array<double, 2, 1>;
template <typename Config>
using Path = std::vector<Config>;
template <typename Config>
using DistanceFunction = std::function<double(const Node<Config>&, const Node<Config>&)>;
template <typename Config>
using SteeringFunction = std::function<std::tuple<Path<Config>, double> (Node<Config>&, Node<Config>&, double stepsize)>;



// =====================================================================================================================
//    RRT TEMPLATE CLASS
// =====================================================================================================================
template <typename Config>
class RRT {
public:
    RRT() {}

    RRT(Config start_config, Config goal_config, ConfigSpace<Config> config_space_)
        : start_node(start_config), goal_node(goal_config), config_space(config_space_) {}
    std::vector<Node<Config>> run(int=1);
    void printNodes() {
        for (Node<Config> &node: nodes) node.printConfig();
    }

protected:
    Node<Config> start_node, goal_node;
    ConfigSpace<Config> config_space;
    std::vector<Node<Config>> nodes;

    // Tree Methods
    Node<Config> sampleNode(double goal_sample_rate);
    Node<Config>& getNearestNode(Node<Config> &input_node);
    Node<Config>& extendNode(Node<Config> &start, Node<Config> &goal, double stepsize=kDoubleMax);
    std::vector<Node<Config>> getFinalPath(Node<Config> &current_node);

    // Distance Methods
    static double getDistance(const Node<Config> &start, const Node<Config> &goal);
    static double getSquaredDistance(const Node<Config> &start, const Node<Config> &goal);

    // Steering Method
    static std::tuple<Path<Config>, double> steer(Node<Config> &start, Node<Config> &goal,
                                                  double stepsize=kDoubleMax, bool calculate_path=true);
};



// =====================================================================================================================
//    RRT TEMPLATE CLASS - METHODS
// =====================================================================================================================
template <typename Config>
std::vector<Node<Config>> RRT<Config>::run(int n_iterations) {
    /*
     * arguments:
     *      - n_iterations: Number of iterations/nodes with which the tree is expanded.
     *
     * Extend the tree by placing nodes between a sampled node (random or goal_config) and the
     * closest node in the tree (relative to the sample node). The new node is placed by
     * taking a step from the nearest node into the direction of the sampled node
     * ...
     */

    // reserve memory for the tree nodes saved in nodes and add start as first node
    nodes.reserve(n_iterations + 1);
    nodes.emplace_back(start_node);

    for (int i=0; i<n_iterations; i++) {
        // sample a configuration randomly from config space or use goal configuration and save it in sampled_config
        Node<Config> sample_node = sampleNode(kGoalSampleRate);

        // get a reference to the closest node in the tree with respect to the sample node
        Node<Config> &nearest_node = getNearestNode(sample_node);

        // extend nearest_node towards sample node with a limiting step size -> new_node
        // calculate path and cost of extension for new_node
        // save new_node in nodes. update parent and children of new_node and nearest_node
        extendNode(nearest_node, sample_node, kExtensionStepSize);

        if (getDistance(nodes.back(), goal_node) <= kExtensionStepSize) {
            extendNode(nodes.back(), goal_node);
            std::cout << "Found path with minimal cost of " << nodes.back().getCost() << " after " << i << " iterations." << std::endl;
            return getFinalPath(nodes.back());
        }
    }
    std::cout << "No path found" << std::endl;
    return std::vector<Node<Config>>();
}


template <typename Config>
Node<Config> RRT<Config>::sampleNode(double goal_sample_rate) {
    // get a random value between 0 and 1
    double random_double = (double) rand() / RAND_MAX;

    // return goal sample with probability kGoalSample
    if (goal_sample_rate > random_double) return goal_node;

    Node<Config>(config_space.getRandomConfig());

    // return a random config within config space
    return Node<Config>(config_space.getRandomConfig());
}


template <typename Config>
Node<Config>& RRT<Config>::getNearestNode(Node<Config> &input_node) {
    double distance=0, min_distance=std::numeric_limits<double>::max();
    Node<Config>* nearest_node = nullptr;

    // find nearest node in nodes
    for (Node<Config> &node: nodes) {
        distance = getDistance(input_node, node);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_node = &node;
        }
    }
    return *nearest_node;
}


template <typename Config>
Node<Config>& RRT<Config>::extendNode(Node<Config> &start, Node<Config> &goal, double stepsize) {
    auto [path, cost] = steer(start, goal, stepsize);
    nodes.emplace_back(path.back());
    start.connectToChild(nodes.back(), path, cost);

    return nodes.back();
}


template <typename Config>
std::vector<Node<Config>> RRT<Config>::getFinalPath(Node<Config> &current_node) {
    std::vector<Node<Config>> final_path = {current_node};

    while (current_node.getParent()) {
        current_node = *current_node.getParent();
        final_path.push_back(current_node);
    }

    std::reverse(final_path.begin(), final_path.end());
    return final_path;
}


// ------ DISTANCE METHOD ----------------------------------------------------------------------------------------------
template <typename Config>
double RRT<Config>::getDistance(const Node<Config> &start, const Node<Config> &goal) {
    return sqrt(getSquaredDistance(start, goal));
}


template <typename Config>
double RRT<Config>::getSquaredDistance(const Node<Config> &start, const Node<Config> &goal) {
    return (goal.config - start.config).pow(2).sum();
}


template <>
double RRT<ConfigXYYaw>::getSquaredDistance(const Node<ConfigXYYaw> &start, const Node<ConfigXYYaw> &goal) {
    ConfigXYYaw weight(1, 1, 0);
    return ((goal.config - start.config) * weight).pow(2).sum();
}


// ------ STEERING METHOD ----------------------------------------------------------------------------------------------
template <typename Config>
std::tuple<Path<Config>, double> RRT<Config>::steer(Node<Config> &start, Node<Config> &goal,
                                                    double stepsize, bool calculate_path) {
    double magnitude = getDistance(start, goal);
    Config direction = (goal.config - start.config) / magnitude;

    stepsize = std::min(magnitude, stepsize);

    Path<Config> path;
    path.reserve(std::ceil(stepsize / kPathResolution));
    path.push_back(start.config);
    Config new_config = magnitude<=stepsize ? goal.config : start.config + direction * stepsize;

    if (!calculate_path) {
        path.push_back(new_config);
        return {path, stepsize};
    }

    Config ds = direction * kPathResolution;
    Config temp_config = start.config;
    for (double s=kPathResolution; s <= stepsize-kPathResolution; s+=kPathResolution) {
        temp_config += ds;
        path.push_back(temp_config);
    }

    path.push_back(new_config);
    return {path, stepsize};
}

#endif

