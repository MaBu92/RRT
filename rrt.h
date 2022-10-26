#ifndef SRC_RRT_H
#define SRC_RRT_H

#include <vector>
#include <iostream>
#include <random>

#include "eigen3/Eigen/Dense"

const int kNumIterations = 1;
const double kGoalSampleRate = 0.5;
const double kExtensionStepSize = 0.5;

using ConfigXYYaw = Eigen::Array<double, 3, 1>;
using ConfigXY = Eigen::Array<double, 2, 1>;

// =====================================================================================================================
//    CONFIGURATION SPACE TEMPLATE STRUCT
// =====================================================================================================================
template <typename Config>
struct ConfigSpace {
    Config min;
    Config max;

    ConfigSpace (double min_x, double max_x, double min_y, double max_y, double min_yaw, double max_yaw)
        : min(min_x, min_y, min_yaw), max(max_x, max_y, max_yaw) {}
    ConfigSpace (Config min_, Config max_) : min(min_), max(max_) {}
};


// =====================================================================================================================
//    NODE TEMPLATE STRUCT
// =====================================================================================================================
template <typename Config>
struct Node {
    double cost = 0;
    std::vector<Node<Config>*> children;
    Node* parent = nullptr;

    Node(Config config_)
        : config(config_) {
        children.reserve(5);
    }
    Config config;

    void printConfig() {std::cout << "Config: (" << config.transpose() << ")" << std::endl;}
    //Node(const Node &n) {std::cout << "Node: " << &n << std::endl;}
};


// =====================================================================================================================
//    RRT TEMPLATE CLASS
// =====================================================================================================================
template <typename Config>
class RRT {
public:
    RRT() {}

    RRT(Config start_, Config goal_, ConfigSpace<Config> config_space_)
        : start(start_), goal(goal_), config_space(config_space_) {}
    void run(int=1);
    void printNodes() {
        for (Node<Config> &node: nodes) node.printConfig();
    }

protected:
    Config start, goal;
    ConfigSpace<Config> config_space;
    std::vector<Node<Config>> nodes;
    //std::function<double(Config&, Config&)> distanceFunction;

    void sampleConfig(Config &config);
    Node<Config>& getNearestNode(Config &input_config);
    void getIntermediateConfig(Config &a, Config &b, Config &c);
    void extendNode(Node<Config> &extended_node, Config &new_config);

    double normalizedSquaredDistance(Config &config1, Config &config2);
    double normalizedDistance(Config &config1, Config &config2);
    double squaredEuklideanDistance(Config &config1, Config &config2);
    double euklideanDistance(Config &config1, Config &config2);
};


// =====================================================================================================================
//    RRT TEMPLATE CLASS - DEFINITIONS
// =====================================================================================================================

template <typename Config>
void RRT<Config>::run(int n_iterations) {
    Config sampled_config, intermediate_config;

    nodes.reserve(n_iterations + 1);
    nodes.emplace_back(start);

    for (int i=0; i<n_iterations; i++) {
        sampleConfig(sampled_config);
        Node<Config> &nearest_node = getNearestNode(sampled_config);
        getIntermediateConfig(nearest_node.config, sampled_config, intermediate_config);
        extendNode(nearest_node, intermediate_config);
    }
}

template <typename Config>
void RRT<Config>::sampleConfig(Config &config) {
    double random_double = (double) rand() / RAND_MAX;

    if (kGoalSampleRate > random_double) {
        config = goal;
        return;
    }

    config = (Config::Random() + 1) / 2;                // Random config with values in range [0, 1]
    config *= (config_space.max - config_space.min);    // Random config with values in range [0, max-min] of config space
    config += config_space.min;                         // Random config with values in range [min, max] of config space
}

template <typename Config>
Node<Config>& RRT<Config>::getNearestNode(Config &input_config) {
    double distance, min_distance=std::numeric_limits<double>::max();
    Node<Config>* nearest_node = nullptr;

    for (Node<Config> &node: nodes) {
        distance = normalizedSquaredDistance(input_config, node.config);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_node = &node;
        }
    }
    return *nearest_node;
}

template <typename Config>
void RRT<Config>::extendNode(Node<Config> &extended_node, Config &new_config) {
    nodes.emplace_back(new_config);
    nodes.back().parent = &extended_node;
    extended_node.children.push_back(&(nodes.back()));
}

template <>
void RRT<ConfigXY>::getIntermediateConfig (ConfigXY &start, ConfigXY &goal, ConfigXY &intermediate) {
    ConfigXY direction = (goal - start);
    double magnitude = sqrt(direction.pow(2).sum());
    double step_factor = 1;

    step_factor = std::min(1., kExtensionStepSize / magnitude);

    intermediate = direction * step_factor + start;
}

template <>
void RRT<ConfigXYYaw>::getIntermediateConfig (ConfigXYYaw &start, ConfigXYYaw &goal, ConfigXYYaw &intermediate) {

    ConfigXYYaw direction = (goal - start) * ConfigXYYaw(1, 1, 0);
    double magnitude = sqrt(direction.pow(2).sum());
    double step_factor = 1;

    step_factor = std::min(1., kExtensionStepSize / magnitude);

    intermediate = (goal - start) * step_factor + start;
}

template <typename Config>
double RRT<Config>::normalizedSquaredDistance(Config &config1, Config &config2) {
    return ((config2 - config1) / (config_space.max - config_space.min)).pow(2).sum();
}

template <typename Config>
double RRT<Config>::normalizedDistance(Config &config1, Config &config2) {
    return sqrt(((config2 - config1) / (config_space.max - config_space.min)).pow(2).sum());
}

template <typename Config>
double RRT<Config>::squaredEuklideanDistance(Config &config1, Config &config2) {
    return (config2 - config1).pow(2).sum();
}

template <typename Config>
double RRT<Config>::euklideanDistance(Config &config1, Config &config2) {
    return sqrt((config2 - config1).pow(2).sum());
}

#endif

