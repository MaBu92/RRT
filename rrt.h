#include <vector>
#include <iostream>
#include <random>

#include <eigen3/Eigen/Dense>

const int kNumIterations = 1;
const double kGoalSampleRate = 0.5;
const double kExtensionStepSize = 0.5;

using ConfigXYYaw = Eigen::Array<double, 3, 1>;
using ConfigXY = Eigen::Array<double, 2, 1>;

template <typename Config>
struct ConfigSpace {
    Config min;
    Config max;

    ConfigSpace (double min_x, double max_x, double min_y, double max_y, double min_yaw, double max_yaw)
        : min(min_x, min_y, min_yaw), max(max_x, max_y, max_yaw) {}
    ConfigSpace (Config min_, Config max_) : min(min_), max(max_) {}
};

template <typename Config>
struct Node {
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

template <typename Config>
class RRT {
public:
    RRT(Config start_, Config goal_, ConfigSpace<Config> config_space_)//, std::function<double(Config&, Config&)> distFunc)
        : start(start_), goal(goal_), config_space(config_space_)//, distanceFunction(distFunc)
    {
        nodes.reserve(kNumIterations + 1);
        nodes.push_back(start);
    }
    void run();
    void printNodes() {
        for (Node<Config> &node: nodes) node.printConfig();
    }
private:
    Config start, goal;
    ConfigSpace<Config> config_space;
    std::vector<Node<Config>> nodes;
    //std::function<double(Config&, Config&)> distanceFunction;

    void sampleConfig(Config &config);
    Node<Config>& getNearestNode(Config &input_config);
    void getIntermediateConfig(Config &a, Config &b, Config &c);
    void extendNode(Node<Config> &extended_node, Config &new_config);
    double normalizedSquaredDistance(Config &config1, Config &config2);
};

//template <typename Config>
//double exampleDistFunc(Config &config1, Config &config2) {
//    return (config2 - config1).pow(2).sum();
//}