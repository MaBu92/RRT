//
// Created by marvin on 30.10.22.
//

#ifndef SRC_NODE_H
#define SRC_NODE_H

#include <iostream>
#include "eigen3/Eigen/Dense"

using ConfigXYYaw = Eigen::Array<double, 3, 1>;
using ConfigXY = Eigen::Array<double, 2, 1>;
template <typename Config>
using Path = std::vector<Config>;


template <typename Config>
class Node {
public:
    Config config;
    double x, y, z;
    int id;
    double cost = 0;
    std::vector<Node<Config>*> children = {};
    Node<Config>* parent = nullptr;
    Path<Config> path = {};

    Node() {};
    Node(Config config_): config(config_), x(config_[0]), y(config[1]), z(config[2]) {}

    void setParent(Node<Config> &parent);
    void addChild(Node<Config> &child);
    void printConfig();
};


template <typename Config>
void Node<Config>::setParent(Node<Config> &parent) {
    this->parent = &parent;
}

template <typename Config>
void Node<Config>::addChild(Node<Config> &child) {
    this->children.push_back(&child);
}

template <typename Config>
void Node<Config>::printConfig() {
    std::cout << "Config: (" << config.transpose() << ")" << std::endl;
}








//// =====================================================================================================================
////    EUKLIDEAN DISTANCE 2D
//// =====================================================================================================================
//template <typename Config>
//double euklideanDistance2D(const Config &config1, const Config &config2) {
//    Config unit_config_2d = Config::Zero();
//    unit_config_2d[0] = 1;
//    unit_config_2d[1] = 1;
//    return sqrt(((config2 - config1) * unit_config_2d).pow(2).sum());
//}
//
//template <>
//double euklideanDistance2D(const ConfigXY &config1, const ConfigXY &config2) {
//    return sqrt(((config2 - config1)).pow(2).sum());
//}
//
//
//template <typename Config>
//double squaredEuklideanDistance2D(const Config &config1, const Config &config2) {
//    Config unit_config_2d = Config::Zero();
//    unit_config_2d[0] = 1;
//    unit_config_2d[1] = 1;
//    return ((config2 - config1) * unit_config_2d).pow(2).sum();
//}
//
//template <>
//double squaredEuklideanDistance2D(const ConfigXY &config1, const ConfigXY &config2) {
//    return (config2 - config1)).pow(2).sum();
//}
//
//
//
//
//// =====================================================================================================================
////    EUKLIDEAN DISTANCE 2D
//// =====================================================================================================================
//template <typename Config>
//double RRT<Config>::normalizedSquaredDistance(Config &config1, Config &config2) {
//    return ((config2 - config1) / (config_space.max - config_space.min)).pow(2).sum();
//}
//
//template <typename Config>
//double RRT<Config>::normalizedDistance(Config &config1, Config &config2) {
//    return sqrt(((config2 - config1) / (config_space.max - config_space.min)).pow(2).sum());
//}
//
//template <typename Config>
//double RRT<Config>::squaredEuklideanDistance(Config &config1, Config &config2) {
//    return (config2 - config1).pow(2).sum();
//}
//
//template <typename Config>
//double RRT<Config>::euklideanDistance(Config &config1, Config &config2) {
//    return sqrt((config2 - config1).pow(2).sum());
//}

#endif //SRC_NODE_H
