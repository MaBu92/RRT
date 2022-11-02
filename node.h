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
    int id=-1;

    Node() {};
    Node(Config config_): config(config_), x(config_[0]), y(config[1]), z(config[2]) {}

    void updateChildIds();
    void connectToChild(Node<Config> &child, Path<Config> path, double cost);
    void reconnectToNewParent(Node<Config> &new_parent, Path<Config> path, double cost);
    void propagateCostToChildren(double difference);
    void printConfig();
    double getCost();
    int getChildId();
    std::vector<Node<Config>*> getChildren();
    Node<Config>* getParent();

private:
    int child_id = -1;
    std::vector<Node<Config>*> children = {};
    Node<Config>* parent = nullptr;
    Path<Config> path = {};
    double cost = 0;
};

template <typename Config>
double Node<Config>::getCost() {
    return this->cost;
}

template <typename Config>
int Node<Config>::getChildId() {
    return this->child_id;
}

template <typename Config>
Node<Config>* Node<Config>::getParent() {
    return this->parent;
}

template <typename Config>
std::vector<Node<Config>*> Node<Config>::getChildren() {
    return this->children;
}

template <typename Config>
void Node<Config>::updateChildIds() {
    for (int i=0; i<children.size(); i++) {
        children[i]->child_id = i;
    }
}

template <typename Config>
void Node<Config>::connectToChild(Node<Config> &child, Path<Config> path, double cost) {
    this->children.push_back(&child);
    child.child_id = this->children.size() - 1;
    child.parent = this;
    child.path = path;
    child.cost =  this->cost + cost;
}

template <typename Config>
void Node<Config>::reconnectToNewParent(Node<Config> &new_parent, Path<Config> path, double cost) {
    if (this->id==141 && new_parent.id == 915) {
        std::cout << "dummy" << std::endl;
    }
    double cost_diff = new_parent.cost + cost - this->cost;                             // calculate difference in cost

    this->parent->children.erase(this->parent->children.begin() + this->child_id);      // remove node from children of parent node

    this->parent->updateChildIds();

    this->parent = &new_parent;                                                         // change parent to new parent

    new_parent.children.push_back(this);                                                // add node as child of new parent
    this->child_id = new_parent.children.size() - 1;

    //update path and cost
    this->path = path;

    std::cout << "cost_diff: " << cost_diff << std::endl;
    this->propagateCostToChildren(cost_diff);
}

template <typename Config>
void Node<Config>::propagateCostToChildren(double difference) {
    for (Node<Config> *child: this->children) {
        std::cout << "node: " << child->id << std::endl;
        std::cout << "cost: " << child->cost << std::endl;
        std::cout << "diff: " << difference << std::endl;
        child->cost += difference;
        std::cout << "new cost: " << child->cost << std::endl;
        std::cout << "---------> down" << std::endl;
        child->propagateCostToChildren(difference);
        std::cout << "<--------- up" << std::endl;
    }
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
