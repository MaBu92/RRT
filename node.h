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
    Node(Config config_): config_(config_), id_(counter_++) {}

    void printConfig() const;
    bool hasParent() const;
    double getCost() const;
    Config const& getConfig() const;
    Node<Config>& getParent() const;

    void connectToChild(Node<Config> &child, Path<Config> path, double cost);
    void reconnectToNewParent(Node<Config> &new_parent, Path<Config> path, double cost);

private:
    static int counter_;

    int id_=-1, child_id_=-1;
    double cost_=0;
    Config config_;
    Node<Config> *parent_ = nullptr;
    std::vector<Node<Config>*> children_;
    Path<Config> path_;

    void updateChildIds();
    void propagateCostToChildren(double difference);
};

template <typename Config>
int Node<Config>::counter_ = 0;

template <typename Config>
double Node<Config>::getCost() const {
    return this->cost_;
}

template <typename Config>
Config const& Node<Config>::getConfig() const {
    return this->config_;
}

template <typename Config>
Node<Config>& Node<Config>::getParent() const {
    return *this->parent_;
}

template <typename Config>
bool Node<Config>::hasParent() const {
    return this->parent_;
}

template <typename Config>
void Node<Config>::updateChildIds() {
    for (int i=0; i < children_.size(); i++) {
        children_[i]->child_id_ = i;
    }
}

template <typename Config>
void Node<Config>::connectToChild(Node<Config> &child, Path<Config> path, double cost) {
    this->children_.push_back(&child);
    child.child_id_ = this->children_.size() - 1;
    child.parent_ = this;
    child.path_ = path;
    child.cost_ = this->cost_ + cost;
}

template <typename Config>
void Node<Config>::reconnectToNewParent(Node<Config> &new_parent, Path<Config> path, double cost) {
    double cost_diff = new_parent.cost_ + cost - this->cost_;                             // calculate difference in cost

    this->parent_->children_.erase(this->parent_->children_.begin() + this->child_id_);      // remove node from children of parent node

    this->parent_->updateChildIds();

    this->parent_ = &new_parent;                                                         // change parent to new parent

    new_parent.children_.push_back(this);                                                // add node as child of new parent
    this->child_id_ = new_parent.children_.size() - 1;

    //update path and cost
    this->path_ = path;

    this->propagateCostToChildren(cost_diff);
}

template <typename Config>
void Node<Config>::propagateCostToChildren(double difference) {
    this->cost_ += difference;
    for (Node<Config> *child: this->children_) {
        child->propagateCostToChildren(difference);
    }
}

template <typename Config>
void Node<Config>::printConfig() const {
    std::cout << "Config: (" << config_.transpose() << ")" << std::endl;
}


// =====================================================================================================================
//    RRT* TEMPLATE CLASS - Tests
// =====================================================================================================================

//void checkForCirclesInTree
//
//void checkIfChildsHaveThisParent

#endif //SRC_NODE_H
