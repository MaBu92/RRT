//
// Created by mab on 26.10.22.
//
#include <chrono>

#include "rrt.h"
#include "rrt_star.h"

const ConfigXY kStart(2, 2);
const ConfigXY kGoal(18, 18);
const ConfigXY kMin(0, 0);
const ConfigXY kMax(20, 20);

const ConfigXYYaw kStart2(2, 2, 20);
const ConfigXYYaw kGoal2(18, 18, 100);
const ConfigXYYaw kMin2(0, 0, 0);
const ConfigXYYaw kMax2(20, 20, 360);

struct Timer {
    std::chrono::high_resolution_clock::time_point start, end;
    std::chrono::duration<float> duration;

    Timer() {
        start = std::chrono::high_resolution_clock::now();
    }
    ~Timer() {
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;
        float ms = duration.count() * 1000.0f;
        std::cout << "\nExecution Time: " << ms << "ms" << std::endl;
    }
};


int main() {
    Timer timer;

    srand(3);

    int n = 1000;

//    const ConfigSpace<ConfigXY> config_space(kMin, kMax);
//    RRT<ConfigXY> rrt(kStart, kGoal, config_space);
//    Path<Node<ConfigXY>> path = rrt.run(n);
//    for (Node<ConfigXY> &node: path) {
//        node.printConfig();
//    }



    const ConfigSpace<ConfigXYYaw> config_space2(kMin2, kMax2);
    RRT<ConfigXYYaw> rrt2(kStart2, kGoal2, config_space2);
    Path<Node<ConfigXYYaw>> path2 = rrt2.run(n);
//    for (Node<ConfigXYYaw> &node: path2) {
//        node.printConfig();
//    }


    RRT_Star<ConfigXYYaw> rrt3(kStart2, kGoal2, config_space2);
    Path<Node<ConfigXYYaw>> path3 = rrt3.run(n);
//    for (Node<ConfigXYYaw> &node: path3) {
//        node.printConfig();
//    }

    Eigen::Array<double, 3, 1> weight(1, 1, 0);
    double min_cost = sqrt(((kGoal2 - kStart2)*weight).pow(2).sum());
//    std::cout << path.back().cost / min_cost * 100 << std::endl;
    std::cout << path2.back().cost / min_cost * 100 << std::endl;
    std::cout << path3.back().cost / min_cost * 100 << std::endl;
}