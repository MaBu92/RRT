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
        std::cout << "Execution Time: " << ms << "ms" << std::endl;
    }
};


int main() {
    Timer timer;

    srand(time(NULL));

    const ConfigSpace<ConfigXY> config_space(kMin, kMax);
    RRT<ConfigXY> rrt(kStart, kGoal, config_space);
    rrt.run(10);

    const ConfigSpace<ConfigXYYaw> config_space2(kMin2, kMax2);
    RRT<ConfigXYYaw> rrt2(kStart2, kGoal2, config_space2);
    rrt2.run(10);


    RRT_Star<ConfigXYYaw> rrt3(kStart2, kGoal2, config_space2);
    rrt3.run(10);
    return 0;
}