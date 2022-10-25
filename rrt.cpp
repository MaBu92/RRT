#include "rrt.h"

const ConfigXY kStart(2, 2);
const ConfigXY kGoal(18, 18);
const ConfigXY kMin(0, 0);
const ConfigXY kMax(20, 20);

const ConfigXYYaw kStart2(2, 2, 20);
const ConfigXYYaw kGoal2(18, 18, 100);
const ConfigXYYaw kMin2(0, 0, 0);
const ConfigXYYaw kMax2(20, 20, 360);


template <typename Config>
void RRT<Config>::run() {
    Config sampled_config, intermediate_config;

    sampleConfig(sampled_config);
    Node<Config> &nearest_node = getNearestNode(sampled_config);
    getIntermediateConfig(nearest_node.config, sampled_config, intermediate_config);
    extendNode(nearest_node, intermediate_config);
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
    nodes.emplace_back(Node<Config>(new_config));
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

template<>
double RRT<ConfigXY>::normalizedSquaredDistance(ConfigXY &config1, ConfigXY &config2) {
    return ((config2 - config1).abs() / (kMax - kMin)).pow(2).sum();
}

template<>
double RRT<ConfigXYYaw>::normalizedSquaredDistance(ConfigXYYaw &config1, ConfigXYYaw &config2) {
    return ((config2 - config1).abs() / (kMax2 - kMin2)).pow(2).sum();
}

int main() {
    srand(time(NULL));

    const ConfigSpace<ConfigXY> config_space(kMin, kMax);
    RRT<ConfigXY> rrt(kStart, kGoal, config_space);
    rrt.run();

    const ConfigSpace<ConfigXYYaw> config_space2(kMin2, kMax2);
    RRT<ConfigXYYaw> rrt2(kStart2, kGoal2, config_space2);
//    rrt2.run();
    return 0;
}

