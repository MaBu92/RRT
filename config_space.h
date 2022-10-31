//
// Created by marvin on 30.10.22.
//

#ifndef SRC_CONFIG_SPACE_H
#define SRC_CONFIG_SPACE_H


template <typename Config>
struct ConfigSpace {
    Config min;
    Config max;

    ConfigSpace (double min_x, double max_x, double min_y, double max_y, double min_yaw, double max_yaw)
            : min(min_x, min_y, min_yaw), max(max_x, max_y, max_yaw) {}
    ConfigSpace (Config min_, Config max_) : min(min_), max(max_) {}

    Config getRandomConfig() {
        Config config = (Config::Random() + 1) / 2;     // Random config with values in range [0, 1]
        return config * (max - min) + min;              // Random config with values in range [min, max]
    }
};

#endif //SRC_CONFIG_SPACE_H
