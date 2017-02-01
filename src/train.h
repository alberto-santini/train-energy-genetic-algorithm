//
// Created by alberto on 04/09/16.
//

#ifndef TEGA_TRAIN_H
#define TEGA_TRAIN_H

#include <stdint.h>

typedef enum TrainType {
    SNCF_TGV = 10,
    SNCF_NORMAL = 11,
    DB_ICE = 20,
    DB_NORMAL = 21
} TrainType;

/**
 * Representation of the train we are scheduling.
 */
typedef struct Train {
    const TrainType       type;               // Train type, used to calculate e.g. the resistance
    const uint_fast32_t   num_coaches;        // Number of coach cars
    const float           mass;               // Mass in kg
    const float           mass_per_axle;      // Mass per axle in kg
    const float           max_acceleration;   // Maximum forward acceleration in m/(s^2)
    const float           max_braking;        // Maximum breaking deceleration in m/(s^2)
    const float           length;             // Length of the train in m
} Train;

/**
 * Prints information about the train
 * @param train     The train
 */
void print_train(const Train* train);

#endif //TEGA_TRAIN_H
