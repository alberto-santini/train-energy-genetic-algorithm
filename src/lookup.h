//
// Created by alberto on 05/09/16.
//

#ifndef TEGA_LOOKUP_H
#define TEGA_LOOKUP_H

#include "instance.h"

// Discretisation step for speeds [m/s]
#define SPEED_STEP  5.0f

// Discretisation step for distances [m]
#define DISTANCE_STEP 50.0f

/**
 * Lookup table for a particular driving style (max acceleration, coasting, max braking).
 */
typedef struct LookupForDrivingStyle {
    // WARNING:
    // For memory contiguity purposes, all arrays contained in this structure are flattened.

    /**
     * Table with running times.
     */
    float* time;

    /**
     * Table with final speeds.
     */
    float* speed;

    /**
     * Table with final positions.
     */
    float* position;
} LookupForDrivingStyle;

/**
 * Container for look-up tables used by the algorithm
 */
typedef struct Lookup {
    /**
     * Second dimension of the tables
     */
    size_t speeds_n;

    /**
     * Third dimension of the tables
     */
    size_t lengths_n;

    /**
     * Maximum Acceleration Driving Style.
     *
     * Running time table:
     * Given a segment i, a speed v = j * SPEED_STEP, and a length l = k * DISTANCE_STEP,
     * max_acceleration.time[i][j][k] gives the time it will take to move the train on
     * segment i, for length l, with initial speed v, applying maximum acceleration.
     *
     * Final speed table:
     * Same as for time, but here we get the final speed reached.
     *
     * Final position table:
     * The final position will always correspond to the end of the run (k * DISTANCE_STEP).
     * We assume that when applying maximum acceleration, the train is always able to move
     * forward.
     */
    LookupForDrivingStyle max_acceleration;

    /**
     * Coasting Driving Style.
     *
     * Running time table:
     * Similar to the one for max acceleration, but here we are coasting.
     *
     * Final speed table:
     * Similar as for time, but here we get the final speed reached. If the final speed is
     * 0 at some point, that will be the final speed: we do not allow it to become negative.
     * Rather, we interrupt the "simulation" at that point and set the final position
     * accordingly.
     *
     * Final position table:
     * If the final speed is > 0, the final position will correspond to the end of the run
     * (k * DISTANCE_STEP). Otherwise, this will be the position at which the speed becomes
     * zero.
     */
    LookupForDrivingStyle coasting;

    /**
     * Maximum Braking Driving Style
     *
     * Running time table:
     * Similar to the one for max acceleration and coasting, but here we are decelerating
     * applying the maximum allowed braking power.
     *
     * Final speed table:
     * If the train cannot stop before running the whole length, this value will be its speed
     * (> 0) after running the distance k * DISTANCE_STEP. Otherwise, it will be 0.
     *
     * Final position table:
     * If the train canno stop before running the whole length, this value will be the distance
     * k * DISTANCE SPEED. Otherwise, it will be the position at which the train stops (i.e.
     * reaches velocity = 0).
     */
    LookupForDrivingStyle max_braking;
} Lookup;

/*
 * All the following methods access a specific element of a 3D flattened lookup table.
 * Notice that the values should be >= 0; if a value < 0 is obtained, this will mean that
 * the particular entry is not available in the lookup table.
 */
float get_max_acceleration_time(const Lookup* l, size_t segment, size_t speed, size_t distance);
float get_max_acceleration_speed(const Lookup* l, size_t segment, size_t speed, size_t distance);
float get_max_acceleration_position(const Lookup* l, size_t segment, size_t speed, size_t distance);
float get_coasting_time(const Lookup* l, size_t segment, size_t speed, size_t distance);
float get_coasting_speed(const Lookup* l, size_t segment, size_t speed, size_t distance);
float get_coasting_position(const Lookup* l, size_t segment, size_t speed, size_t distance);
float get_max_braking_time(const Lookup* l, size_t segment, size_t speed, size_t distance);
float get_max_braking_speed(const Lookup* l, size_t segment, size_t speed, size_t distance);
float get_max_braking_position(const Lookup* l, size_t segment, size_t speed, size_t distance);

/**
 * Initialises the lookup tables
 * @param instance  The instance we are solving
 * @return          The lookup tables
 */
Lookup generate_lookup_tables(const Instance* instance);

/**
 * Frees the memory used by the lookup table
 * @param lookup
 */
void free_lookup_tables(Lookup* lookup);

/**
 * Print a layout of the lookup tables
 * @param l         The lookup tables
 * @param instance  The instance
 */
void print_lookup_tables(const Lookup* l, const Instance* instance);

#endif //TEGA_LOOKUP_H
