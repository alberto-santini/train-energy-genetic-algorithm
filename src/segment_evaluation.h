//
// Created by alberto on 12/09/16.
//

#include "instance.h"
#include "lookup.h"

#ifndef TEGA_SEGMENT_EVALUATION_H
#define TEGA_SEGMENT_EVALUATION_H

/*
 * Penalty for each [m/s] above the maximum allowed speed.
 */
#define SPEED_EXCESS_PENALTY    100

/*
 * Penalty for each [m] the train ran short (i.e. stopped before the end of the segment).
 */
#define SHORT_RUN_PENALTY       100

/*
 * Penalty for each [s] of difference between actual and desired arrival time.
 */
#define RUN_TIME_PENALTY        10

/*
 * Number of driving phases on a segment (max acceleration, crusing, coasting, max braking).
 */
#define DRIVING_PHASES          4

/**
 * Data that goes as input to the evaluation and describe the state of the run up to the
 * current segment, and the switching points.
 */
typedef struct EvaluationInput {
    /**
     * Segment progressive id in the segment list.
     */
    size_t segment_id;

    /**
     * Index for the distance where max acceleration ends (distance = x1 * DISTANCE_STEP)
     */
    size_t x1;

    /**
     * Index for the distance where cruising ends (distance = x2 * DISTANCE_STEP)
     */
    size_t x2;

    /**
     * Index for the distance where coasting ends (distance = x3 * DISTANCE_STEP)
     */
    size_t x3;

    /**
     * Entrance speed.
     */
    float e_speed;

    /**
     * Entry time.
     */
    float e_time;

} EvaluationInput;

/**
 * Gives the cost of driving through a segment with given break points.
 * The cost is given by:
 *  1) Energy spent at maximum acceleration
 *  2) Energy spent at crusing speed
 *  3) Penalties:
 *      a) Crusing speed exceeds speed limit at present segment
 *      b) Final speed exceeds speed limit at next segment (if any)
 *      c) Rest position of the train occurs before the segment ends
 *      d) Final (arrival) time for this segment is far from the desired time (if any)
 *
 *  @param  instance The instance considered
 *  @param  lt       Look-up tables to be used in the calculations
 *  @param  input    Input state used for the evaluation.
 *  @return          The cost of the segment with the driving style implied by x1, x2, and x3
 */
float cost_of_segment(const Instance* instance, const Lookup* lt, const EvaluationInput* input);

#endif //TEGA_SEGMENT_EVALUATION_H_H
