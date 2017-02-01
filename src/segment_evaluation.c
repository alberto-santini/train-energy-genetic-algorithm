//
// Created by alberto on 12/09/16.
//

#include <assert.h>
#include "segment_evaluation.h"
#include "lookup.h"
#include "davis.h"

/*
 * implementation-enum
 *
 * Successive driving phases when driving on a segment.
 */
typedef enum DrivingPhase {
    MAX_ACCELERATION = 0,
    CRUISING = 1,
    COASTING = 2,
    MAX_BRAKING = 3
} DrivingPhase;

/*
 * implementation-struct
 *
 * Describes the run of a train on a segment: switching points, switching times, lengths run, speeds achieved.
 * Notice that a run is always made of four phases, therefore we can use static vectors of size 4.
 */
typedef struct SegmentRun {
    float start_positions[DRIVING_PHASES];
    float end_positions[DRIVING_PHASES];
    float run_lengths[DRIVING_PHASES];
    float start_times[DRIVING_PHASES];
    float end_times[DRIVING_PHASES];
    float start_speeds[DRIVING_PHASES];
    float end_speeds[DRIVING_PHASES];
} SegmentRun;

/*
 * implementation method
 */
static float cruising_acceleration(const Instance* instance, const EvaluationInput* input, float speed) {
    assert(input->segment_id < instance->num_segments);

    const Segment* seg = &instance->segments[input->segment_id];
    return - resistance(&instance->train, seg, speed);
}

/*
 * implementation-method
 */
static void invalidate_segment(const Instance* instance, SegmentRun* run, DrivingPhase from) {
    // TODO
}

/*
 * implementation-method
 */
static SegmentRun run_on_segment(const Instance* instance, const Lookup* lt, const EvaluationInput* input) {
    assert(input->segment_id < instance->num_segments);

    const Segment* seg = &instance->segments[input->segment_id];

    assert(input->x1 <= input->x2);
    assert(input->x2 <= input->x3);
    assert(input->x3 <= seg->length / DISTANCE_STEP);

    SegmentRun run;

    // 1) Max-acceleration phase
    run.start_speeds[MAX_ACCELERATION] = input->e_speed;
    run.start_positions[MAX_ACCELERATION] = 0.0f;
    run.start_times[MAX_ACCELERATION] = input->e_time;

    size_t ma_start_speed_index = input->e_speed / SPEED_STEP;
    size_t ma_distance_index = input->x1;

    float ma_end_speed = get_max_acceleration_speed(lt, input->segment_id, ma_start_speed_index, ma_distance_index);
    float ma_end_pos = get_max_acceleration_position(lt, input->segment_id, ma_start_speed_index, ma_distance_index);
    float ma_end_time = get_max_acceleration_time(lt, input->segment_id, input->e_speed, ma_distance_index);

    run.end_speeds[MAX_ACCELERATION] = run.start_speeds[CRUISING] = ma_end_speed;
    run.end_positions[MAX_ACCELERATION] = run.start_positions[CRUISING] = ma_end_pos;
    run.end_times[MAX_ACCELERATION] = run.start_times[CRUISING] = ma_end_time;
    run.run_lengths[MAX_ACCELERATION] = ma_end_pos;

    // 2) Cruising phase
    size_t cr_start_speed = ma_end_speed;
    float cr_distance = DISTANCE_STEP * (input->x2 - input->x1);
    float cr_acceleration = cruising_acceleration(instance, input, cr_start_speed);

    // Cannot mantain cruising acceleration!
    if(cr_acceleration > instance->train.max_acceleration) {
        invalidate_segment(instance, &run, CRUISING);
    }

    // TODO
}

/*
 * api-method
 */
float cost_of_segment(const Instance* instance, const Lookup* lt, const EvaluationInput* input) {
    // TODO
    return 0.0;
}
