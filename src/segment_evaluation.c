//
// Created by alberto on 12/09/16.
//

#include <assert.h>
#include <memory.h>
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
 * Describes the run of a train on a segment: switching points, switching times, speeds achieved, accelerations.
 * Notice that a run is always made of four phases, therefore we can use static vectors of size 4.
 */
typedef struct SegmentRun {
    float start_positions[DRIVING_PHASES];
    float end_positions[DRIVING_PHASES];
    float start_times[DRIVING_PHASES];
    float end_times[DRIVING_PHASES];
    float start_speeds[DRIVING_PHASES];
    float end_speeds[DRIVING_PHASES];
    float accelerations[DRIVING_PHASES];
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
    const float invalid[DRIVING_PHASES] = {-1.0f, -1.0f, -1.0f, -1.0f};
    memcpy(run->start_positions, invalid, sizeof(invalid));
    memcpy(run->end_positions, invalid, sizeof(invalid));
    memcpy(run->start_times, invalid, sizeof(invalid));
    memcpy(run->end_times, invalid, sizeof(invalid));
    memcpy(run->start_speeds, invalid, sizeof(invalid));
    memcpy(run->end_speeds, invalid, sizeof(invalid));
    memcpy(run->accelerations, invalid, sizeof(invalid));
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
    run.accelerations[MAX_ACCELERATION] = instance->train.max_acceleration;

    size_t ma_start_speed_index = input->e_speed / SPEED_STEP;
    size_t ma_distance_index = input->x1;

    float ma_end_speed = get_max_acceleration_speed(lt, input->segment_id, ma_start_speed_index, ma_distance_index);
    float ma_end_pos = get_max_acceleration_position(lt, input->segment_id, ma_start_speed_index, ma_distance_index);
    float ma_end_time = get_max_acceleration_time(lt, input->segment_id, input->e_speed, ma_distance_index);

    run.end_speeds[MAX_ACCELERATION] = run.start_speeds[CRUISING] = ma_end_speed;
    run.end_positions[MAX_ACCELERATION] = run.start_positions[CRUISING] = ma_end_pos;
    run.end_times[MAX_ACCELERATION] = run.start_times[CRUISING] = ma_end_time;

    // 2) Cruising phase
    size_t cr_start_speed_index = ma_end_speed / SPEED_STEP;
    size_t cr_distance_index = input->x2 - input->x1;

    float cr_start_speed = ma_end_speed;
    float cr_acceleration = cruising_acceleration(instance, input, cr_start_speed);

    run.accelerations[CRUISING] = cr_acceleration;

    float cr_end_speed = get_cruising_speed(cr_start_speed_index, cr_distance_index);
    float cr_end_pos = ma_end_pos + get_cruising_position(cr_start_speed_index, cr_distance_index);
    float cr_end_time = get_cruising_time(cr_start_speed_index, cr_distance_index);

    run.end_speeds[CRUISING] = run.start_speeds[COASTING] = cr_end_speed;
    run.end_positions[CRUISING] = run.start_positions[COASTING] = cr_end_pos;
    run.end_times[CRUISING] = run.start_times[COASTING] = cr_end_time;

    // 3) Coasting phase
    size_t co_start_speed_index = cr_end_speed / SPEED_STEP;
    size_t co_distance_index = input->x3 - input->x2;

    run.accelerations[COASTING] = 0;

    float co_end_speed = get_coasting_speed(lt, input->segment_id, co_start_speed_index, co_distance_index);
    float co_end_pos = cr_end_pos + get_coasting_position(lt, input->segment_id, co_start_speed_index, co_distance_index);
    float co_end_time = get_coasting_time(lt, input->segment_id, co_start_speed_index, co_distance_index);

    run.end_speeds[COASTING] = run.start_speeds[MAX_BRAKING] = co_end_speed;
    run.end_positions[COASTING] = run.start_positions[MAX_BRAKING] = co_end_pos;
    run.end_times[COASTING] = run.start_times[MAX_BRAKING] = co_end_time;

    // 4) Max-braking phase
    size_t mb_start_speed_index = co_end_speed / SPEED_STEP;
    size_t mb_distance_index = (instance->segments[input->segment_id].length / SPEED_STEP) - input->x3;

    run.accelerations[MAX_BRAKING] = -instance->train.max_braking;

    float mb_end_speed = get_max_braking_speed(lt, input->segment_id, mb_start_speed_index, mb_distance_index);
    float mb_end_pos = co_end_pos + get_max_braking_position(lt, input->segment_id, mb_start_speed_index, mb_distance_index);
    float mb_end_time = get_max_braking_time(lt, input->segment_id, mb_start_speed_index, mb_distance_index);

    run.end_speeds[MAX_BRAKING] = mb_end_speed;
    run.end_positions[MAX_BRAKING] = mb_end_pos;
    run.end_times[MAX_BRAKING] = mb_end_time;

    return run;
}

/*
 * api-method
 */
float cost_of_segment(const Instance* instance, const Lookup* lt, const EvaluationInput* input) {
    // TODO
    return 0.0;
}
