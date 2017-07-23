//
// Created by alberto on 05/09/16.
//

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "lookup.h"
#include "davis.h"
#include "eps.h"

/*
 * implementation-method
 */
static float lookup_table_element(const Lookup* l, const float* table, size_t segment, size_t speed, size_t distance) {
    return *(   table +
                segment * l->speeds_n * l->lengths_n +
                speed * l->lengths_n +
                distance);
}

/*
 * implementation-method
 */
static void set_lookup_table_element(Lookup* l, float* table, size_t segment, size_t speed, size_t distance, float value) {
    *(   table +
         segment * l->speeds_n * l->lengths_n +
         speed * l->lengths_n +
         distance) = value;
}

/*
 * implementation-method
 */
static void free_lookup_tables_for_driving_stlye(LookupForDrivingStyle* lt) {
    free(lt->speed); lt->speed = NULL;
    free(lt->time); lt->time = NULL;
    free(lt->position); lt->position = NULL;
}

/*
 * implementation-method
 */
static LookupForDrivingStyle empty_lookup_table_for_driving_style(size_t segments_n, size_t speeds_n, size_t lengths_n) {
    LookupForDrivingStyle lt;

    lt.speed = malloc(segments_n * speeds_n * lengths_n * sizeof(*lt.speed));
    lt.time = malloc(segments_n * speeds_n * lengths_n * sizeof(*lt.time));
    lt.position = malloc(segments_n * speeds_n * lengths_n * sizeof(*lt.position));

    if(lt.speed == NULL || lt.time == NULL || lt.position == NULL) {
        printf("Could not allocate memory for look-up tables\n");
        exit(EXIT_FAILURE);
    }

    for(size_t i = 0; i < segments_n * speeds_n * lengths_n; i++) {
        lt.speed[i] = -1.0f;
        lt.time[i] = -1.0f;
        lt.position[i] = -1.0f;
    }

    return lt;
}

/*
 * implementation-method
 */
static void generate_lookup_table_for_acceleration(const Instance* instance, Lookup* l, LookupForDrivingStyle* lt, float train_acceleration) {
    for(size_t i = 0; i < instance->num_segments; i++) {
        size_t j = 0;

        while(j * SPEED_STEP <= instance->segments[i].speed_limit) {
            float cur_speed = j * SPEED_STEP;
            float cur_time = 0;
            float cur_position = 0;

            // When distance = 0, everything is 0
            set_lookup_table_element(l, lt->speed, i, j, 0, cur_speed);
            set_lookup_table_element(l, lt->time, i, j, 0, 0);
            set_lookup_table_element(l, lt->position, i, j, 0, 0);

            // For 0-length segments, only speed = 0 should be considered
            if(instance->segments[i].length <= SEGMENT_LENGTH_EPS) { break; }

            size_t k = 1;

            while(k * DISTANCE_STEP <= instance->segments[i].length) {
                float acc = train_acceleration + resistance(&instance->train, &instance->segments[i], cur_speed);
                float final_time;
                float final_speed;
                float final_position;
                float distance_to_run = k * DISTANCE_STEP - cur_position;

                if(acc > ACCELERATION_EPS) {
                    // Uniformly accelerated linear motion

                    float running_time = (sqrtf(powf(cur_speed, 2) + 2 * acc * distance_to_run) - cur_speed) / (2 * acc);
                    final_time = cur_time + running_time;
                    final_speed = cur_speed + acc * running_time;
                    final_position = cur_position + distance_to_run; // Move for the whole length requested

                    // printf("Uniformly accelerated linear motion\n");
                    // printf("i: %zu, j: %zu, k: %zu\n", i, j, k);
                    // printf("Cur time: %.2f, cur speed: %.2f, cur position: %.2f\n", cur_time, cur_speed, cur_position);
                    // printf("Fin time: %.2f, fin speed: %.2f, fin position: %.2f\n", final_time, final_speed, final_position);
                    // printf("Train acc: %.2f, Total acc: %.2f\n", train_acceleration, acc);
                    assert(running_time >= 0);
                    assert(final_position > cur_position + distance_to_run - DISTANCE_EPS);
                    assert(final_position > k * DISTANCE_STEP - DISTANCE_EPS);
                } else if(acc > - ACCELERATION_EPS) {
                    // Uniform linear motion

                    if(cur_speed > SPEED_EPS) {
                        // Moving forward with uniform linear motion (constant velocity)
                        final_time = cur_time + distance_to_run / cur_speed;
                        final_speed = cur_speed;
                        final_position = cur_position + distance_to_run; // Move for the whole length requested

                        // printf("Uniform linear motion\n");
                        // printf("i: %zu, j: %zu, k: %zu\n", i, j, k);
                        // printf("Cur time: %.2f, cur speed: %.2f, cur position: %.2f\n", cur_time, cur_speed, cur_position);
                        // printf("Fin time: %.2f, fin speed: %.2f, fin position: %.2f\n", final_time, final_speed, final_position);
                        // printf("Train acc: %.2f, Total acc: %.2f\n", train_acceleration, acc);
                        assert(final_position > cur_position + distance_to_run - DISTANCE_EPS);
                        assert(final_position > k * DISTANCE_STEP - DISTANCE_EPS);
                    } else if(cur_speed > -SPEED_EPS) {
                        // Standing still: impossible to run the length required
                        final_time = cur_time;
                        final_speed = cur_speed;
                        final_position = cur_position;
                    } else {
                        // Going backwards: we really don't want this to happen!
                        set_lookup_table_element(l, lt->speed, i, j, k, -1.0f);
                        set_lookup_table_element(l, lt->time, i, j, k, -1.0f);
                        set_lookup_table_element(l, lt->position, i, j, k, -1.0f);

                        k++; continue;
                    }
                } else {
                    // Uniformly decelerated motion

                    float running_length = - powf(cur_speed, 2) / (2 * acc);

                    if(running_length >= DISTANCE_STEP) {
                        // The train will not stop before it runs all the length distance_to_run

                        float running_time = (sqrtf(powf(cur_speed, 2) + 2 * acc * distance_to_run) - cur_speed) / (2 * acc);
                        final_time = cur_time + running_time;
                        final_speed = cur_speed + acc * running_time;
                        final_position = cur_position + distance_to_run;

                        // printf("Uniformly decelerated linear motion (full distance)\n");
                        // printf("i: %zu, j: %zu, k: %zu\n", i, j, k);
                        // printf("Cur time: %.2f, cur speed: %.2f, cur position: %.2f\n", cur_time, cur_speed, cur_position);
                        // printf("Fin time: %.2f, fin speed: %.2f, fin position: %.2f\n", final_time, final_speed, final_position);
                        // printf("Train acc: %.2f, Total acc: %.2f\n", train_acceleration, acc);
                        assert(running_time >= 0);
                        assert(final_speed >= 0);
                        assert(final_position > cur_position + distance_to_run - DISTANCE_EPS);
                        assert(final_position > k * DISTANCE_STEP - DISTANCE_EPS);
                    } else {
                        // The train will stop before being able to run all the length distance_to_run

                        final_time = cur_time - cur_speed / acc;
                        final_speed = 0;
                        final_position = cur_position - powf(cur_speed, 2) / (2 * acc);

                        // printf("Uniformly decelerated linear motion (early stop)\n");
                        // printf("i: %zu, j: %zu, k: %zu\n", i, j, k);
                        // printf("Cur time: %.2f, cur speed: %.2f, cur position: %.2f\n", cur_time, cur_speed, cur_position);
                        // printf("Fin time: %.2f, fin speed: %.2f, fin position: %.2f\n", final_time, final_speed, final_position);
                        // printf("Train acc: %.2f, Total acc: %.2f\n", train_acceleration, acc);
                        assert(final_time >= cur_time);
                        assert(final_position >= cur_position);
                        assert(final_position < cur_position + distance_to_run);
                        assert(final_position < cur_position + DISTANCE_STEP);
                    }
                }

                // Updated current values
                cur_speed = final_speed;
                cur_time = final_time;
                cur_position = final_position;

                set_lookup_table_element(l, lt->speed, i, j, k, cur_speed);
                set_lookup_table_element(l, lt->time, i, j, k, cur_time);
                set_lookup_table_element(l, lt->position, i, j, k, cur_position);

                k++;
            }

            j++;
        }
    }
}

/*
 * api-method
 */
void free_lookup_tables(Lookup* lookup) {
    free_lookup_tables_for_driving_stlye(&lookup->max_acceleration);
    free_lookup_tables_for_driving_stlye(&lookup->coasting);
    free_lookup_tables_for_driving_stlye(&lookup->max_braking);
}

/*
 * api-method
 */
Lookup generate_lookup_tables(const Instance* instance) {
    Lookup l;

    float max_speed = 0;
    float max_length = 0;

    for(size_t i = 0; i < instance->num_segments; i++) {
        if(instance->segments[i].speed_limit > max_speed) { max_speed = instance->segments[i].speed_limit; }
        if(instance->segments[i].length > max_length) { max_length = instance->segments[i].length; }
    }

    size_t speeds_n = (size_t) (max_speed / SPEED_STEP + 1);
    size_t lengths_n = (size_t) (max_length / DISTANCE_STEP + 1);

    l.speeds_n = speeds_n;
    l.lengths_n = lengths_n;
    l.max_acceleration = empty_lookup_table_for_driving_style(instance->num_segments, speeds_n, lengths_n);
    l.coasting = empty_lookup_table_for_driving_style(instance->num_segments, speeds_n, lengths_n);
    l.max_braking = empty_lookup_table_for_driving_style(instance->num_segments, speeds_n, lengths_n);

    generate_lookup_table_for_acceleration(
        instance,
        &l,
        &l.max_acceleration,
        instance->train.max_acceleration
    );
    generate_lookup_table_for_acceleration(
        instance,
        &l,
        &l.coasting,
        0
    );
    generate_lookup_table_for_acceleration(
        instance,
        &l,
        &l.max_braking,
        - instance->train.max_braking
    );

    return l;
}

/*
 * api-methods
 */
float get_max_acceleration_time(const Lookup* l, size_t segment, size_t speed, size_t distance) {
    return lookup_table_element(l, l->max_acceleration.time, segment, speed, distance);
}
float get_max_acceleration_speed(const Lookup* l, size_t segment, size_t speed, size_t distance) {
    return lookup_table_element(l, l->max_acceleration.speed, segment, speed, distance);
}
float get_max_acceleration_position(const Lookup* l, size_t segment, size_t speed, size_t distance) {
    return lookup_table_element(l, l->max_acceleration.position, segment, speed, distance);
}
float get_coasting_time(const Lookup* l, size_t segment, size_t speed, size_t distance) {
    return lookup_table_element(l, l->coasting.time, segment, speed, distance);
}
float get_coasting_speed(const Lookup* l, size_t segment, size_t speed, size_t distance) {
    return lookup_table_element(l, l->coasting.speed, segment, speed, distance);
}
float get_coasting_position(const Lookup* l, size_t segment, size_t speed, size_t distance) {
    return lookup_table_element(l, l->coasting.position, segment, speed, distance);
}
float get_max_braking_time(const Lookup* l, size_t segment, size_t speed, size_t distance) {
    return lookup_table_element(l, l->max_braking.time, segment, speed, distance);
}
float get_max_braking_speed(const Lookup* l, size_t segment, size_t speed, size_t distance) {
    return lookup_table_element(l, l->max_braking.speed, segment, speed, distance);
}
float get_max_braking_position(const Lookup* l, size_t segment, size_t speed, size_t distance) {
    return lookup_table_element(l, l->max_braking.position, segment, speed, distance);
}
float get_cruising_time(size_t speed, size_t distance) {
    return (DISTANCE_STEP * distance) / (SPEED_STEP * speed);
}
float get_cruising_speed(size_t speed, size_t distance) {
    return SPEED_STEP * speed;
}
float get_cruising_position(size_t speed, size_t distance) {
    return DISTANCE_STEP * distance;
}

/*
 * api-method
 */
void print_lookup_tables(const Lookup* l, const Instance* instance) {
    for(size_t i = 0; i < instance->num_segments; i++) {
        for(size_t j = 0; j < l->speeds_n; j++) {
            for(size_t k = 0; k < l->lengths_n; k++) {
                float a_speed = get_max_acceleration_speed(l, i, j, k);
                float a_time = get_max_acceleration_time(l, i, j, k);
                float a_position = get_max_acceleration_position(l, i, j, k);

                assert((a_speed < 0) == (a_time < 0));
                assert((a_speed < 0) == (a_position < 0));

                float c_speed = get_coasting_speed(l, i, j, k);
                float c_time = get_coasting_time(l, i, j, k);
                float c_position = get_coasting_position(l, i, j, k);

                assert((c_speed < 0) == (c_time < 0));
                assert((c_speed < 0) == (c_position < 0));

                float b_speed = get_max_braking_speed(l, i, j, k);
                float b_time = get_max_braking_time(l, i, j, k);
                float b_position = get_max_braking_position(l, i, j, k);

                assert((b_speed < 0) == (b_time < 0));
                assert((b_speed < 0) == (b_position < 0));

                if(a_speed >= 0 || b_speed >= 0 || c_speed >= 0) {
                    printf("Seg: %zu, Speed: %.2f, Dist: %.2f\n", i, j * SPEED_STEP, k * DISTANCE_STEP);
                }

                if(a_speed >= 0) {
                    printf("\t[Acceleration] Final speed: %.2f, Final t: %.2f\n", a_speed, a_time);
                }

                if(c_speed >= 0) {
                    printf("\t[Coasting] Final speed: %.2f, Final t: %.2f, Final pos: %.2f\n", c_speed, c_time, c_position);
                }

                if(b_speed >= 0) {
                    printf("\t[Braking] Final speed: %.2f, Final t: %.2f, Final pos: %.2f\n", b_speed, b_time, b_position);
                }
            }
        }
    }
}
