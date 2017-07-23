//
// Created by alberto on 04/09/16.
//

#include "instance.h"
#include <stdlib.h>
#include <jansson.h>
#include <memory.h>
#include <assert.h>
#include <inttypes.h>

/*
 * api-method
 */
Instance read_instance(const char *const filename) {
    FILE* fd;
    fd = fopen(filename, "r");

    if(fd == NULL) {
        fprintf(stderr, "Cannot read input file: %s\n", filename);
        exit(EXIT_FAILURE);
    }

    size_t file_sz;

    // Get the input file size
    fseek(fd, 0, SEEK_END);
    file_sz = ftell(fd);
    rewind(fd);

    // Allocates enough space for the contents, plus 1 char for \0
    char* file_contents;
    file_contents = malloc((file_sz + 1) * sizeof(*file_contents));

    if(file_contents == NULL) {
        fprintf(stderr, "Could not allocate memory to read input file: %s\n", filename);
        exit(EXIT_FAILURE);
    }

    // Read the whole file into a string
    size_t chars_read;
    chars_read = fread(file_contents, sizeof(*file_contents), file_sz, fd);
    assert(chars_read == file_sz);

    // Add string null-terminator
    file_contents[file_sz] = '\0';

    fclose(fd);

    json_t* root;
    json_error_t error;

    // Parse the json content
    root = json_loads(file_contents, 0, &error);

    free(file_contents);

    if(root == NULL) {
        fprintf(stderr, "Error reading json in %s, line %d: %s\n", filename, error.line, error.text);
        exit(EXIT_FAILURE);
    }

    assert(json_is_object(root));

    json_t* train_data;
    train_data = json_object_get(root, "train");
    assert(json_is_object(train_data));

    json_t* train_type_data;
    json_t* num_coaches_data;
    json_t* mass_data;
    json_t* mass_per_axle_data;
    json_t* max_acceleration_data;
    json_t* max_braking_data;
    json_t* length_data;
    const char* train_type_str;
    TrainType train_type;

    train_type_data = json_object_get(train_data, "type");
    assert(json_is_string(train_type_data));

    train_type_str = json_string_value(train_type_data);

    if(strcmp(train_type_str, "SNCF TVG") == 0) {
        train_type = SNCF_TGV;
    } else {
        fprintf(stderr, "Error reading train type from %s: type %s is not currently supported\n", filename, train_type_str);
        json_decref(root);
        exit(EXIT_FAILURE);
    }

    num_coaches_data = json_object_get(train_data, "num_coaches");
    assert(json_is_number(num_coaches_data));

    mass_data = json_object_get(train_data, "mass");
    assert(json_is_real(mass_data));

    mass_per_axle_data = json_object_get(train_data, "mass_per_axle");
    assert(json_is_real(mass_per_axle_data));

    max_acceleration_data = json_object_get(train_data, "max_acceleration");
    assert(json_is_real(max_acceleration_data));

    max_braking_data = json_object_get(train_data, "max_braking");
    assert(json_is_real(max_braking_data));

    length_data = json_object_get(train_data, "length");
    assert(json_is_real(length_data));

    Train train = {
        .type = train_type,
        .num_coaches = (uint_fast32_t) json_integer_value(num_coaches_data),
        .mass = (float) json_real_value(mass_data),
        .mass_per_axle = (float) json_real_value(mass_per_axle_data),
        .max_acceleration = (float) json_real_value(max_acceleration_data),
        .max_braking = (float) json_real_value(max_braking_data),
        .length = (float) json_real_value(length_data)
    };

    assert(train.mass > 0);
    assert(train.mass_per_axle > 0);
    assert(train.mass_per_axle < train.mass);
    assert(train.max_acceleration > 0);
    assert(train.max_braking > 0);
    assert(train.length > 0);

    json_t* num_segments_data;
    uint_fast32_t num_segments;

    num_segments_data = json_object_get(root, "num_segments");
    assert(json_is_integer(num_segments_data));

    num_segments = (uint_fast32_t) json_integer_value(num_segments_data);

    json_t* segments_ary;

    segments_ary = json_object_get(root, "segments");
    assert(json_is_array(segments_ary));

    size_t segments_ary_sz = json_array_size(segments_ary);

    if(segments_ary_sz != (size_t) num_segments) {
        fprintf(stderr, "Error reading segments from %s: ``num_segments'' is %" PRIuFAST32 ", but the ``segments'' array contains %zu entries\n", filename, num_segments, segments_ary_sz);
        json_decref(root);
        exit(EXIT_FAILURE);
    }

    Segment* segments;
    segments = malloc(segments_ary_sz * sizeof(*segments));

    if(segments == NULL) {
        fprintf(stderr, "Could not allocate memory for the segments\n");
        json_decref(root);
        exit(EXIT_FAILURE);
    }

    float current_x = 0;
    for(size_t i = 0; i < segments_ary_sz; i++) {
        json_t* segment_data;
        segment_data = json_array_get(segments_ary, i);
        assert(json_is_object(segment_data));

        json_t* id_data;
        json_t* station_data;
        json_t* arrival_time_data;
        uint_fast32_t id;
        bool station;

        id_data = json_object_get(segment_data, "id");
        assert(json_is_integer(id_data));

        station_data = json_object_get(segment_data, "station");
        assert(json_is_boolean(station_data));

        arrival_time_data = json_object_get(segment_data, "arrival_time");

        id = (uint_fast32_t) json_number_value(id_data);
        station = (bool) json_boolean_value(station_data);

        if(station) {
            assert(json_is_number(arrival_time_data));

            json_t* stop_time_data;
            stop_time_data = json_object_get(segment_data, "stop_time");
            assert(json_is_number(stop_time_data));

            segments[i] = (Segment) {
                .id = id,
                .arrival_time = (float) json_real_value(arrival_time_data),
                .stop_time = (float) json_real_value(stop_time_data),
                .length = 0,
                .slope = 0,
                .curve = 0,
                .speed_limit = 0,
                .start_x = current_x,
                .end_x = current_x,
                .is_station = true,
                .has_arrival_time = true
            };

            assert(segments[i].arrival_time >= 0);
            assert(segments[i].stop_time >= 0);
        } else {
            bool has_arrival_time = (arrival_time_data != NULL);

            json_t* length_data;
            json_t* slope_data;
            json_t* curve_data;
            json_t* speed_limit_data;

            length_data = json_object_get(segment_data, "length");
            slope_data = json_object_get(segment_data, "slope");
            curve_data = json_object_get(segment_data, "curve");
            speed_limit_data = json_object_get(segment_data, "speed_limit");

            float length = (float) json_real_value(length_data);

            segments[i] = (Segment) {
                .id = id,
                .arrival_time = (has_arrival_time ? (float) json_real_value(arrival_time_data) : -1),
                .stop_time = -1,
                .length = length,
                .slope = (float) json_real_value(slope_data),
                .curve = (float) json_real_value(curve_data),
                .speed_limit = (float) json_real_value(speed_limit_data),
                .start_x = current_x,
                .end_x = current_x + length,
                .is_station = false,
                .has_arrival_time = has_arrival_time
            };

            assert(segments[i].length > 0);
            assert(segments[i].curve >= 0);
            assert(segments[i].speed_limit > 0);
            assert(!segments[i].has_arrival_time || segments[i].arrival_time >= 0);

            current_x += length;
        }
    }

    json_decref(root);

    return (Instance) {.segments = segments, .train = train, .num_segments = num_segments};
}

/*
 * api-method
 */
void free_instance(Instance* instance) {
    free((Segment*) instance->segments);
    instance->segments = NULL;
}

/*
 * api-method
 */
void print_instance(const Instance* instance) {
    printf("=== INSTANCE ===\n");
    print_train(&instance->train);
    for(size_t i = 0; i < instance->num_segments; i++) {
        print_segment(instance->segments + i);
    }
}