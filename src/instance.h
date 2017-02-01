//
// Created by alberto on 04/09/16.
//

#ifndef TEGA_INSTANCE_H
#define TEGA_INSTANCE_H

#include <stddef.h>
#include "segment.h"
#include "train.h"

/**
 * Represents a complete instance, with a series of segments and a train.
 */
typedef struct Instance {
    const Segment*    segments;       // List of segments
    const Train       train;          // Train
    const size_t      num_segments;   // Number of segments in the instance
} Instance;

/**
 * Creates a new instance, reading from a json file.
 * @param filename  The json file name
 * @return          The newly created instance
 */
Instance read_instance(const char const* filename);

/**
 * Frees memory for an instance.
 * @param inst  The instance to be deleted
 */
void free_instance(Instance* instance);

/**
 * Prints info about the instance
 * @param instance  The instance
 */
void print_instance(const Instance* instance);

#endif //TEGA_INSTANCE_H
