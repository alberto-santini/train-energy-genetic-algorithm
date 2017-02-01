//
// Created by alberto on 05/09/16.
//

#include "segment.h"
#include <stdio.h>
#include <inttypes.h>

/*
 * api-method
 */
void print_segment(const Segment* segment) {
    printf("Segment #%" PRIuFAST32 " [%.2f m, %.2f m]", segment->id, segment->start_x, segment->end_x);
    if(segment->is_station) {
        printf(" (station, stop time: %.2f s)\n", segment->stop_time);
    } else {
        printf("\n");
    }

    if(segment->has_arrival_time) {
        printf("\tArrival time: %.2f s\n", segment->arrival_time);
    }

    printf("\tLength: %.2f m\n", segment->length);
    printf("\tSlope: %.2f rad\n", segment->slope);
    printf("\tCurve radius: %.2f m\n", segment->curve);
    printf("\tSpeed limit: %.2f m/s\n", segment->speed_limit);
}