//
// Created by alberto on 04/09/16.
//

#ifndef TEGA_SEGMENT_H
#define TEGA_SEGMENT_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Representation of a uniform segment of track.
 */
typedef struct Segment {
    uint_fast32_t   id;                 // Unique progressive id
    float           arrival_time;       // (Eventual) time at which the train should arrive at the end of the segment
    float           stop_time;          // (Eventual) stop time at the station
    float           length;             // Length in metres
    float           slope;              // Slope in radians
    float           curve;              // Curve ray in metres
    float           speed_limit;        // Hard speed limit in m/s
    float           start_x;            // Coordinate of the beginning of the segment
    float           end_x;              // Coordinate of the end of the segment
    bool            is_station;         // If this is true, the segment will have length 0 and the train should stop here
    bool            has_arrival_time;   // The train should arrive at the end of this segment at a predetermined time
} Segment;

/**
 * Print information on a segment.
 * @param segment   The segment
 */
void print_segment(const Segment* segment);

#endif //TEGA_SEGMENT_H
