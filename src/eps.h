//
// Created by alberto on 08/09/16.
//

#ifndef TEGA_EPS_H
#define TEGA_EPS_H

// We represent a straight line of tracks with a curve of 0, by convention
// The following constant is used to check that the curve radius is small enough to be considered 0
#define STRAIGHT_TRACK_RADIUS_EPS       1e-6

// When the acceleration is below this level, consider it 0 [m/s^2]
#define ACCELERATION_EPS 1e-6

// When the speed is below this level, consider it 0 [m/s]
#define SPEED_EPS 1e-6

// When a distance is below this number, consider it 0 [m]
#define DISTANCE_EPS 1e-6

// When a segment length is below this level, consider it 0 [m]
#define SEGMENT_LENGTH_EPS 1e-6

#endif //TEGA_EPS_H
