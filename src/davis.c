//
// Created by alberto on 04/09/16.
//

#include "davis.h"
#include "eps.h"
#include <math.h>
#include <assert.h>

/*
 * implementation-method
 *
 * Calculates the resistance [kN] according to SNCF's general formula
 */
inline static float davis_resistance_kN_sncf_general(const Train* t, float speed, float lambda, float k1, float k2, bool starting) {
    assert(speed >= 0);
    assert(lambda > 0);
    assert(k1 > 0);
    assert(k2 > 0);
    assert(t->length > 0);
    assert(t->mass > 0);
    assert(t->mass_per_axle > 0);
    assert(t->mass >= t->mass_per_axle);

    float A = 1e-5 * lambda * t->mass * sqrtf(1e4 / t->mass_per_axle);
    float B = 3.6e-7 * t->mass;
    float C = 0.1296 * (k1 * SNCF_XSECTION_AREA + k2 * SNCF_PARTIAL_PERIMETER * t->length);

    return (starting ? 0 : A) + B * speed + C * powf(speed, 2);
}

/*
 * implementation-method
 *
 * Calculates the resistance [m/s^2] according to SNCF's specialised formulas
 */
inline static float davis_resistance_sncf(float speed, float a, float b, float c, bool starting) {
    return (starting ? 0 : a) + b * speed + c * powf(speed, 2);
}

/*
 * implementation-method
 *
 * Calculates the resistance [m/s^2] according to DB's Strahl formula
 */
inline static float davis_resistance_db_strahl(float speed, float m, bool starting) {
    assert(speed >= 0);
    assert(m > 0);

    return (starting ? 0 : 0.02) + (9.072e-6 + 1.296e-3 / m) * powf(speed, 2);
}

/*
 * implementation-method
 *
 * Calculates the resistance [m/s^2] according to DB's Sauthoff formula
 */
inline static float davis_resistance_db_sauthoff(const Train* t, float speed, float b, float eqarea, bool starting) {
    assert(speed >= 0);
    assert(b > 0);
    assert(eqarea > 0);
    assert(t->mass > 0);
    assert(t->num_coaches > 0);

    float mass_in_tn = t->mass / 1000; // Tonnes to kilogrammes
    float A = 1.9 + eqarea / mass_in_tn * (t->num_coaches + 2.7) * 1.08;
    float B = 3.6 * b + eqarea / mass_in_tn * (t->num_coaches + 2.7) * 0.518;
    float C = eqarea / mass_in_tn * (t->num_coaches + 2.7) * 6.2208e-2;

    return (starting ? 0 : A) + B * speed + C * powf(speed, 2);
}

/*
 * implementation-method
 *
 * Calculates the resistance (in [m/s^2]) given by Davis' equation at a certain speed.
 * It only takes in consideration the train's properties, not those of the segment.
 * @param t The train
 * @param v The current speed
 * @return  The deceleration in [m/s^2] as per Davis' equation
 */
inline static float davis_resistance(const Train* t, float speed) {
    bool starting = (speed <= MINIMUM_SPEED_FOR_DAVIS);

    switch(t->type) {
        case SNCF_TGV:
            return davis_resistance_sncf(speed, SNCF_TGV_A, SNCF_TGV_B, SNCF_TGV_C, starting);
        case SNCF_NORMAL:
            return davis_resistance_sncf(speed, SNCF_PASSENGER_AXLES_A, SNCF_PASSENGER_AXLES_B, SNCF_PASSENGER_AXLES_C, starting);
        case DB_ICE:
            return davis_resistance_db_sauthoff(t, speed, DB_SAUTHOFF_B_2AXLES, DB_EQAREA_FAST_TRAIN_FAST_LINE, starting);
        case DB_NORMAL:
            return davis_resistance_db_sauthoff(t, speed, DB_SAUTHOFF_B_2AXLES, DB_EQAREA_SLOW_TRAIN, starting);
        default:
            assert(false); return 0;
    }
}

/*
 * api-method
 */
float resistance(const Train* train, const Segment* segment, float speed) {
    assert(speed >= 0);
    assert(segment->curve >= 0);

    float davis_r = - davis_resistance(train, speed);
    float curve_r = (segment->curve < STRAIGHT_TRACK_RADIUS_EPS) ? 0 : (- CURVE_RESISTANCE_CONSTANT / segment->curve);
    float gravity_r = GRAVITATIONAL_ACCELERATION * sinf(segment->slope);

    return davis_r + curve_r + gravity_r;
}