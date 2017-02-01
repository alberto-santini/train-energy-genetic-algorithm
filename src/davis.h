//
// Created by alberto on 04/09/16.
//

#ifndef TEGA_DAVIS_H
#define TEGA_DAVIS_H

#include "segment.h"
#include "train.h"

/*
 * We assume below that Davis' formula
 * R = A + B * speed + C * speed^2
 * Gives the ``specific resistance'' in [N/Kg], which is to say, it gives the
 * deceleration in [m/s^2].
 */

/*
 * SNCF calculates the coefficients of Davis' equation according to the formulas:
 * A = 1e-5 * lambda * mass * sqrt(1e5 / mass_per_axle)
 * B = 3.6e-7 * mass
 * C = 0.1296 * (k1 * xsection + k2 * partial_perimeter * length)
 *
 * lambda, k1, and k2 are parameters (see below)
 * xsection and partial_perimeter are estimates and are assumed to be fixed for any train
 * mass, mass_per_axle, and length are taken from input data
 *
 * mass: [kg]
 * mass_per_axle: [kg]
 * length: [m]
 *
 * However, SNCF's ``general'' formula, gives the resistance in [kN], not the ``specific
 * resistance'' in [m/s^2].
 */

#define SNCF_LAMBDA_TGV                 0.9
#define SNCF_LAMBDA_PASSENGER           1.2
#define SNCF_LAMBDA_FREIGHT             1.5

#define SNCF_K1_TGV                     9e-4
#define SNCF_K1_OTHER                   2e-4

#define SNCF_K2_TGV                     20e-6
#define SNCF_K2_OTHER                   30e-6

#define SNCF_XSECTION_AREA              10
#define SNCF_PARTIAL_PERIMETER          10

/*
 * SNCF also uses specialised formulas that give the specific resistance:
 *
 * Normal passenger trains (on bogies):
 * A = 1.5e-2
 * B = 0
 * C = 3.6e-5
 *
 * Normal passenger trains (2- or 3-axled):
 * A = 1.5e-2
 * B = 0
 * C = 5.4e-5
 *
 * TGV:
 * A = 6e-2
 * B = 2.86e-3
 * C = 1.71e-4
 */

#define SNCF_PASSENGER_BOGIES_A         1.5e-2
#define SNCF_PASSENGER_BOGIES_B         0
#define SNCF_PASSENGER_BOGIES_C         3.6e-5

#define SNCF_PASSENGER_AXLES_A          1.5e-2
#define SNCF_PASSENGER_AXLES_B          0
#define SNCF_PASSENGER_AXLES_C          5.4e-5

#define SNCF_TGV_A                      6e-2
#define SNCF_TGV_B                      2.86e-3
#define SNCF_TGV_C                      1.71e-4

/*
 * DB calculates the coefficients of Davis' equation according to the formulas:
 *
 * For Freight trains: Strahl's formula
 * A = 0.02
 * B = 0
 * C = 9.072e-6 + 1.296e-3 / m
 *
 * m is a parameter (see below)
 *
 * For passenger trains: Sauthoff's formula
 * A = 1.9 + equivalent_area / mass * (num_coaches + 2.7) * 1.08
 * B = 3.6 * b + equivalent_area / mass * (num_coaches + 2.7) * 0.5184
 * C = equivalent_area / mass * (num_coaches + 2.7) * 6.2208e-2
 *
 * b is a parameter (see below)
 * equivalent_area is an esimate and is assumed to be fixed, given a certain train type
 * mass and num_coaches are taken from input data
 *
 * mass: [tonnes]
 * num_coaches: [dimensionless]
 */
#define DB_STRAHL_M_4AXLES              40
#define DB_STRAHL_M_3AXLES              30
#define DB_STRAHL_M_LOADED_FREIGHT      25
#define DB_STRAHL_M_EMPTY_FREIGHT       10

#define DB_SAUTHOFF_B_4AXLES            2.5e-3
#define DB_SAUTHOFF_B_3AXLES            0.004
#define DB_SAUTHOFF_B_2AXLES            0.007

#define DB_EQAREA_FAST_TRAIN_FAST_LINE  1.45
#define DB_EQAREA_FAST_TRAIN_SLOW_LINE  1.55
#define DB_EQAREA_SLOW_TRAIN            1.15

/*
 * Gravitational acceleration [m/s]
 */
#define GRAVITATIONAL_ACCELERATION      9.81

/*
 * Minimum speed to apply Davis equation. This equation, in fact, only
 * applies to a train in motion, not to a train starting to move. A train
 * is usually started "one car at a time" by exploiting the slack between
 * the cars. In order to keep things simple, we will assume that below
 * this speed [m/s], the constant part of Davis equation is 0.
 */
#define MINIMUM_SPEED_FOR_DAVIS         1.0

/*
 * Constant used to calculate the resistance due to the tracks' curve
 */
#define CURVE_RESISTANCE_CONSTANT       8

/**
 * Calculates the total resistance opposing or favouring a train's motion.
 * It takes into account:
 * 1) Davis resistance (always opposes the motion)
 * 2) Track curvature resistance (always opposes the motion)
 * 3) Gravity resistance (can favour the motion, if going downhill)
 *
 * @param t         The train
 * @param s         The segment where it's moving
 * @param speed     The current train speed [m/s]
 * @return          The resistance (acceleration if >= 0, or deceleration if < 0) [m/s^2]
 */
float resistance(const Train* train, const Segment* segment, float speed);

#endif //TEGA_DAVIS_H
