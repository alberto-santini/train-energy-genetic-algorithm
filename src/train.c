//
// Created by alberto on 05/09/16.
//

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <inttypes.h>
#include "train.h"

/*
 * api-method
 */
void print_train(const Train* train) {
    switch(train->type) {
        case SNCF_TGV:
            printf("Train of type SNCF (TGV)\n");
            break;
        case SNCF_NORMAL:
            printf("Train of type SNCF (normal)\n");
            break;
        case DB_ICE:
            printf("Train of type DB (ICE\n");
            break;
        case DB_NORMAL:
            printf("Train of type DB (normal)\n");
            break;
        default:
            assert(false);
            break;
    }

    printf("\tNumber of coaches: %" PRIuFAST32 ", total length: %.2f m\n", train->num_coaches, train->length);
    printf("\tMass: %.2f kg (%.2f kg per axle)\n", train->mass, train->mass_per_axle);
    printf("\tMax acceleration: %.2f m/s^2, max braking: %.2f m/s^2\n", train->max_acceleration, train->max_braking);
}