#ifndef KALMAN_H
#define KALMAN_H

#include "matrix.h"

typedef struct 
{
	int timestep;
	int state_dimension, observation_dimension;

	MAT * state_transition;
	MAT * control_input_model;
	MAT * control_input;
	MAT * observation_model;
	MAT * process_noise_covariance;
	MAT * observation_noise_covariance;

	MAT * observation;

	MAT * predicted_state;
	MAT * predicted_estimate_covariance;
	MAT * innovation;
	MAT * innovation_covariance;
	MAT * inverse_innovation_covariance;
	MAT * optimal_gain;
	MAT * state_estimate;
	MAT * estimate_covariance;

	MAT * vertical_scratch;
	MAT * small_square_scratch;
	MAT * big_square_scratch;
} kalman_filter;

kalman_filter alloc_filter(int state_dimension, int observation_dimension);

void free_filter(kalman_filter f);

void predict(kalman_filter f);

void estimate(kalman_filter f);

void update(kalman_filter f);
#endif