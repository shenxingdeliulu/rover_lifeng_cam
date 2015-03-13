#include <stdio.h>
#include "kalman.h"
#include "matrix.h"
#include "matrix2.h"
#include "matrix_kalman.h"

kalman_filter alloc_filter(int state_dimension, int observation_dimension)
{
	kalman_filter f;
	f.timestep = 0;
	f.state_dimension = state_dimension;
	f.observation_dimension = observation_dimension;
	//F(k)
	f.state_transition = m_get(state_dimension, state_dimension);
	//B(K)
	f.control_input_model = m_get(state_dimension, observation_dimension);
	//u(k)
	f.control_input = m_get(observation_dimension, 1);
	//H(k)
	f.observation_model = m_get(observation_dimension, state_dimension);
	//Q(k)
	f.process_noise_covariance = m_get(state_dimension, state_dimension);
	//R(k)
	f.observation_noise_covariance = m_get(observation_dimension, observation_dimension);
	//z(k)
	f.observation = m_get(observation_dimension, 1);
	//x(k+1|k)
	f.predicted_state = m_get(state_dimension, 1);
	//P(k+1|k)
	f.predicted_estimate_covariance = m_get(state_dimension, state_dimension);
	//y(k)
	f.innovation = m_get(observation_dimension, 1);
	//S(k)
	f.innovation_covariance = m_get(observation_dimension, observation_dimension);
	//S(k)^-1
	f.inverse_innovation_covariance = m_get(observation_dimension, observation_dimension);
	//K(k)
	f.optimal_gain = m_get(state_dimension, state_dimension);
	//x(k+1|k+1)
	f.state_estimate = m_get(state_dimension, 1);
	//p(k+1|k+1)
	f.estimate_covariance = m_get(state_dimension, state_dimension);
	
	f.vertical_scratch = m_get(state_dimension, observation_dimension);
	f.small_square_scratch = m_get(observation_dimension, observation_dimension);
	f.big_square_scratch = m_get(state_dimension, state_dimension);
#ifdef KALMAN_DEBUG
	fprintf(stdout, "alloc_filter succeed!\n");
#endif 
	return f;
}

void free_filter(kalman_filter f)
{
	m_free(f.state_transition);
	m_free(f.control_input_model);
	m_free(f.control_input);
	m_free(f.observation_model);
	m_free(f.process_noise_covariance);
	m_free(f.observation_noise_covariance);
	m_free(f.observation);
	m_free(f.predicted_state);
	m_free(f.predicted_estimate_covariance);
	m_free(f.innovation);
	m_free(f.innovation_covariance);
	m_free(f.inverse_innovation_covariance);
	m_free(f.optimal_gain);
	m_free(f.state_estimate);
	m_free(f.estimate_covariance);

	m_free(f.vertical_scratch);
	m_free(f.small_square_scratch);
	m_free(f.big_square_scratch);

}

void predict(kalman_filter f)
{
	f.timestep++;
#ifdef KALMAN_DEBUG
	fprintf(stdout, "timestep succeed!\n");
#endif 
	static MAT *predicted_state_tmp;
	predicted_state_tmp = m_resize(predicted_state_tmp, f.state_dimension, f.observation_dimension);

	//predict the state
	//x(k+1|k) = F(k) * x(k) + B(k) * u(k) 
	m_mlt(f.control_input_model, f.control_input, predicted_state_tmp);
#ifdef KALMAN_DEBUG
		fprintf(stdout, "control_input_model is :\n");
		m_output(f.control_input_model);
		fprintf(stdout, "control_input is :\n");
		m_output(f.control_input);
#endif

	m_mlt(f.state_transition, f.state_estimate, f.predicted_state);
#ifdef KALMAN_DEBUG
	fprintf(stdout, "predicted_state_tmp is :\n");
	m_output(predicted_state_tmp);
	fprintf(stdout, "predicted_state is :\n");
	m_output(f.predicted_state);

#endif 

	m_add(f.predicted_state, predicted_state_tmp, f.predicted_state);

#ifdef KALMAN_DEBUG
	fprintf(stdout, "predicted_state is :\n");
	m_output(f.predicted_state);
#endif 

	//predict the state estimate covariance
	//P(k+1|k) = F(k) * P(k|k) F(k)^T + Q(k)
	m_mlt(f.state_transition, f.estimate_covariance, f.big_square_scratch);
	m_mlt_by_trans(f.big_square_scratch, f.state_transition, f.predicted_estimate_covariance);
	m_add(f.predicted_estimate_covariance, f.process_noise_covariance, f.predicted_estimate_covariance);
#ifdef KALMAN_DEBUG
	fprintf(stdout, "predicted_estimate_covariance is :\n");
	m_output(f.predicted_estimate_covariance);
#endif

#ifdef KALMAN_DEBUG
	fprintf(stdout, "predict succeed!\n");
#endif 
} 

void estimate(kalman_filter f)
{
	//calculate innovation
	//y(k+1)^~ = y(k+1) - H(K+1) * x(k+1|k) 
	m_mlt(f.observation_model, f.predicted_state, f.innovation);
	m_sub(f.observation, f.innovation , f.innovation);
	//calculate innovation covariance
	//S(k+1) = H(k+1|k) * P(k+1|k) * H(k+1|k)^T + R(k+1)
	m_mlt_by_trans(f.predicted_estimate_covariance, f.observation_model, f.vertical_scratch);
	m_mlt(f.observation_model, f.vertical_scratch, f.innovation_covariance);
	m_add(f.innovation_covariance, f.observation_noise_covariance, f.innovation_covariance);
	//calculate S(k+1)^-1
	m_inverse(f.innovation_covariance, f.inverse_innovation_covariance);
	//calculate the kalman gain
	//K(k+1) = P(k+1|k) * H(k+1)^T *S(k+1)
	m_mlt_by_trans(f.vertical_scratch, f.inverse_innovation_covariance, f.optimal_gain);
	//estimate tht state
	//x(k+1|k+1) = x(k+1|k) + y(k+1)^~
	m_mlt(f.optimal_gain, f.innovation, f.state_estimate);
	m_add(f.state_estimate, f.predicted_state, f.state_estimate);
	//estimate the state covariance
	//P(k+1|k+1) = P(k+1|k) - K(k+1) * H(k+1) * P(k+1|k)
	m_mlt(f.optimal_gain, f.observation_model, f.big_square_scratch);
	m_mlt(f.big_square_scratch, f.predicted_estimate_covariance, f.estimate_covariance);
	m_sub(f.predicted_estimate_covariance, f.estimate_covariance, f.estimate_covariance);
#ifdef KALMAN_DEBUG
	fprintf(stdout, "estimate succeed!\n");
#endif 
}

void update(kalman_filter f)
{
	predict(f);
	estimate(f);
}

