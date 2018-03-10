#pragma once
#ifndef kf
#define kf
#include "Eigen/Dense"


class kalman {
public:

	/**
	*   sys_dyn_mat - System dynamics matrix
	*   output_mat - Output matrix
	*   process_noise_covar - Process noise covariance
	*   measurement_noise_covar - Measurement noise covariance
	*   est_err_covar - Estimate error covariance
	*/
	kalman(
		double dt,
		const Eigen::MatrixXd& sys_dyn_mat,
		const Eigen::MatrixXd& output_mat,
		const Eigen::MatrixXd& process_noise_covar,
		const Eigen::MatrixXd& measurement_noise_covar,
		const Eigen::MatrixXd& est_err_covar
	);

	/**
	* Create a blank estimator.
	*/
	kalman();

	/**
	* Initialize the filter with initial states as zero.
	*/
	void init();

	/**
	* Initialize the filter with a guess for initial states.
	*/
	void init(double t0, const Eigen::VectorXd& x0);

	/**
	* Update the estimated state based on measured values. The
	* time step is assumed to remain constant.
	*/
	void update(const Eigen::VectorXd& y);

	/**
	* Update the estimated state based on measured values,
	* using the given time step and dynamics matrix.
	*/
	void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

	/**
	* Return the current state and time.
	*/
	Eigen::VectorXd state() { return x_hat; };
	double time() { return t; };

private:

	// Matrices for computation
	Eigen::MatrixXd A, C, Q, R, P, K, P0;

	// System dimensions
	int m, n;

	// Initial and current time
	double t0, t;

	// Discrete time step
	double dt;

	// Is the filter initialized?
	bool initialized;

	// n-size identity
	Eigen::MatrixXd I;

	// Estimated states
	Eigen::VectorXd x_hat, x_hat_new;
};
#endif