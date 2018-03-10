#include "stdafx.h"
#include <stdexcept>
#include "kf.h"

/**
*   sys_dyn_mat - System dynamics matrix
*   output_mat - Output matrix
*   process_noise_covar - Process noise covariance
*   measurement_noise_covar - Measurement noise covariance
*   est_err_covar - Estimate error covariance
*/
kalman::kalman(
	double dt,
	const Eigen::MatrixXd& sys_dyn_mat,
	const Eigen::MatrixXd& output_mat,
	const Eigen::MatrixXd& process_noise_covar,
	const Eigen::MatrixXd& measurement_noise_covar,
	const Eigen::MatrixXd& est_err_covar)
	:A(A), C(C), Q(Q), R(R), P0(P),
	m(C.rows()), n(A.rows()), dt(dt), initialized(false),
	I(n, n), x_hat(n), x_hat_new(n) {
	I.setIdentity();
}

kalman::kalman(){}

/**
 * Initalize Kalman Filters
 */
void kalman::init(double t0,const Eigen::VectorXd& x0)
{
	x_hat = x0;
	P = P0;
	this->t0 = t0;
	initialized = true;
}

void kalman::init()
{
	x_hat.setZero();
	P = P0;
	t0 = 0;
	t = t0;
	initialized = true;
}

void kalman::update(const Eigen::VectorXd& y)
{
	if (!initialized)
		throw std::runtime_error("Filter isn't initialized yet.");
	x_hat_new = A * x_hat;
	P = A * P*A.transpose() + Q;
	K = P * C.transpose()*(C*P*C.transpose() + R).inverse();
	x_hat_new += K * (y - C * x_hat_new);
	P = (I - K * C)*P;
	x_hat = x_hat_new;

	t += dt;
}

void kalman::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {
	this->A = A;
	this->dt = dt;
	update(y);
}