#include "stdafx.h"
#include <stdexcept>
#include "kf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

kalman::kalman() {}

kalman::~kalman() {}

void kalman::init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
	MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	state_vec = x_in; // object state
	state_covar_mat = P_in; // object covariance matrix
	state_trans_mat = F_in; // state transition matrix
	measurement_mat = H_in; // measurement matrix
	measurement_covar_mat = R_in; // measurement covariance matrix
	process_covar_mat = Q_in; // process covariance matrix
}

void kalman::predict() {
	/**
	TODO:
	* predict the state
	*/
	state_vec = state_trans_mat * state_vec; // There is no external motion, so, we do not have to add "+u"
	MatrixXd Ft = state_trans_mat.transpose();
	state_covar_mat = state_trans_mat * state_covar_mat * Ft + process_covar_mat;
}

void kalman::update(const VectorXd &z) {
	/**
	TODO:
	* update the state by using Kalman Filter equations
	* Kalman filter update step. 
	*/
	VectorXd y = z - measurement_mat * state_vec; // error calculation
	univ_update_kf(y);
}

void kalman::update_ekf(const VectorXd &z) {
	/**
	TODO:
	* update the state by using Extended Kalman Filter equations
	*/
	double rho = sqrt(state_vec(0)*state_vec(0) + state_vec(1)*state_vec(1));
	double theta = atan(state_vec(1) / state_vec(0));
	double rho_dot = (state_vec(0)*state_vec(2) + state_vec(1)*state_vec(3)) / rho;
	VectorXd h = VectorXd(3); // h(x_)
	h << rho, theta, rho_dot;

	VectorXd y = z - h;
	// Calculations are essentially the same to the Update function
	univ_update_kf(y);
}

// Universal update Kalman Filter step. Equations from the lectures
void kalman::univ_update_kf(const VectorXd &y) {
	MatrixXd Ht = measurement_mat.transpose();
	MatrixXd S = measurement_mat * state_covar_mat * Ht + measurement_covar_mat;
	MatrixXd Si = S.inverse();
	MatrixXd K = state_covar_mat * Ht * Si;
	// New state
	state_vec = state_vec + (K * y);
	int x_size = state_vec.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	state_covar_mat = (I - K * measurement_mat) * state_covar_mat;
}