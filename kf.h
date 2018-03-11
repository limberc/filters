#pragma once
#ifndef kf
#define kf
#include "Eigen/Dense"


class kalman {
public:

	// state vector
	Eigen::VectorXd state_vec;

	// state covariance matrix
	Eigen::MatrixXd state_covar_mat;

	// state transistion matrix
	Eigen::MatrixXd state_trans_mat;

	// process covariance matrix
	Eigen::MatrixXd process_covar_mat;

	// measurement matrix
	Eigen::MatrixXd measurement_mat;

	// measurement covariance matrix
	Eigen::MatrixXd measurement_covar_mat;

	/**
	* Constructor
	*/
	kalman();

	/**
	* Destructor
	*/
	virtual ~kalman();

	/**
	* Init Initializes Kalman filter
	* @param x_in Initial state
	* @param P_in Initial state covariance
	* @param F_in Transition matrix
	* @param H_in Measurement matrix
	* @param R_in Measurement covariance matrix
	* @param Q_in Process covariance matrix
	*/
	void init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
		Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

	/**
	* Prediction Predicts the state and the state covariance
	* using the process model
	* @param delta_T Time between k and k+1 in s
	*/
	void predict();

	/**
	* Updates the state by using standard Kalman Filter equations
	* @param z The measurement at k+1
	*/
	void update(const Eigen::VectorXd &z);

	/**
	* Updates the state by using Extended Kalman Filter equations
	* @param z The measurement at k+1
	*/
	void update_ekf(const Eigen::VectorXd &z);

	/**
	* Universal update Kalman Filter step. Equations from the lectures
	* @param y The error
	*/
	void univ_update_kf(const Eigen::VectorXd &y);

};
#endif