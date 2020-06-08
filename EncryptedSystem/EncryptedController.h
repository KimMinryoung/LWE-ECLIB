#pragma once
class Actuator;
#include "Eigen-3.3.7/Eigen/Dense"
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;

class EncryptedController {

private:
	Actuator* actuator; // connected actuator

	// >>controller<<
	// x+ = FGR * xyu
	// u = HJ * xy
	MatrixXu encm_FGR; // encrypted matrix FGR
	MatrixXu encm_HJ; // encrypted matrix HJ
	MatrixXu enc_x; // saved controller state x
	MatrixXu enc_xy; // saved merged matrix of controller state x and plant output y
	int logq = 48; // default!!??!?!?!?
	unsigned __int64 q_ = (unsigned __int64)(pow(2, logq)) - 1;
	const int nu = 16; // base to split a ciphertext(ex: nu = 16 -> split a ciphertext by 2^16)
	const unsigned __int64 nu_ = (unsigned __int64)(pow(2, nu)) - 1;
	int d = logq / nu; // number of pieces to split

public:
	EncryptedController(MatrixXu encm_FGR, MatrixXu encm_HJ, MatrixXu enc_x_init, int logq, Actuator* actuator);
	//----------------------------------------------------------------------------------
	//   CONTROL OPERATION
	//----------------------------------------------------------------------------------
	/**
	* calculate controller output u = HJ * xy and send u to the actuator
	* @param MatrixXu enc_y: an encrypted plant output signal from sensor
	*/
	void GetOutput(MatrixXu enc_y);
	/**
	* calculate and update controller state x+ = FGH * xyu
	* @param MatrixXu enc_u_prime: re-encrypted controller output u which controller got back from the actuator(has the same scale with signal y, which is 1/L)
	*/
	void UpdateState(MatrixXu enc_u_prime);
	//----------------------------------------------------------------------------------
	//   TOOL FUNCTIONS FOR CONTROL OPERATION
	//----------------------------------------------------------------------------------
	/**
	* merge two ciphertext matrices by row(ex: when a = {1;2} and b = {3}, return ab = {1;2;3})
	* @param MatrixXu a, b: ciphertext matrix
	*/
	MatrixXu MergeByRow(MatrixXu a, MatrixXu b);
	/**
	* multiply two ciphertext matrices(controller matrix and signal)
	* @param MatrixXu encm: a ciphertext matrix which was encrypted by Encm function
	* @param MatrixXu split_enc: a ciphertext matrix which was encrypted by Enc function and then splited by SplitMtx function
	*/
	MatrixXu MultMxM(MatrixXu encm, MatrixXu split_enc);
	/**
	* split and unfold ciphertext signal to be a form for multiplication between ciphertexts
	* @param MatrixXu c: a ciphertext matrix to split
	*/
	MatrixXu SplitMtx(MatrixXu c);
};