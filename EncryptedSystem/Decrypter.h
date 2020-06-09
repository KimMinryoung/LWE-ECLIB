#ifndef DECRYPTER_H
#define DECRYPTER_H
#include <iostream>
using namespace std;

#include "Eigen-3.3.7/Eigen/Dense"
using Eigen::MatrixXd;
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, 1> VectorXu;
typedef Eigen::Matrix<unsigned __int64, 1, Eigen::Dynamic> RowVectorXu;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;
class Decrypter {
private:
	unsigned __int64 N; // plaintext modulus
	unsigned __int64 q; // ciphertext modulus
	int logN; // logN=log_2(N)
	int logq; // logq = log_2(q)
	unsigned __int64 N_; // N + 1
	int n;
	int n_; // ciphertext dimension n_ = n+1

	RowVectorXu secretKey;

	int r_y_inverse; // sensor resolution of signal y, 1/r_y
	int L_inverse; // signal scaling factor, 1/L
	int s_1_inverse; // matrix(G, R, J) scaling factor, 1/s_1
	int s_2_inverse; // matrix(H, J) scaling factor, 1/s_2

public:
	//----------------------------------------------------------------------------------
	//   DECRYPTER CONSTRUCTION
	//----------------------------------------------------------------------------------
	Decrypter(RowVectorXu secretKey, int r_y_inverse, int s_1_inverse, int s_2_inverse, int L_inverse, int logq, int n);
	//----------------------------------------------------------------------------------
	//   DECRYPTION
	//----------------------------------------------------------------------------------
	/**
	* decrypt a ciphertext matrix and rescale
	* @param MatrixXu c: a ciphertext matrix which was encrypted by Enc function of Encrypter
	*/
	MatrixXd Dec(MatrixXu c, unsigned __int64 scaling, bool signal);
	// decrypt a non-scaled matrix(not used for control system)
	MatrixXd Dec(MatrixXu c);
	// decrypt a control signal and rescale
	MatrixXd Dec_u(MatrixXu c);
	// decrypt a control state and rescale
};
#endif