#ifndef ENCRYPTER_H
#define ENCRYPTER_H
#include <iostream>
using namespace std;
#include <random>
#include <time.h>

#include <Eigen/Dense>
using Eigen::MatrixXd;
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, 1> VectorXu;
typedef Eigen::Matrix<unsigned __int64, 1, Eigen::Dynamic> RowVectorXu;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;
class Encrypter {
private:
	unsigned __int64 N; // plaintext modulus
	unsigned __int64 q; // ciphertext modulus
	int logN; // logN=log_2(N)
	int logq; // logq = log_2(q)
	int n;
	int n_; // ciphertext dimension n_ = n+1
	double sigma; // standard deviation for Gaussian distribution of noise

	RowVectorXu secretKey;
	unsigned __int64 s; // secret key range !!!!!!!

	int r_y_inverse; // scaling¿ª encrypter∞°??
	int L_inverse;
	double r_dividedby_L;
	int s_1_inverse;
	int s_2_inverse;
	int U;
	int nu;
	int d;
	double security_level;

	unsigned __int64 q_dividedby_N;
	unsigned __int64 nu_;
	unsigned __int64 q_;
	unsigned __int64 N_;

public:
	Encrypter(int r_y_inverse, int s_1_inverse, int s_2_inverse, int U, int L_inverse, double sigma, int n);
	int Set_n(double currentTimeSpan, double T_s);
	void PrintSecurityLevel();
	//----------------------------------------------------------------------------------
	//   Decryption
	//----------------------------------------------------------------------------------
	/**
	* decrypt a ciphertext matrix and rescale
	* @param MatrixXu c: a ciphertext matrix which was encrypted by Enc function
	*/
	MatrixXd Dec(MatrixXu c, unsigned __int64 scaling, bool signal);
	// decrypt a non-scaled matrix(not used for control system)
	MatrixXd Dec(MatrixXu c);
	// decrypt a control signal and rescale
	MatrixXd Dec_u(MatrixXu c);
	// decrypt a control state and rescale
	MatrixXd Dec_state(MatrixXu x);
	//----------------------------------------------------------------------------------
	//   Encryption
	//----------------------------------------------------------------------------------
	/**
	* encrypt a matrix
	* @param MatrixXd m: plaintext matrix to encrypt
	*/
	MatrixXu Enc(MatrixXd m, bool signal);
	/**
	* encrypt a matrix to be a form for multiplication between ciphertexts
	* @param MatrixXd m: plaintext matrix to encrypt
	*/
	MatrixXu Encm(MatrixXd m);
	//----------------------------------------------------------------------------------
	//   Operations for ciphertext
	//----------------------------------------------------------------------------------
	/**
	* split and unfold a matrix which was encrypted by Enc function to be a form for multiplication between ciphertexts
	* @param MatrixXu c: ciphertext matrix to split
	*/
	MatrixXu SplitMtx(MatrixXu c);
	/**
	* multiply two ciphertext matrices
	* @param MatrixXu encm: a ciphertext matrix which was encrypted by Encm function
	* @param MatrixXu split_enc: a ciphertext matrix which was encrypted by Enc function and then splited by SplitMtx function
	*/
	MatrixXu MultMxM(MatrixXu encm, MatrixXu split_enc);
	VectorXu MultMxV(MatrixXu m, RowVectorXu vec);
	/**
	* multiply a plaintext scalar to a ciphertext matrix
	* @param unsigned __int64 scalar: a plaintext scalar to multiply
	* @param MatrixXu c: a ciphertext matrix which was encrypted by Enc function
	*/
	MatrixXu ScalarMult(unsigned __int64 scalar, MatrixXu c);
	/**
	* add two ciphertext matrices
	* @param MatrixXu c1, c2: two ciphertext matrices which were encrypted by Enc function
	*/
	MatrixXu Add(MatrixXu c1, MatrixXu c2);
	int Getq();
};
#endif