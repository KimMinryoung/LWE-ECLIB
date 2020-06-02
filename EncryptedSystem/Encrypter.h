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

	int r_y_inverse; // scalingÀ» encrypter°¡??
	int L_inverse; // signal scaling factor, 1/L
	double r_dividedby_L; // r/L
	int s_1_inverse; // matrix(G, R) scaling factor, 1/s_1
	int s_2_inverse; // matrix(H, J) scaling factor, 1/s_2
	int U; // range size of controller output u
	int nu; // range size of controller output u
	int d; // base to split a ciphertext(ex: nu = 16 -> split a ciphertext by 2^16)
	double security_level; // lambda which represents expected value of needed operations to attack(=log2(NeededOperationNumber))

	unsigned __int64 q_dividedby_N; // q/N
	unsigned __int64 nu_; // nu + 1(used for bitwise operation that substitutes modular operation)
	unsigned __int64 q_; // q + 1
	unsigned __int64 N_; // N + 1

public:
	//----------------------------------------------------------------------------------
	//   ENCRYPTER CONSTRUCTION
	//----------------------------------------------------------------------------------
	Encrypter(int r_y_inverse, int s_1_inverse, int s_2_inverse, int U, int L_inverse, double sigma, int n);
	/**
	* decide n as large as possible while satisfying the control time constraint and return the n
	* @param double currentTimeSpan: total time cost of 1 step control operation and signal communication, with current n(and matrices sizes)
	* @param double T_s: sampling time of this controller
	* @param[out] int n: proper n to satisfy time constraint
	*/
	int Set_n(double currentTimeSpan, double T_s);
	/**
	* estimate and print security level lambda decided by Encrypter parameters(q, n, sigma)
	*/
	void PrintSecurityLevel();
	//----------------------------------------------------------------------------------
	//   DECRYPTION
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
	//   ENCRYPTION
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
	//   OPERATIONS FOR CIPHERTEXTS
	//----------------------------------------------------------------------------------
	/**
	* add two ciphertext matrices
	* @param MatrixXu c1, c2: two ciphertext matrices which were encrypted by Enc function
	*/
	MatrixXu Add(MatrixXu c1, MatrixXu c2);
	/**
	* multiply a plaintext scalar to a ciphertext matrix
	* @param unsigned __int64 scalar: a plaintext scalar to multiply
	* @param MatrixXu c: a ciphertext matrix which was encrypted by Enc function
	*/
	MatrixXu ScalarMult(unsigned __int64 scalar, MatrixXu c);
	/**
	* multiply two ciphertext matrices
	* @param MatrixXu encm: a ciphertext matrix which was encrypted by Encm function
	* @param MatrixXu split_enc: a ciphertext matrix which was encrypted by Enc function and then splited by SplitMtx function
	*/
	MatrixXu MultMxM(MatrixXu encm, MatrixXu split_enc);
	VectorXu MultMxV(MatrixXu m, RowVectorXu vec);
	/**
	* split and unfold a matrix which was encrypted by Enc function to be a form for multiplication between ciphertexts
	* @param MatrixXu c: a ciphertext matrix to split
	*/
	MatrixXu SplitMtx(MatrixXu c);
	/**
	* return ciphertext space size q
	*/
	int Getq();
};
#endif