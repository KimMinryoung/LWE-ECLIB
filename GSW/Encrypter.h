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
typedef Eigen::Matrix<double, 2, 1> Vector2d;
class Encrypter {
private:
	RowVectorXu secretKey;
	unsigned __int64 s; // secret key range
	unsigned __int64 q; // ciphertext modulus
	unsigned __int64 N; // plaintext modulus
	double sigma; // stddev of noise
	int n;
	int n_;

	int r_y_inverse;
	int L_inverse;
	double r_dividedby_L;
	int s_1_inverse;
	int s_2_inverse;
	int U;
	int nu;
	int d;
	int logN;
	int logq;
	double security_level;

	unsigned __int64 q_dividedby_N;
	unsigned __int64 nu_;
	unsigned __int64 q_;
	unsigned __int64 N_;

public:
	Encrypter(int r_y_inverse, int s_1_inverse, int s_2_inverse, int U, int L_inverse, double sigma, int n);
	int Set_n(double currentTimeSpan, double T_s);
	void PrintSecurityLevel();
	MatrixXd Dec(MatrixXu c, int scaling, bool signal);
	// Decrypt non-scaled matrix(not used for control system)
	MatrixXd Dec(MatrixXu c);
	// Decrypt control signal and auto-rescale
	MatrixXd Dec_u(MatrixXu c);
	// Decrypt control state and auto-rescale
	MatrixXd Dec_state(MatrixXu x);
	MatrixXu Enc(MatrixXd m, bool signal);
	MatrixXu Encm(MatrixXd m);
	MatrixXu SplitMtx(MatrixXu m);
	MatrixXu MultMxM(MatrixXu encm, MatrixXu split_enc);
	VectorXu MultMxV(MatrixXu m, RowVectorXu vec);
	MatrixXu ScalarMult(unsigned __int64 scalar, MatrixXu c);
	MatrixXu Add(MatrixXu c1, MatrixXu c2);
	int Getq();
};
#endif