#pragma once
class Actuator;
#include <Eigen/Dense>
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;

class EncryptedController {

private:
	Actuator* actuator;

	// >>controller<<
	// x+ = FGR * xyu
	// u = HJ * xy
	MatrixXu encm_FGR; // encrypted matrix FGR
	MatrixXu encm_HJ; // encrypted matrix HJ
	MatrixXu enc_xy;
	int logq = 48; // default!!??!?!?!?
	unsigned __int64 q_ = pow(2, logq) - 1;
	const int nu = 16;
	const unsigned __int64 nu_ = pow(2, nu) - 1;
	int d = logq / nu;

public:
	EncryptedController(MatrixXu encm_FGR, MatrixXu encm_HJ, MatrixXu enc_x_init, int logq, Actuator* actuator);
	void GetOutput(MatrixXu enc_y);
	void UpdateState(MatrixXu enc_u_prime);
	MatrixXu MergeByRow(MatrixXu a, MatrixXu b);
	MatrixXu MultMxM(MatrixXu encm, MatrixXu split_enc);
	MatrixXu SplitMtx(MatrixXu m);
	MatrixXu enc_x;
};