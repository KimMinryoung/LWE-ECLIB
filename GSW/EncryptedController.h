#pragma once
class Actuator;
#include <Eigen/Dense>
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;
#include <chrono>
using namespace std::chrono;

class EncryptedController {

private:
	Actuator* actuator;
	MatrixXu encm_FGR;
	MatrixXu encm_HJ;
	MatrixXu enc_xy;
	int logq = 48;
	unsigned __int64 q_ = pow(2, logq) - 1;
	const int nu = 16;
	const unsigned __int64 nu_ = pow(2, nu) - 1;
	int d = logq / nu;
	high_resolution_clock::time_point beginTime;
	void startCount();
	void calculateInterval();

public:
	duration<double> time_span;
	MatrixXu enc_x;
	EncryptedController(MatrixXu encm_FGR, MatrixXu encm_HJ, MatrixXu enc_x_init, int logq, Actuator* actuator);
	MatrixXu GetOutput(MatrixXu enc_y);
	void UpdateState(MatrixXu enc_u_prime);
	MatrixXu MergeByRow(MatrixXu a, MatrixXu b);
	MatrixXu MultMxM(MatrixXu encm, MatrixXu split_enc);
	MatrixXu SplitMtx(MatrixXu m);
};