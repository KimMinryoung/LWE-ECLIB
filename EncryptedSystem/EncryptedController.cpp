#include "stdafx.h"

#include "iostream"
using namespace std;
#include "Actuator.h"
#include "EncryptedController.h"

EncryptedController::EncryptedController(MatrixXu encm_FGR, MatrixXu encm_HJ, MatrixXu enc_x_init, int logq, Actuator* actuator) {
	this->encm_FGR = encm_FGR;
	this->encm_HJ = encm_HJ;
	this->enc_x = enc_x_init;
	this->logq = logq;
	q_ = pow(2, logq) - 1;
	d = ceil((double)logq / nu);
	this->actuator = actuator;
}

void EncryptedController::GetOutput(MatrixXu enc_y) { // calculate and send u to plant
	enc_xy = MergeByRow(enc_x, enc_y);
	MatrixXu split_enc_xy = SplitMtx(enc_xy);
	MatrixXu enc_u = MultMxM(encm_HJ, split_enc_xy);  // controller output
	actuator->GetControllerOutput(enc_u);
}
void EncryptedController::UpdateState(MatrixXu enc_u_prime) {
	MatrixXu enc_xyu = MergeByRow(enc_xy, enc_u_prime);
	MatrixXu split_enc_xyu = SplitMtx(enc_xyu);
	enc_x = MultMxM(encm_FGR, split_enc_xyu); // controller state
}
MatrixXu EncryptedController::MergeByRow(MatrixXu a, MatrixXu b) {
	MatrixXu result(a.rows() + b.rows(), a.cols());
	for (int i = 0;i < a.rows();i++)
		for (int j = 0;j < a.cols();j++)
			result(i, j) = a(i, j);
	for (int i = 0;i < b.rows();i++)
		for (int j = 0;j < b.cols();j++)
			result(a.rows() + i, j) = b(i, j);
	return result;
}
MatrixXu EncryptedController::MultMxM(MatrixXu encm, MatrixXu split_enc) {
	int size_i = encm.rows();
	int size_k = split_enc.rows();
	int size_j = split_enc.cols();
	MatrixXu result(size_i, size_j);
	result.noalias() = encm * split_enc;
	for (int i = 0; i < size_i;i++)
		for (int j = 0;j < size_j;j++)
			result(i, j) &= q_;
	return result;
}
MatrixXu EncryptedController::SplitMtx(MatrixXu c) {
	MatrixXu result(c.rows() * d, c.cols());
	MatrixXu c_temp(c.rows(), c.cols());
	for (int i = 0;i < c.rows(); i++) {
		for (int j = 0;j < c.cols();j++) {
			c_temp(i, j) = m(i, j);
		}
	}

	for (int piece = 0;piece < d;piece++) {
		for (int i = 0;i < c.rows();i++) {
			for (int j = 0;j < c.cols();j++) {
				result(piece * c.rows() + i, j) = c_temp(i, j) & nu_;
				c_temp(i, j) = c_temp(i, j) >> (nu);
			}
		}
	}
	return result;
}