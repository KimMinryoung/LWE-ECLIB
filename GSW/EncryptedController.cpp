#include "stdafx.h"

#include "EncryptedController.h"

EncryptedController::EncryptedController(MatrixXu encm_FGR, MatrixXu encm_HJ, MatrixXu enc_x_init, int logq) {
	this->encm_FGR = encm_FGR;
	this->encm_HJ = encm_HJ;
	this->enc_x = enc_x_init;
	this->logq = logq;
	q_ = pow(2, logq) - 1;
	d = ceil((double)logq / nu);
}

MatrixXu EncryptedController::GetOutput(MatrixXu enc_y) { // calculate and send u to plant
	startCount();
	enc_xy = MergeByRow(enc_x, enc_y);
	MatrixXu split_enc_xy = SplitMtx(enc_xy);
	MatrixXu enc_u = MultMxM(encm_HJ, split_enc_xy);  // controller output
	return enc_u;
}
MatrixXu EncryptedController::UpdateState(MatrixXu enc_y, MatrixXu enc_u_prime) { // enc_u_prime: rearrived version of output u(has the same scale with signal y)
	MatrixXu enc_xyu = MergeByRow(enc_xy, enc_u_prime);
	MatrixXu split_enc_xyu = SplitMtx(enc_xyu);
	enc_x = MultMxM(encm_FGR, split_enc_xyu); // controller state
	calculateInterval();
	return enc_x;
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

	/*for (int i = 0; i < size_i;i++)
	for (int j = 0;j < size_j;j++)
	for (int k = 0;k < size_k;k++)
	result(i, j) = (result(i, j) + encm(i, k) * split_enc(k, j)) & q_;*/
	return result;
}
MatrixXu EncryptedController::SplitMtx(MatrixXu m) {
	MatrixXu result(m.rows() * d, m.cols());
	MatrixXu c_temp(m.rows(), m.cols());
	for (int i = 0;i < m.rows(); i++) {
		for (int j = 0;j < m.cols();j++) {
			c_temp(i, j) = m(i, j);
		}
	}

	for (int piece = 0;piece < d;piece++) {
		for (int i = 0;i < m.rows();i++) {
			for (int j = 0;j < m.cols();j++) {
				result(piece * m.rows() + i, j) = c_temp(i, j) & nu_;
				c_temp(i, j) = c_temp(i, j) >> (nu);
			}
		}
	}
	return result;
}
void EncryptedController::startCount() {
	beginTime = high_resolution_clock::now();
}
void EncryptedController::calculateInterval() {
	time_span = duration_cast<duration<double>>(high_resolution_clock::now() - beginTime);
}