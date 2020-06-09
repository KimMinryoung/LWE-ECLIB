#include "stdafx.h"
#include "Decrypter.h"

Decrypter::Decrypter(RowVectorXu secretKey, int r_y_inverse, int s_1_inverse, int s_2_inverse, int L_inverse, int logq, int n) {
	if (n == -1)
		this->n = 100; // temporal ciphertext dimension for speed measuring
	else
		this->n = n;
	this->n_ = this->n + 1;

	this->secretKey = secretKey;

	this->r_y_inverse = r_y_inverse; // resolution of controller input signal y
	this->L_inverse = L_inverse; // scaling factor for signal y
	this->s_1_inverse = s_1_inverse; // scaling factor for matrix G
	this->s_2_inverse = s_2_inverse; // scaling factor for matrix H and J

	this->logq = logq;
	this->logN = logq;
	N = (unsigned __int64)pow(2, logN); // plaintext modulus
	q = (unsigned __int64)pow(2, logq); // ciphertext modulus
	N_ = N - 1;
}
MatrixXd Decrypter::Dec(MatrixXu c, unsigned __int64 scaling, bool signal) {
	int l = c.rows() / n_;
	MatrixXd y(l, c.cols());

	for (int j = 0;j < c.cols();j++) {
		for (int i = 0;i < l;i++) {
			unsigned __int64 temp = c(i*n_, j);
			for (int k = 0;k < n;k++) {
				temp += secretKey(k) * c(i*n_ + k + 1, j);
			}
			unsigned __int64 m = temp & N_;
			signed __int64 real_m = m - (m >= N / 2) * N;
			if (signal)
				y(i, j) = (double)real_m / (double)L_inverse / (double)scaling;
			else
				y(i, j) = (double)real_m / (double)r_y_inverse / (double)scaling;
		}
	}
	return y;
}
// Decrypt a non-scaled matrix(not used for control system)
MatrixXd Decrypter::Dec(MatrixXu c) {
	return Dec(c, 1, false);
}
// Decrypt controller output and auto-rescale
MatrixXd Decrypter::Dec_u(MatrixXu c) {
	return Dec(c, s_1_inverse*s_2_inverse, true);
}