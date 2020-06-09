//#pragma _D_SCL_SECURE_NO_WARNINGS
#include "stdafx.h"
//#include <mkl.h>
//#define EIGEN_USE_MKL_ALL
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <windows.h>
#include "Encrypter.h"
#include "Decrypter.h"
#include "SystemBuilder.h"

#include <chrono>

//#define EIGEN_VECTORIZE_SSE4_2
#include "Eigen-3.3.7/Eigen/Dense"
using Eigen::MatrixXd;
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, 1> VectorXu;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;

using namespace std;

class ConvertedController {
private:
	MatrixXd FGR;
	MatrixXd HJ;
	MatrixXd x;
	MatrixXd u;
	MatrixXd xy;

public:
	ConvertedController(MatrixXd FGR, MatrixXd HJ, MatrixXd x_init) {
		this->FGR = FGR;
		this->HJ = HJ;
		this->x = x_init;
	}

	MatrixXd GetOutput(MatrixXd y) {
		xy = MergeByRow(x, y);
		u = HJ * xy;
		return u;
	}
	void UpdateState(MatrixXd u_prime) {
		MatrixXd xyu = MergeByRow(xy, u_prime);
		x = FGR * xyu;
	}
	MatrixXd MergeByRow(MatrixXd a, MatrixXd b) {
		MatrixXd result(a.rows() + b.rows(), a.cols());
		for (int i = 0;i < a.rows();i++)
			for (int j = 0;j < a.cols();j++)
				result(i, j) = a(i, j);
		for (int i = 0;i < b.rows();i++)
			for (int j = 0;j < b.cols();j++)
				result(a.rows() + i, j) = b(i, j);
		return result;
	}
};
class QuantizedController {
private:
	MatrixXd FGR;
	MatrixXd HJ;
	MatrixXd x;
	MatrixXd u;
	MatrixXd xy;
	int r_y_inverse;
	int s_1_inverse;
	int s_2_inverse;

public:
	QuantizedController(MatrixXd FGR, MatrixXd HJ, MatrixXd x_init, int r_y_inverse, int s_1_inverse, int s_2_inverse) {
		this->FGR = FGR;
		this->HJ = HJ;
		this->x = x_init;
		this->r_y_inverse = r_y_inverse;
		this->s_1_inverse = s_1_inverse;
		this->s_2_inverse = s_2_inverse;
	}

	MatrixXd GetOutput(MatrixXd y) {
		xy = MergeByRow(x, y);
		u = HJ * xy;
		return u / s_1_inverse / s_2_inverse;
	}
	void UpdateState(MatrixXd u_prime) {
		MatrixXd xyu = MergeByRow(xy, u_prime);
		x = FGR * xyu;
	}
	MatrixXd MergeByRow(MatrixXd a, MatrixXd b) {
		MatrixXd result(a.rows() + b.rows(), a.cols());
		for (int i = 0;i < a.rows();i++)
			for (int j = 0;j < a.cols();j++)
				result(i, j) = a(i, j);
		for (int i = 0;i < b.rows();i++)
			for (int j = 0;j < b.cols();j++)
				result(a.rows() + i, j) = b(i, j);
		return result;
	}
};

//---------TEST CODE FOR SIMPLE LWE OPERATION--------------
void LWECheck() {
	Encrypter *enc = new Encrypter(1000, 1, 1, 10000, 1000, 3, 1);
	Decrypter *dec = enc->GenerateDecrypter();

	MatrixXd A(2, 3);
	A << 1.25, 0.125, -10,
		-100, -5, 1000;
	cout << "A:\n";
	cout << (A) << endl;

	MatrixXd B(2, 3);
	B << 4, 521, -10,
		150, 1, 2000;
	cout << "B:\n";
	cout << (B) << endl;

	MatrixXu A_enc = enc->Enc(A, false);
	MatrixXu B_enc = enc->Enc(B, false);
	MatrixXu A_enc_mult_scalar = enc->ScalarMult(3, A_enc);
	MatrixXu A_enc_add_B_enc = enc->Add(A_enc, B_enc);
	//cout << "Enc(A):\n";
	//cout << (A_enc) << endl;
	MatrixXd dec1 = dec->Dec(A_enc);
	cout << "Dec(Enc(A)):\n";
	cout << (dec1) << endl;
	MatrixXd dec2 = dec->Dec(A_enc_add_B_enc);
	cout << "Dec(Enc(A) + Enc(B)):\n";
	cout << (dec2) << endl;
	MatrixXd dec3 = dec->Dec(A_enc_mult_scalar);
	cout << "Dec(Enc(A)*3):\n";
	cout << (dec3) << endl;
}

//------TEST CODE FOR SIMPLE GSW MULTIPLICATION(ciphertext multiplied by ciphertext) OPERATION-------
void GSWCheck() {
	Encrypter *enc = new Encrypter(100, 1, 1, 8000, 1000, 3, 1);
	Decrypter *dec = enc->GenerateDecrypter();

	int FG_row = 2;
	int FG_col = 3;
	MatrixXd FG(2, 3);
	FG << -4, 3, 1,
		4, 11, 0;
	int xy_row = FG_col;
	int xy_col = 2;
	MatrixXd xu(3, 2);
	xu << -5, 1,
		-2.5, 0.125,
		2, 0;

	printf("FG:\n");
	cout << (FG) << endl;
	printf("xu:\n");
	cout << (xu) << endl;

	MatrixXu encm_FG = enc->Encm(FG);
	MatrixXu enc_xu = enc->Enc(xu, false);
	MatrixXu split_enc_xu = enc->SplitMtx(enc_xu);
	MatrixXu enc_mult = enc->MultMxM(encm_FG, split_enc_xu);
	MatrixXd answer = dec->Dec(enc_mult);
	printf("Dec(Enc(FG) x Enc(xu)):\n");
	cout << (answer) << endl;
}

int main()
{
	//LWECheck();
	//GSWCheck();
	SystemBuilder* controlSystem = new SystemBuilder();
	controlSystem->ControlLoop();
	return 0;
}
