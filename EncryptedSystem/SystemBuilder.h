#pragma once
#include <fstream>
#include <math.h>
#include "EncryptedController.h"
#include "Encrypter.h"
#include "Plant.h"
#include "Actuator.h"
#include "Sensor.h"
typedef Eigen::Matrix<double, 2, 1> Vector2d;

class SystemBuilder {

private:
	// controller matrices
	MatrixXd F;
	MatrixXd G;
	MatrixXd R;
	MatrixXd H;
	MatrixXd J;
	int s_1_inverse;
	int s_2_inverse;
	int r_y_inverse;
	int r_u_inverse;
	int U;
	int L_inverse;
	double bandwidth; // data bandwidth of network(Mbit/s)
	double sigma = 1;
	double degrade_bound = 0.01;
	double T_s;
	Plant* plant;
	EncryptedController *controller;
	Encrypter *encdec;

	string filePath = "parameters.txt";
	string writeFilePath = "result.txt";
	ifstream ifs;
	ofstream ofs;

public:
	SystemBuilder();

	VectorXd Determinant(Eigen::Matrix<Vector2d, Eigen::Dynamic, Eigen::Dynamic> mtx);
	Eigen::Matrix<VectorXd, Eigen::Dynamic, Eigen::Dynamic> Adjoint(Eigen::Matrix<Vector2d, Eigen::Dynamic, Eigen::Dynamic> mtx);
	Eigen::Matrix<VectorXd, 1, Eigen::Dynamic> LeftMtxMultipleAdj(MatrixXd leftMtx, Eigen::Matrix<VectorXd, Eigen::Dynamic, Eigen::Dynamic> adj);
	VectorXd LeftSideMultipleRightMtx(Eigen::Matrix<VectorXd, 1, Eigen::Dynamic> leftSide, MatrixXd rightMtx);

	void BuildController(double T_s, int F_precision, int G_precision, int H_precision, int J_precision);

	void ControlLoop();

	MatrixXd MergeByRow(MatrixXd a, MatrixXd b);
	double GetInfinityNorm(MatrixXd m);
	double DeltaEnc();
	double DeltaMult(int n);
	int needed_precision;
	MatrixXd ReadMatrix(string end);
};