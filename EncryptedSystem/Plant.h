#pragma once
#include "EncryptedController.h"
#include "Sensor.h"
#include <fstream>
#include <chrono>
using namespace std::chrono;
class ConvertedController;
class QuantizedController;

class Plant {

private:
	Sensor* sensor;

	// put any form of plant parameters in here
	MatrixXd AB;
	MatrixXd CD;
	// ~~~~~~~~~~

	MatrixXd x; // state of plant
	MatrixXd y; // output of plant
	MatrixXd r; // reference of plant
	double T_s; // sampling time of plant

	int step = 0; // current step number of system loop
	double time_lapsed = 0; // time counter
	high_resolution_clock::time_point lastTime; // recorded last time point
	bool receivedContSignal; // set true if received controller signal from actuator


	/**
	* Just for performance comparing simulation...*/
	MatrixXd x_quan;
	MatrixXd x_prime;
	MatrixXd u_quan;
	MatrixXd u_prime;
	MatrixXd y_quan; // plant output of quantized system
	MatrixXd y_prime; // plant output of original system

	string writeFilePath = "result.txt"; // output file name or path
	ofstream ofs; // output file stream

public:
	Plant(Sensor* sensor);
	//----------------------------------------------------------------------------------
	//   PLANT OPERATION
	//----------------------------------------------------------------------------------
	/**
	* get an actuator signal u
	* calculate and update plant state x
	* calculate plant output y
	* @param MatrixXu u: an encrypted actuator signal u
	*/
	void GetActuatorSignal(MatrixXd u);
	/**
	* send a substraction of reference signal r and plant output y to the sensor
	*/
	void SendOutputToSensor();
	/**
	* start infinite control system loop
	*/
	void ControlLoop();
	/**
	* return total time cost of 1 step control operation and signal communication, with current parameters(ex: n)
	* @param[out] double time_lapsed: lapsed time between plant output and actuator input
	*/
	double ControlTimeTest();
	//----------------------------------------------------------------------------------
	//   TOOL FUNCTIONS FOR PLANT OPERATION
	//----------------------------------------------------------------------------------
	/**
	* merge two plaintext matrices by row(ex: when a = {1;2} and b = {3}, return ab = {1;2;3})
	* @param MatrixXd a, b: plaintext matrix to merge
	*/
	MatrixXd MergeByRow(MatrixXd a, MatrixXd b);
	/**
	* calculate a substraction of two plaintext matrices
	*/
	MatrixXd Substraction(MatrixXd mtx_left, MatrixXd mtx_right);

	/**
	* For performance comparing simulation...
	*/
	ConvertedController * originalController; // for simulation
	QuantizedController *quantizedController; // for simulation
};


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
		for (int i = 0; i < a.rows(); i++)
			for (int j = 0; j < a.cols(); j++)
				result(i, j) = a(i, j);
		for (int i = 0; i < b.rows(); i++)
			for (int j = 0; j < b.cols(); j++)
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
	int r_u_inverse;
	int s_1_inverse;
	int s_2_inverse;

public:
	QuantizedController(MatrixXd FGR, MatrixXd HJ, MatrixXd x_init, int r_y_inverse, int r_u_inverse, int s_1_inverse, int s_2_inverse) {
		this->FGR = FGR;
		this->HJ = HJ;
		this->x = x_init;
		this->r_y_inverse = r_y_inverse;
		this->r_u_inverse = r_u_inverse;
		this->s_1_inverse = s_1_inverse;
		this->s_2_inverse = s_2_inverse;
	}

	MatrixXd GetOutput(MatrixXd y) {
		MatrixXd y_trun = TruncateMatrix(y, r_y_inverse);
		xy = MergeByRow(x, y_trun);
		u = HJ * xy;
		return u / s_1_inverse / s_2_inverse;
	}
	void UpdateState(MatrixXd u_prime) {
		MatrixXd u_trun = TruncateMatrix(u_prime, r_u_inverse);
		MatrixXd xyu = MergeByRow(xy, u_trun);
		x = FGR * xyu;
	}
	MatrixXd MergeByRow(MatrixXd a, MatrixXd b) {
		MatrixXd result(a.rows() + b.rows(), a.cols());
		for (int i = 0; i < a.rows(); i++)
			for (int j = 0; j < a.cols(); j++)
				result(i, j) = a(i, j);
		for (int i = 0; i < b.rows(); i++)
			for (int j = 0; j < b.cols(); j++)
				result(a.rows() + i, j) = b(i, j);
		return result;
	}
	MatrixXd TruncateMatrix(MatrixXd mtx, int truncator) {
		MatrixXd result(mtx.rows(), mtx.cols());
		for (int i = 0; i < mtx.rows(); i++)
			for (int j = 0; j < mtx.cols(); j++)
				result(i, j) = round(mtx(i, j) * truncator) / truncator;
		return result;
	}
};