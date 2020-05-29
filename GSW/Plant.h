#pragma once
#include "EncryptedController.h"
#include "Sensor.h"
class Plant {

private:
	EncryptedController * controller;
	Sensor* sensor;

	// any plant parameters
	MatrixXd AB;
	MatrixXd CD;

	MatrixXd x;
	MatrixXd y;
	MatrixXd r;
	double T_s;
	int r_u_inverse;

	MatrixXd xu;

public:
	Plant(EncryptedController* controller, Sensor* sensor);
	void GetActuatorSignal(MatrixXd u);
	void SendOutputToSensor();
	void ControlLoop();
	MatrixXd MergeByRow(MatrixXd a, MatrixXd b);
	MatrixXd Substraction(MatrixXd mtx_left, MatrixXd mtx_right);
};