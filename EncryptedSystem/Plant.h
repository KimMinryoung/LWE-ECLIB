#pragma once
#include "EncryptedController.h"
#include "Sensor.h"
#include <chrono>
using namespace std::chrono;
class Plant {

private:
	EncryptedController * controller;
	Sensor* sensor;

	// put any form of plant parameters in here
	MatrixXd AB;
	MatrixXd CD;
	// ~~~~~~~~~~

	MatrixXd x; // state of plant
	MatrixXd y; // output of plant
	MatrixXd r; // reference of plant
	double T_s; // sampling time of plant

	int step = 0;
	double time_lapsed = 0; // timer
	high_resolution_clock::time_point lastTime;
	bool receivedContSignal; // set true if received controller signal from actuator

public:
	Plant(EncryptedController* controller, Sensor* sensor);
	void GetActuatorSignal(MatrixXd u);
	void SendOutputToSensor();
	void ControlLoop();
	double ControlTimeTest();
	MatrixXd MergeByRow(MatrixXd a, MatrixXd b);
	MatrixXd Substraction(MatrixXd mtx_left, MatrixXd mtx_right);
};