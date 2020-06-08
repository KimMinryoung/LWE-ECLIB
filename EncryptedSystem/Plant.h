#pragma once
#include "EncryptedController.h"
#include "Sensor.h"
#include <fstream>
#include <chrono>
using namespace std::chrono;
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
};