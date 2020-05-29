#pragma once
#include "EncryptedController.h"
#include "Encrypter.h"
class Sensor {
private:
	EncryptedController * controller;
	Encrypter* encrypter;
public:
	Sensor(EncryptedController* controller);
	void GetPlantOutput(MatrixXd y);
};