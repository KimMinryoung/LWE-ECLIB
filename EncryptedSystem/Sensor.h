#pragma once
#include "EncryptedController.h"
#include "Encrypter.h"
class Sensor {
private:
	EncryptedController * controller;
	Encrypter* encrypter;
public:
	Sensor(EncryptedController* controller, Encrypter* encrypter);
	void GetPlantOutput(MatrixXd y);
};