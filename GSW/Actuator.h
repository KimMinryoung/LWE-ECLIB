#pragma once
#include "Encrypter.h"
#include "EncryptedController.h"
#include "Plant.h"
class Actuator {
private:
	Plant * plant;
	EncryptedController* controller;
	Encrypter* encdec;
public:
	Actuator(Plant* plant);
	void GetControllerOutput(MatrixXu enc_u);
};