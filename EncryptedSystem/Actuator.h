#pragma once
class Encrypter;
#include "EncryptedController.h"
class Plant;
class Actuator {
private:
	Plant * plant;
	EncryptedController* controller;
	Encrypter* encdec;
public:
	Actuator(Encrypter* encdec);
	Actuator(Plant* plant, Encrypter* encdec);
	void SetPlant(Plant* plant);
	void SetController(EncryptedController* controller);
	void GetControllerOutput(MatrixXu enc_u);
};