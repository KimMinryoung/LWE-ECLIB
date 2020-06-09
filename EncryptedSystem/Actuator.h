#pragma once
class Encrypter;
#include "EncryptedController.h"
#include "Decrypter.h"
class Plant;
class Actuator {
private:
	Plant * plant;
	EncryptedController* controller;
	Decrypter* dec;
	Encrypter* enc;
public:
	Actuator(Decrypter* dec, Encrypter* enc);
	void SetPlant(Plant* plant);
	void SetController(EncryptedController* controller);
	void GetControllerOutput(MatrixXu enc_u);
};