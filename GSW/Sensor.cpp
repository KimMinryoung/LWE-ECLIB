#include "Sensor.h"
Sensor::Sensor(EncryptedController* controller, Encrypter* encrypter) {
	this->controller = controller;
	this->encrypter = encrypter;
}
void Sensor::GetPlantOutput(MatrixXd y) {
	controller->GetOutput(encrypter->Enc(y, true));
}