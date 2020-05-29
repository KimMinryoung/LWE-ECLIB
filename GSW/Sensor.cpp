#include "Sensor.h"
Sensor::Sensor(EncryptedController* controller) {
	this->controller = controller;
}
void Sensor::GetPlantOutput(MatrixXd y) {
	controller->GetOutput(encrypter->Enc(y, true));
}