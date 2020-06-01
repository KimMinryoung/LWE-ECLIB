#include "Actuator.h"
#include "Encrypter.h"
#include "Plant.h"

Actuator::Actuator(Encrypter* encdec) {
	this->encdec = encdec;
}
Actuator::Actuator(Plant* plant, Encrypter* encdec) {
	this->plant = plant;
	this->encdec = encdec;
}
void Actuator::SetPlant(Plant* plant) {
	this->plant = plant;
}
void Actuator::SetController(EncryptedController* controller) {
	this->controller = controller;
}
void Actuator::GetControllerOutput(MatrixXu enc_u) {
	MatrixXd u = encdec->Dec_u(enc_u);
	MatrixXu enc_u_prime = encdec->Enc(u, true);
	controller->UpdateState(enc_u_prime);
	plant->GetActuatorSignal(u);
}