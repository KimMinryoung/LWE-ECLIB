#include "Actuator.h"
#include "Encrypter.h"
#include "Plant.h"

Actuator::Actuator(Decrypter* dec, Encrypter* enc) {
	this->dec = dec;
	this->enc = enc;
}
void Actuator::SetPlant(Plant* plant) {
	this->plant = plant;
}
void Actuator::SetController(EncryptedController* controller) {
	this->controller = controller;
}
void Actuator::GetControllerOutput(MatrixXu enc_u) {
	MatrixXd u = dec->Dec_u(enc_u);
	MatrixXu enc_u_prime = enc->Enc(u, true);
	controller->UpdateState(enc_u_prime);
	plant->GetActuatorSignal(u);
}