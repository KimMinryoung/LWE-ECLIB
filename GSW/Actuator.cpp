#include "Actuator.h"

Actuator::Actuator(Plant* plant) {
	this->plant = plant;
}
void Actuator::GetControllerOutput(MatrixXu enc_u) {
	MatrixXd u = encdec->Dec_u(enc_u);
	plant->GetActuatorSignal(u);
	MatrixXu enc_u_prime = encdec->Enc(u, true);
	controller->UpdateState(enc_u_prime);
}