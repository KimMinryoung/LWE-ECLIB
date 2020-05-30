#include "Plant.h"

Plant::Plant(EncryptedController* controller, Sensor* sensor) {
	this->controller = controller;
	this->sensor = sensor;
	MatrixXd AB_(3, 4);
	AB_ << 0.9048, 0.08611, 0.003306, 0.0001222,
		0, 0.8187, 0.05636, 0.003428,
		0, 0, 0.3679, 0.06321;
	AB = AB_;
	MatrixXd CD_(1, 4);
	CD_ << 1, 0, 0, 0;
	CD = CD_;
	MatrixXd x_(3, 1);
	x_ << 100, 70, 50;
	x = x_;
	MatrixXd y_(1, 1);
	y_ << 0;
	y = y_;
	MatrixXd r_(1, 1);
	r_ << 25.75;
	r = r_;
	T_s = 0.1;
}
void Plant::GetActuatorSignal(MatrixXd u) {
	//cout << "GetActuatorSignal" << endl;
	step++;
	// fill with any plant operation
	MatrixXd xu = MergeByRow(x, u);
	x = AB * xu;
	xu = MergeByRow(x, u);
	y = CD * xu;
	// ~~~~~~~~~~~~~~
	//cout << "step=" << t << endl;
	SendOutputToSensor();
}
void Plant::SendOutputToSensor() {
	//cout << "SendOutputToSensor" << endl;
	if (step % 50 == 0) {
		cout << "r - y=\t\t(reference signal vs.encrypted system)" << endl;
		cout << (Substraction(r, y)) << endl;
	}
	sensor->GetPlantOutput(Substraction(r,y));
}
void Plant::ControlLoop(){
	step = 0;
	SendOutputToSensor();
	/*
	for (int t = 0;t <= 120 / T_s;t++) {
		GetOutput();
		if (t % 50 == 10) {
			cout << "step=" << t << endl;
			cout << "r - y=\t\t(reference signal vs.encrypted system)" << endl;
			cout << (Substraction(r, y)) << endl;
		}
		u = encdec->Dec_u(controller->GetOutput(encdec->Enc(Substraction(r, y), true)));
		controller->UpdateState(encdec->Enc(u, true));
		UpdateState();
	}*/
}

MatrixXd Plant::MergeByRow(MatrixXd a, MatrixXd b) {
	MatrixXd result(a.rows() + b.rows(), a.cols());
	for (int i = 0;i < a.rows();i++)
		for (int j = 0;j < a.cols();j++)
			result(i, j) = a(i, j);
	for (int i = 0;i < b.rows();i++)
		for (int j = 0;j < b.cols();j++)
			result(a.rows() + i, j) = b(i, j);
	return result;
}
MatrixXd Plant::Substraction(MatrixXd mtx_left, MatrixXd mtx_right) {
	MatrixXd result(mtx_left.rows(), mtx_left.cols());
	for (int i = 0;i < mtx_left.rows(); i++)
		for (int j = 0;j < mtx_left.cols();j++)
			result(i, j) = mtx_left(i, j) - mtx_right(i, j);
	return result;
}