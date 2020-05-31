#include "Plant.h"
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;
#include <chrono>

Plant::Plant(EncryptedController* controller, Sensor* sensor) {
	this->controller = controller;
	this->sensor = sensor;

	// Parameters of any form of plant
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
	// ~~~~~~~~~~

	cout << ">> plant <<" << endl;
	cout << "x+ = AB * xy" << endl;
	cout << "y = CD * xy" << endl;
	printf("plant matrix AB=\n");
	cout << AB << endl;
	printf("plant matrix CD=\n");
	cout << CD << endl;
	printf("plant initial state x=\n");
	cout << x << endl;
	printf("plant initial output y=\n");
	cout << y << endl;
	printf("reference signal r=\n");
	cout << r << endl;
}
void Plant::GetActuatorSignal(MatrixXd u) {
	step++;

	// Fill with any plant operation
	MatrixXd xu = MergeByRow(x, u);
	x = AB * xu;
	xu = MergeByRow(x, u);
	y = CD * xu;
	// ~~~~~~~~~~~

}
void Plant::SendOutputToSensor() {
	if (step % 50 == 0) {
		cout << "r - y=\t\t(reference signal vs.encrypted system)" << endl;
		cout << (Substraction(r, y)) << endl;
	}
	sensor->GetPlantOutput(Substraction(r,y));
}
void Plant::ControlLoop(){
	step = 0;
	while (true) {
		SendOutputToSensor();
		time_lapsed += duration_cast<duration<double>>(high_resolution_clock::now() - lastTime).count();
		lastTime = high_resolution_clock::now();
		if (time_lapsed > T_s) {
			time_lapsed -= T_s;
			SendOutputToSensor();
		}
	}
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