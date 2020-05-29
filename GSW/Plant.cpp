#include "Plant.h"

Plant::Plant() {
}
void Plant::GetActuatorSignal(MatrixXd u) {

	// fill with any plant operation
	MatrixXd xu = MergeByRow(x, u);
	x = AB * xu;
	y = CD * xu;
	// ~~~~~~~~~~~~~~

	SendOutputToSensor();
}
void Plant::SendOutputToSensor() {
	sensor->GetPlantOutput(Substraction(r, y));
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