//#pragma _D_SCL_SECURE_NO_WARNINGS
#include "stdafx.h"
//#include <mkl.h>
//#define EIGEN_USE_MKL_ALL
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <string>
#include <windows.h>
#include "EncryptedController.h"
#include "Encrypter.h"
#include "Plant.h"
#include "Actuator.h"
#include "Sensor.h"

#include <chrono>

using namespace std::chrono;

//#define EIGEN_VECTORIZE_SSE4_2
#include "Eigen-3.3.7/Eigen/Dense"
using Eigen::MatrixXd;
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, 1> VectorXu;
typedef Eigen::Matrix<unsigned __int64, 1, Eigen::Dynamic> RowVectorXu;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;
typedef Eigen::Matrix<double, 2, 1> Vector2d;

using namespace std;

/*
class Controller {

private:

public:
	Controller() {}

	virtual MatrixXd GetOutput(MatrixXd y) = 0;
	virtual void UpdateState() = 0;
	virtual void UpdateState(MatrixXd u_prime) = 0;
	MatrixXd MergeByRow(MatrixXd a, MatrixXd b) {
		MatrixXd result(a.rows() + b.rows(), a.cols());
		for (int i = 0;i < a.rows();i++)
			for (int j = 0;j < a.cols();j++)
				result(i, j) = a(i, j);
		for (int i = 0;i < b.rows();i++)
			for (int j = 0;j < b.cols();j++)
				result(a.rows() + i, j) = b(i, j);
		return result;
	}
	MatrixXu MergeByRow(MatrixXu a, MatrixXu b) {
		MatrixXu result(a.rows() + b.rows(), a.cols());
		for (int i = 0;i < a.rows();i++)
			for (int j = 0;j < a.cols();j++)
				result(i, j) = a(i, j);
		for (int i = 0;i < b.rows();i++)
			for (int j = 0;j < b.cols();j++)
				result(a.rows() + i, j) = b(i, j);
		return result;
	}
};*/

class ConvertedController {
private:
	MatrixXd FGR;
	MatrixXd HJ;
	MatrixXd x;
	MatrixXd u;
	MatrixXd xy;

public:
	ConvertedController(MatrixXd FGR, MatrixXd HJ, MatrixXd x_init) {
		this->FGR = FGR;
		this->HJ = HJ;
		this->x = x_init;
	}

	MatrixXd GetOutput(MatrixXd y) {
		xy = MergeByRow(x, y);
		u = HJ * xy;
		return u;
	}
	void UpdateState(MatrixXd u_prime) {
		MatrixXd xyu = MergeByRow(xy, u_prime);
		x = FGR * xyu;
	}
	MatrixXd MergeByRow(MatrixXd a, MatrixXd b) {
		MatrixXd result(a.rows() + b.rows(), a.cols());
		for (int i = 0;i < a.rows();i++)
			for (int j = 0;j < a.cols();j++)
				result(i, j) = a(i, j);
		for (int i = 0;i < b.rows();i++)
			for (int j = 0;j < b.cols();j++)
				result(a.rows() + i, j) = b(i, j);
		return result;
	}
};
class QuantizedController {
private:
	MatrixXd FGR;
	MatrixXd HJ;
	MatrixXd x;
	MatrixXd u;
	MatrixXd xy;
	int r_y_inverse;
	int s_1_inverse;
	int s_2_inverse;

public:
	QuantizedController(MatrixXd FGR, MatrixXd HJ, MatrixXd x_init, int r_y_inverse, int s_1_inverse, int s_2_inverse) {
		this->FGR = FGR;
		this->HJ = HJ;
		this->x = x_init;
		this->r_y_inverse = r_y_inverse;
		this->s_1_inverse = s_1_inverse;
		this->s_2_inverse = s_2_inverse;
	}

	MatrixXd GetOutput(MatrixXd y) {
		xy = MergeByRow(x, y);
		u = HJ * xy;
		return u / s_1_inverse / s_2_inverse;
	}
	void UpdateState(MatrixXd u_prime) {
		MatrixXd xyu = MergeByRow(xy, u_prime);
		x = FGR * xyu;
	}
	MatrixXd MergeByRow(MatrixXd a, MatrixXd b) {
		MatrixXd result(a.rows() + b.rows(), a.cols());
		for (int i = 0;i < a.rows();i++)
			for (int j = 0;j < a.cols();j++)
				result(i, j) = a(i, j);
		for (int i = 0;i < b.rows();i++)
			for (int j = 0;j < b.cols();j++)
				result(a.rows() + i, j) = b(i, j);
		return result;
	}
};

class ControlSystem {

private:
	// controller matrices
	MatrixXd F;
	MatrixXd G;
	MatrixXd R;
	MatrixXd H;
	MatrixXd J;
	int s_1_inverse;
	int s_2_inverse;
	int r_y_inverse;
	int r_u_inverse;
	int U;
	int L_inverse;
	double sigma = 1;
	double degrade_bound = 3;
	double T_s;
	Plant* plant;
	EncryptedController *controller;
	ConvertedController *originalController;
	QuantizedController *quantizedController;
	Encrypter *encdec;

	string filePath = "parameters.txt";
	string writeFilePath = "result.txt";
	ifstream ifs;
	ofstream ofs;

public:
	ControlSystem() {
		ifs.open(filePath.data());
		if (!ifs.is_open()) {
			cout << "No input file" << endl;
			return;
		}
		string line;
		getline(ifs, line, '\n');
		F = ReadMatrix("G");
		int F_precision = needed_precision;
		G = ReadMatrix("H");
		int G_precision = needed_precision;
		H = ReadMatrix("J");
		int H_precision = needed_precision;
		J = ReadMatrix("T_s");
		int J_precision = needed_precision;
		getline(ifs, line, '\n');
		size_t point_pos;
		T_s = stod(line);
		int T_s_decimal;
		point_pos = line.find('.');
		if (point_pos == string::npos)
			T_s_decimal = 0;
		else
			T_s_decimal = line.length() - (point_pos + 1);

		getline(ifs, line, '\n');
		getline(ifs, line, '\n');
		size_t pos = line.find('\t');
		string token = line.substr(0, pos);
		r_y_inverse = stoi(token);
		line.erase(0, pos + 1);
		token = line;
		r_u_inverse = stoi(token);
		line.erase(0, pos + 1);
		token = line;
		U = stoi(token);
		getline(ifs, line, '\n');
		getline(ifs, line, '\n');
		pos = line.find('\t');
		token = line.substr(0, pos);
		sigma = stod(token);
		line.erase(0, pos + 1);
		token = line;
		degrade_bound = stod(token);

		ifs.close();

		cout << "sampling time:\nT_s=" << T_s << endl;
		cout << "degrade:\n" << degrade_bound << "\nsigma:\n" << sigma << endl;
		printf("---------------\n");

		BuildController(T_s, F_precision, G_precision, H_precision, J_precision);
	}

	VectorXd Determinant(Eigen::Matrix<Vector2d, Eigen::Dynamic, Eigen::Dynamic> mtx) {
		int row = mtx.rows();
		int col = row;
		if (row == 1)
			return mtx(0, 0);
		VectorXd det = VectorXd::Zero(row + 1, 1);
		for (int j = 0; j < col; j++) {
			Eigen::Matrix<Vector2d, Eigen::Dynamic, Eigen::Dynamic> part(row - 1, col - 1);
			if (j == 0)
				part = mtx.block(1, j + 1, row - 1, col - 1); // block(i,j,p,q): block of size (p,q), starting at (i,j)
			else if (j == col - 1)
				part = mtx.block(1, 0, row - 1, col - 1);
			else
				part << mtx.block(1, 0, row - 1, j), mtx.block(1, j + 1, row - 1, col - j - 1);
			VectorXd part_det = (j % 2 == 0 ? 1 : -1) * Determinant(part);
			for (int i = 0; i < row; i++) {
				det[i] += mtx(0, j)[0] * part_det[i];
				det[i+1] += mtx(0, j)[1] * part_det[i];
			}
		}
		return det;
	}
	Eigen::Matrix<VectorXd, Eigen::Dynamic, Eigen::Dynamic> Adjoint(Eigen::Matrix<Vector2d, Eigen::Dynamic, Eigen::Dynamic> mtx) {
		int row = mtx.rows();
		if (row == 1)
			return mtx.cast<VectorXd>();
		Eigen::Matrix<VectorXd, Eigen::Dynamic, Eigen::Dynamic> adj(row, row);
		for (int i = 0; i < row; i++) {
			for (int j = 0; j < row; j++) {
				Eigen::Matrix<Vector2d, Eigen::Dynamic, Eigen::Dynamic> part(row - 1, row - 1);
				if (j == 0) {
					if (i == 0)
						part = mtx.block(1, 1, row - 1, row - 1);
					else if (i == row - 1)
						part = mtx.block(0, 1, row - 1, row - 1);
					else
						part << mtx.block(0, 1, i, row - 1), mtx.block(i + 1, 1, row - i - 1, row - 1);
				}
				else if (j == row - 1) {
					if (i == 0)
						part = mtx.block(1, 0, row - 1, row - 1);
					else if (i == row - 1)
						part = mtx.block(0, 0, row - 1, row - 1);
					else
						part << mtx.block(0, 0, i, row - 1), mtx.block(i + 1, 0, row - i - 1, row - 1);
				}
				else {
					if (i == 0)
						part << mtx.block(1, 0, row - 1, j), mtx.block(1, j + 1, row - 1, row - j - 1);
					else if (i == row - 1)
						part << mtx.block(0, 0, row - 1, j), mtx.block(0, j + 1, row - 1, row - j - 1);
					else { // four blocks
						Eigen::Matrix<Vector2d, Eigen::Dynamic, Eigen::Dynamic> upper(i, row - 1);
						upper << mtx.block(0, 0, i, j), mtx.block(0, j + 1, i, row - j - 1);
						Eigen::Matrix<Vector2d, Eigen::Dynamic, Eigen::Dynamic> lower(row - i - 1, row - 1);
						lower << mtx.block(i + 1, 0, row - i - 1, j), mtx.block(i + 1, j + 1, row - i - 1, row - j - 1);
						part << upper, lower;
					}
				}
				VectorXd cofactor = ((i + j) % 2 == 0 ? 1 : -1) * Determinant(part);
				adj(j, i) = cofactor;
			}
		}
		return adj;
	}
	Eigen::Matrix<VectorXd, 1, Eigen::Dynamic> LeftMtxMultipleAdj(MatrixXd leftMtx, Eigen::Matrix<VectorXd, Eigen::Dynamic, Eigen::Dynamic> adj) {
		int row = adj.rows();
		int col = adj.cols();
		Eigen::Matrix<VectorXd, 1, Eigen::Dynamic> result(1, col);
		VectorXd init = VectorXd::Zero(row);
		for (int j = 0; j < col; j++) {
			result[j] = init;
			for (int i = 0; i < row; i++) {
				result[j] += leftMtx(0, i) * adj(i, j);
			}
		}
		return result;
	}
	VectorXd LeftSideMultipleRightMtx(Eigen::Matrix<VectorXd, 1, Eigen::Dynamic> leftSide, MatrixXd rightMtx) {
		int col = leftSide.cols();
		VectorXd result = VectorXd::Zero(leftSide[0].rows());
		for (int i = 0; i < col; i++) {
			result += rightMtx(i, 0) * leftSide[i];
		}
		return result;
	}

	void BuildController(double T_s, int F_precision, int G_precision, int H_precision, int J_precision) {
		int x_row = F.rows();
		int y_row = G.cols();
		int u_row = H.rows(); // u_row should be 1
		VectorXd a_(x_row);
		VectorXd b_1(x_row);
		double b_0;

		// s_1_inverse: scaling factor for controller matrix G and R
		s_1_inverse = pow(10, max(H_precision + G_precision + F_precision, 2 * F_precision + J_precision));
		// s_2_inverse: scaling factor for controller matrices H and J
		s_2_inverse = pow(10, J_precision);

		// converting to integer controller ->
		Eigen::Matrix<Vector2d, Eigen::Dynamic, Eigen::Dynamic> sI_F(x_row, x_row);
		for (int i = 0; i < x_row; i++) {
			for (int j = 0; j < x_row; j++) {
				Vector2d f_element;
				f_element << ((i == j) ? 1 : 0), -F(i, j);
				sI_F(i, j) = f_element;
			}
		}
		VectorXd denominator = Determinant(sI_F);
		Eigen::Matrix<VectorXd, Eigen::Dynamic, Eigen::Dynamic> adj = Adjoint(sI_F);
		Eigen::Matrix<VectorXd, 1, Eigen::Dynamic> lft = LeftMtxMultipleAdj(H, adj);
		VectorXd numerator = LeftSideMultipleRightMtx(lft, G);
		for (int i = 0; i < x_row; i++)
			a_[i] = denominator[i + 1];
		b_1 = numerator;
		b_0 = J(0, 0);
		R = MatrixXd(x_row, u_row);
		MatrixXd FGR(x_row, x_row + y_row + u_row);
		for (int row = 0; row < x_row; row++) {
			for (int col = 0; col < x_row; col++)
				F(row, col) = FGR(row, col) = (row + 1 == col ? 1 : 0);
			for (int col = x_row; col < x_row + y_row; col++) {
				G(row, col - x_row) = FGR(row, col) = b_1[row] + b_0 * a_[row];
			}
			for (int col = x_row + y_row; col < x_row + y_row + u_row; col++) {
				R(row, col - x_row - y_row) = FGR(row, col) = -a_[row];
			}
		}
		MatrixXd FGR_scaled(x_row, x_row + y_row + u_row);
		for (int row = 0;row < x_row;row++) {
			for (int col = 0;col < x_row;col++)
				FGR_scaled(row, col) = FGR(row, col);
			for (int col = x_row;col < x_row + y_row + u_row;col++)
				FGR_scaled(row, col) = round(FGR(row, col) * s_1_inverse);
		}

		MatrixXd HJ(u_row, x_row + y_row); // u_row=1
		H(0, 0) = HJ(0, 0) = 1;
		for (int i = 1; i < x_row + y_row - 1; i++) {
			H(0, i) = HJ(0, i) = 0;
		}
		J(0, 0) = HJ(0, x_row + y_row - 1) = b_0;
		MatrixXd HJ_scaled = MatrixXd(u_row, x_row + y_row);
		for (int row = 0;row < u_row;row++) {
			for (int col = 0;col < x_row;col++)
				HJ_scaled(row, col) = round(HJ(row, col) * s_2_inverse);
			for (int col = x_row;col < x_row + y_row;col++)
				HJ_scaled(row, col) = round(HJ(row, col) * s_2_inverse * s_1_inverse);
		}
		// -> converting to integer controller

		cout << ">> converted controller <<" << endl;
		cout << "x+ = FGR * xyu" << endl;
		cout << "u = HJ * xy" << endl;
		cout << "FGR=" << endl;
		cout << FGR << endl;
		cout << "FGR_scaled=" << endl;
		cout << FGR_scaled << endl;
		cout << "HJ=" << endl;
		cout << HJ << endl;
		cout << "HJ_scaled=" << endl;
		cout << HJ_scaled << endl;

		MatrixXd x_init_con = MatrixXd::Zero(x_row, 1);
		MatrixXd x_init_con_scaled = s_1_inverse * x_init_con;

		// temporal system for computing time simulation
		encdec = new Encrypter(r_y_inverse, s_1_inverse, s_2_inverse, U, r_y_inverse, sigma, -1);
		MatrixXu encm_FGR = encdec->Encm(FGR_scaled);
		MatrixXu encm_HJ = encdec->Encm(HJ_scaled);
		MatrixXu enc_x_init_con = encdec->Enc(x_init_con, false);
		Actuator* tempAct = new Actuator(encdec);
		EncryptedController* tempController = new EncryptedController(encm_FGR, encm_HJ, enc_x_init_con, encdec->Getq(), tempAct);
		Sensor* tempSensor = new Sensor(tempController, encdec);
		Plant* tempPlant = new Plant(tempSensor);
		tempAct->SetPlant(tempPlant);
		tempAct->SetController(tempController);
		int n = encdec->Set_n(tempPlant->ControlTimeTest(), T_s);
		// -> proper n(ciphertext dimension) to satisfy computing time constraint

		cout << "delta_Enc: "<<DeltaEnc()<<" delta_Mult: "<< DeltaMult(n) << endl;

		// 1/L choosing routine
		double G_norm = GetInfinityNorm(G);
		double R_norm = GetInfinityNorm(R);
		double J_norm = GetInfinityNorm(J);

		cout << "degrade=" << degrade_bound << endl;
		double L1 = ((G_norm + R_norm) * DeltaEnc() + (x_row + y_row + u_row) * DeltaMult(n) / s_1_inverse) / ((G_norm + R_norm) / (double)r_y_inverse / 2) * pow(10, degrade_bound);
		double L2 = (J_norm * DeltaEnc() + (x_row + y_row) * DeltaMult(n) / (double)s_1_inverse / (double)s_2_inverse) / (J_norm / (2 * r_y_inverse) + 1 / (2 * r_u_inverse)) * pow(10, degrade_bound);
		double L3 = 2 * DeltaEnc() * pow(10, degrade_bound);
		int logL_inverse = ceil(log2(max(r_y_inverse, max(max(L1, L2), L3))));
		int logN = ceil(log2(U) + logL_inverse + log2(s_1_inverse) + log2(s_2_inverse));
		bool adjusted = false;
		if (logN > 48) {
			adjusted = true;
			logL_inverse -= logN - 48;
		}
		L_inverse = pow(2, logL_inverse);
		cout << "log2(1/L)=" << logL_inverse << ", 1/L=" << L_inverse << endl;
		double alpha_x = ((G_norm + R_norm) * DeltaEnc() + (x_row + y_row + u_row) * DeltaMult(n) / s_1_inverse)  / ((G_norm + R_norm)/ (double)r_y_inverse / 2) / L_inverse;
		double alpha_u = (J_norm * DeltaEnc() + (x_row + y_row) * DeltaMult(n) / (double)s_1_inverse / (double)s_2_inverse) / (J_norm / (2 * r_y_inverse) + 1 / (2 * r_u_inverse)) / L_inverse;
		double alpha_0 = 2 * DeltaEnc() / L_inverse;
		double alpha = max(max(alpha_x, alpha_u), alpha_0);
		degrade_bound = log10(1 / alpha);
		cout << "alpha_x=" << alpha_x << " alpha_u=" << alpha_u << " alpha_0=" << alpha_0 << endl;
		if(adjusted)
			cout << "new degrade=" << degrade_bound << endl;

		encdec = new Encrypter(r_y_inverse, s_1_inverse, s_2_inverse, U, L_inverse, sigma, n);

		// building modules and construct system
		encm_FGR = encdec->Encm(FGR_scaled);
		encm_HJ = encdec->Encm(HJ_scaled);
		enc_x_init_con = encdec->Enc(x_init_con_scaled, true);
		Actuator* actuator = new Actuator(encdec);
		controller = new EncryptedController(encm_FGR, encm_HJ, enc_x_init_con, encdec->Getq(), actuator);

		/*
		originalController = new ConvertedController(FGR, HJ, x_init_con);
		quantizedController = new QuantizedController(FGR_scaled, HJ_scaled, x_init_con_scaled, r_y_inverse, s_1_inverse, s_2_inverse);*/

		Sensor* sensor = new Sensor(controller, encdec);
		plant = new Plant(sensor);
		actuator->SetPlant(plant);
		actuator->SetController(controller);
	}

	void ControlLoop() {
		plant->ControlLoop();
	}

	MatrixXd MergeByRow(MatrixXd a, MatrixXd b) {
		MatrixXd result(a.rows() + b.rows(), a.cols());
		for (int i = 0;i < a.rows();i++)
			for (int j = 0;j < a.cols();j++)
				result(i, j) = a(i, j);
		for (int i = 0;i < b.rows();i++)
			for (int j = 0;j < b.cols();j++)
				result(a.rows() + i, j) = b(i, j);
		return result;
	}
	double GetInfinityNorm(MatrixXd m) {
		double max = 0;
		for (int i = 0;i < m.rows();i++) {
			double sum = 0;
			for (int j = 0;j < m.cols();j++)
				sum += abs(m(i, j));
			if (sum > max)
				max = sum;
		}
		return max;
	}
	double DeltaEnc() {
		return round(2.58*sigma);
	}
	double DeltaMult(int n) {
		int d = 3;
		int nu = 16;
		return round(d*(n + 1)*2.58*sigma * nu) + 1;
	}
	int needed_precision;
	MatrixXd ReadMatrix(string end) {
		vector<vector<double>> m_;
		string line;
		int i = 0;
		needed_precision = 0;
		while (getline(ifs, line, '\n')) {
			if (line.compare(end) == 0)
				break;
			m_.push_back(vector<double>());
			size_t pos = 0;
			string token;
			size_t point_pos;
			int precision = 0;
			while ((pos = line.find('\t')) != std::string::npos) {
				token = line.substr(0, pos);
				m_[i].push_back(stod(token));
				point_pos = token.find('.');
				if (point_pos == string::npos)
					precision = 0;
				else
					precision = token.length() - (point_pos + 1);
				if (precision > needed_precision)
					needed_precision = precision;
				line.erase(0, pos + 1);
			}
			token = line;
			m_[i].push_back(stod(token));
			point_pos = token.find('.');
			if (point_pos == string::npos)
				precision = 0;
			else
				precision = token.length() - (point_pos + 1);
			if (precision > needed_precision)
				needed_precision = precision;
			line.erase(0, pos + 1);
			i++;
		}
		MatrixXd m(m_.size(), m_[0].size());
		for (int i = 0; i < m.rows(); i++)
			for (int j = 0; j < m.cols(); j++)
				m(i, j) = m_[i][j];
		return m;
	}
};

//------------------------------------
void LWECheck() {
	Encrypter *encdec = new Encrypter(1000, 1, 1, 10000, 1000, 3, 1);

	MatrixXd A(2, 3);
	A << 1.25, 0.125, -10,
		-100, -5, 1000;
	cout << "A:\n";
	cout << (A) << endl;

	MatrixXd B(2, 3);
	B << 4, 521, -10,
		150, 1, 2000;
	cout << "B:\n";
	cout << (B) << endl;

	MatrixXu A_enc = encdec->Enc(A, false);
	MatrixXu B_enc = encdec->Enc(B, false);
	MatrixXu A_enc_mult_scalar = encdec->ScalarMult(3, A_enc);
	MatrixXu A_enc_add_B_enc = encdec->Add(A_enc, B_enc);
	//cout << "Enc(A):\n";
	//cout << (A_enc) << endl;
	MatrixXd dec1 = encdec->Dec(A_enc);
	cout << "Dec(Enc(A)):\n";
	cout << (dec1) << endl;
	MatrixXd dec2 = encdec->Dec(A_enc_add_B_enc);
	cout << "Dec(Enc(A) + Enc(B)):\n";
	cout << (dec2) << endl;
	MatrixXd dec3 = encdec->Dec(A_enc_mult_scalar);
	cout << "Dec(Enc(A)*3):\n";
	cout << (dec3) << endl;
}
void GSWCheck() {
	Encrypter *encdec = new Encrypter(100, 1, 1, 8000, 1000, 3, 1);

	int FG_row = 2;
	int FG_col = 3;
	MatrixXd FG(2, 3);
	FG << -4, 3, 1,
		4, 11, 0;
	int xy_row = FG_col;
	int xy_col = 2;
	MatrixXd xu(3, 2);
	xu << -5, 1,
		-2.5, 0.125,
		2, 0;

	printf("FG:\n");
	cout << (FG) << endl;
	printf("xu:\n");
	cout << (xu) << endl;

	MatrixXu encm_FG = encdec->Encm(FG);
	MatrixXu enc_xu = encdec->Enc(xu, false);
	MatrixXu split_enc_xu = encdec->SplitMtx(enc_xu);
	MatrixXu enc_mult = encdec->MultMxM(encm_FG, split_enc_xu);
	MatrixXd answer = encdec->Dec(enc_mult);
	printf("Dec(Enc(FG) x Enc(xu)):\n");
	cout << (answer) << endl;
}

int main()
{
	//LWECheck();
	//GSWCheck();
	ControlSystem* controlSystem = new ControlSystem();
	controlSystem->ControlLoop();
	return 0;
}
