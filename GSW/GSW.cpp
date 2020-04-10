#pragma _D_SCL_SECURE_NO_WARNINGS
//#pragma GCC optimize ("Ofast")
#include "stdafx.h"
#include "mkl.h"
#define EIGEN_USE_MKL_ALL
#include "time.h"
#include "iostream"
#include "stdlib.h"
#include "math.h"
#include "random"
#include "vector"
#include "fstream"
#include "string"
#include "windows.h"

#include "chrono"
using namespace std::chrono;

//#define EIGEN_VECTORIZE_SSE4_2
#include <Eigen/Dense>
using Eigen::MatrixXd;
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, 1> VectorXu;
typedef Eigen::Matrix<unsigned __int64, 1, Eigen::Dynamic> RowVectorXu;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;

high_resolution_clock::time_point beginTime;

using namespace std;

vector<double> errors;

std::random_device random;
std::mt19937_64 generator(random());
std::normal_distribution<> normal_random{ 0, 1 };
std::uniform_int_distribution<unsigned __int64> distribution;

duration<double> time_span;
void startCount() {
	//beginTime = GetTickCount();
	beginTime = high_resolution_clock::now();
}
void calculateInterval() {
	//elapsedTime = (GetTickCount() - beginTime);
	time_span = duration_cast<duration<double>>(high_resolution_clock::now() - beginTime);
	//cout << GetTickCount() - beginTime << "\t(ms)"
	//	<< " ( " << (double)(elapsedTime) / 1000 << "\t(s) )" << endl;
}

class PIDControllerOrigin  {

private:
	MatrixXd FG;
	MatrixXd HJ;
	MatrixXd x;
	MatrixXd u;

public:
	PIDControllerOrigin(MatrixXd FG, MatrixXd HJ, MatrixXd x_init) {
		this->FG = FG;
		this->HJ = HJ;
		this->x = x_init;
	}

	MatrixXd Step(MatrixXd y) {
		MatrixXd xy = mergeByRow(x, y);
		u = HJ * xy;
		x = FG * xy;
		return u;
	}
	MatrixXd mergeByRow(MatrixXd a, MatrixXd b) {
		MatrixXd result(a.rows() + b.rows(), a.cols());
		for (int i = 0;i < a.rows();i++) {
			for (int j = 0;j < a.cols();j++) {
				result(i, j) = a(i, j);
			}
		}
		for (int i = 0;i < b.rows();i++) {
			for (int j = 0;j < b.cols();j++) {
				result(a.rows() + i, j) = b(i, j);
			}
		}
		return result;
	}
};
class PIDQuantizedController {

private:
	MatrixXd FG;
	MatrixXd HJ;
	MatrixXd x;
	MatrixXd u;
	int r_y_inverse;

public:
	PIDQuantizedController(MatrixXd FG, MatrixXd HJ, MatrixXd x_init, int r_y_inverse) {
		this->FG = FG;
		this->HJ = HJ;
		this->x = x_init;
		this->r_y_inverse = r_y_inverse;
	}

	MatrixXd Step(MatrixXd y) {
		MatrixXd xy = mergeByRow(x, y);
		u = HJ * xy;
		x = FG * xy;
		return u;
	}
	MatrixXd mergeByRow(MatrixXd a, MatrixXd b) {
		MatrixXd result(a.rows() + b.rows(), a.cols());
		for (int i = 0;i < a.rows();i++) {
			for (int j = 0;j < a.cols();j++) {
				result(i, j) = a(i, j);
			}
		}
		for (int i = 0;i < b.rows();i++) {
			for (int j = 0;j < b.cols();j++) {
				result(a.rows() + i, j) = round(b(i, j) * r_y_inverse) / r_y_inverse;
			}
		}
		return result;
	}
};
class EncrypterDecrypter {
private:
	RowVectorXu secretKey;
	unsigned __int64 s; // secret key range
	unsigned __int64 q; // ciphertext modulus
	unsigned __int64 N; // plaintext modulus
	unsigned __int64 mod_e; // error range
	int n;
	int n_;

	int r_y_inverse;
	int L_inverse;
	//int r_dividedby_L;
	double r_dividedby_L;
	int s_1_inverse;
	int s_2_inverse;
	int M;
	int nu;
	int d;
	int logN;
	int logq;
	double security_level;

	unsigned __int64 mod_qe;
	unsigned __int64 q_dividedby_N;
	unsigned __int64 nu_;
	unsigned __int64 q_;
	unsigned __int64 N_;

public:
	EncrypterDecrypter(int r_y_inverse, int s_1_inverse, int s_2_inverse, int M, int L_inverse, int mod_e) {
		srand(time(NULL));

		n = 50; // ciphertext dimension
		n_ = n + 1;
		s = 256; // secret key range

		secretKey.resize(n);
		for (int i = 0;i < n;i++) {
			secretKey(i) = rand() % s;
		}

		this->r_y_inverse = r_y_inverse; // signal resolution
		this->L_inverse = L_inverse; // scaling factor for signal
		this->r_dividedby_L = (double)L_inverse / (double)r_y_inverse;
		this->s_1_inverse = s_1_inverse; // scaling factor for matrix G
		this->s_2_inverse = s_2_inverse; // scaling factor for matrix H and J
		this->M = ceil(M * r_dividedby_L);
		logN = ceil(log2(this->M) + log2(r_y_inverse) + log2(s_1_inverse) + log2(s_2_inverse));
		N = pow(2, logN); // plaintext modulus
		logq = logN;
		q = pow(2, logq); // ciphertext modulus
		nu = 16;
		d = ceil((double)logq / nu);
		this->mod_e = mod_e; // error range -> odd number

		cout << "parameters of the LWE cryptosystem" << endl;
		cout << "q=N=2^" << logq << ", nu=2^" << nu << ", d=" << d << ",\nn=" << n << ", sigma=" << (mod_e - 1) / 2 << endl;
		printf("---------------\n");
		cout << "parameters for quantization:" << endl;
		cout << "1/L=" << L_inverse << ", 1/s_1=" << s_1_inverse << ", 1/s_2=" << s_2_inverse << ", 1/r_y=" << r_y_inverse << endl;
		printf("---------------\n");

		mod_qe = q - (mod_e - 1) / 2;
		q_dividedby_N = pow(2, logq - logN);
		q_ = q - 1;
		N_ = N - 1;
		nu_ = pow(2, nu) - 1;
	}
	void Set_n(duration<double> currentTimeSpan, double T_s) {
		std::cout << "Time span test(by n=50) result: " << currentTimeSpan.count() << " seconds" << endl;
		n_ = floor(n_ * sqrt(T_s / 2 / max(0.0000001, currentTimeSpan.count())));
		n = n_ - 1;
		if (n < 1) {
			cout << "Impossible to implement since required n is too small(n < 1)" << endl;
			return;
		}
		else {
			cout << "new n=" << n << endl;
			printf("---------------\n");
			secretKey.resize(n);
			for (int i = 0;i < n;i++) {
				secretKey(i) = rand() % s;
			}
		}
	}
	void PrintSecurityLevel() {
		if (n <= 500) {
			security_level = 25.094 * pow(n, 0.0503);
		}
		else {
			security_level = 0.00009 * n * n - 0.0431 * n + 31.237;
			security_level += 3.0 * (48 - logq);
		}
		cout << "security level: rop=2^" << security_level << endl;
		printf("---------------\n");
	}
	MatrixXd Dec(MatrixXu c, int scaling, bool signal) {
		int l = c.rows() / n_;
		MatrixXd y(l, c.cols());

		for (int j = 0;j < c.cols();j++) {
			for (int i = 0;i < l;i++) {
				unsigned __int64 temp = c(i*n_, j);
				for (int k = 0;k < n;k++) {
					temp += secretKey(k) * c(i*n_ + k + 1, j);
				}
				unsigned __int64 m = (unsigned __int64)round(temp) & (N_);
				signed __int64 real_m = m - (m >= N / 2) * N;
				if (signal)
					y(i, j) = (double)real_m / (double)L_inverse / (double)scaling;
				else
					y(i, j) = real_m / (double)r_y_inverse / (double)scaling;
			}
		}
		return y;
	}
	MatrixXd Dec(MatrixXu c) {
		return Dec(c, 1, false);
	}

	MatrixXd Dec_signal(MatrixXu c) {
		return Dec(c, s_1_inverse*s_2_inverse, true);
	}

	MatrixXu Enc(MatrixXd m, bool signal) {
		MatrixXu c(n_*m.rows(), m.cols());

		for (int i_m = 0;i_m < m.rows();i_m++) {
			// a
			MatrixXu a(n, m.cols());

			for (int i_a = 0; i_a < n; i_a++) {
				for (int j_a = 0;j_a < m.cols();j_a++) {
					a(i_a, j_a) = distribution(generator) & q_;
					c(i_m*n_ + (i_a + 1), j_a) = a(i_a, j_a);
				}
			}

			// b
			VectorXu s_mult_a = multMxV(a, secretKey);
			for (int j_c = 0;j_c < m.cols();j_c++) {
				//int e = rand() % mod_e + mod_qe;
				int e = round(normal_random(generator));
				int quantized = round(m(i_m, j_c) * r_y_inverse);
				if (signal) {
					//signed __int64 scaled = (signed __int64)r_dividedby_L * (signed __int64)quantized;
					signed __int64 scaled = round(r_dividedby_L * quantized);
					c(i_m*n_, j_c) = (unsigned __int64)(q - s_mult_a(j_c) + (N + scaled) + e) & q_;
				}
				else
					c(i_m*n_, j_c) = (unsigned __int64)(q - s_mult_a(j_c) + (N + quantized) + e) & q_;
			}

		}
		return c;
	}
	MatrixXu Encm(MatrixXd m) {
		MatrixXu c = MatrixXu::Zero(n_ * m.rows(), n_ * m.cols() * d);
		MatrixXu m_temp(m.rows(), m.cols());
		for (int i = 0;i < m.rows(); i++) {
			for (int j = 0;j < m.cols();j++) {
				m_temp(i, j) = q + round(m(i, j));
			}
		}

		int piece_start = 0;
		MatrixXd zeros = MatrixXd::Zero(1, n_);
		MatrixXu enc_zeros;

		for (int piece = 0;piece < d;piece++) {
			for (int i_m = 0;i_m < m.rows();i_m++) {
				for (int j_m = 0;j_m < m.cols();j_m++) {
					enc_zeros = Enc(zeros, false);
					for (int i_c = 0;i_c < n_;i_c++) {
						int i_c_real = i_m * n_ + i_c;
						for (int j_c = 0;j_c < n_;j_c++) {
							int j_c_real = piece_start + j_m * n_ + j_c;
							c(i_c_real, j_c_real) = (c(i_c_real, j_c_real) + (i_c == j_c) * m_temp(i_m, j_m)) & q_;
							c(i_c_real, j_c_real) = (c(i_c_real, j_c_real) + enc_zeros(i_c, j_c)) & q_;
						}
					}

				}
			}

			for (int i_m = 0;i_m < m.rows(); i_m++) {
				for (int j_m = 0;j_m < m.cols();j_m++) {
					m_temp(i_m, j_m) = (m_temp(i_m, j_m) << nu) & q_;
				}
			}
			piece_start = piece_start + (n_)* m.cols();
		}

		return c;
	}
	MatrixXu splitm(MatrixXu c) {
		MatrixXu result(c.rows() * d, c.cols());
		MatrixXu c_temp(c.rows(), c.cols());
		c_temp = c;

		for (int piece = 0; piece < d; piece++) {
			for (int i = 0; i < c.rows(); i++) {
				for (int j = 0; j < c.cols(); j++) {
					result(piece * c.rows() + i, j) = c_temp(i, j) & nu_;
					c_temp(i, j) = c_temp(i, j) >> (nu);
				}
			}
		}
		return result;
	}
	MatrixXu multMxM(MatrixXu encm, MatrixXu split_enc) {
		int size_i = encm.rows();
		int size_k = split_enc.rows();
		int size_j = split_enc.cols();
		MatrixXu result(size_i, size_j);
		result.noalias() = encm * split_enc;

		for (int i = 0; i < size_i;i++)
			for (int j = 0;j < size_j;j++)
				result(i, j) &= q_;

		return result;
	}
	VectorXu multMxV(MatrixXu m, RowVectorXu vec) {
		int size_i = m.rows();
		int size_ii = m.cols();
		VectorXu y = VectorXu::Zero(size_ii);

		y = (vec * m);
		for (int i = 0;i < size_ii;i++)
			y(i) &= q_;

		return y;
	}
	MatrixXu scalar_mult(unsigned __int64 scalar, MatrixXu c) {
		MatrixXu result(c.rows(), c.cols());
		result = scalar * c;
		for (int i = 0; i < c.rows(); i++) {
			for (int j = 0; j < c.cols(); j++) {
				result(i, j) = result(i, j) & q_;
			}
		}
		return result;
	}
	MatrixXu add_enc(MatrixXu c1, MatrixXu c2) {
		MatrixXu result(c1.rows(), c1.cols());
		result = c1 + c2;
		for (int i = 0; i < c1.rows(); i++) {
			for (int j = 0; j < c1.cols(); j++) {
				result(i, j) = result(i, j) & q_;
			}
		}
		return result;
	}
	int Getq() {
		return logq;
	}
};
class PIDController {

private:
	MatrixXu encm_FG;
	MatrixXu encm_HJ;
	int logq = 48;
	unsigned __int64 q_ = pow(2, logq) - 1;
	const int nu = 16;
	const unsigned __int64 nu_ = pow(2, nu) - 1;
	int d = logq / nu;

public:
	MatrixXu enc_x;
	PIDController(MatrixXu encm_FG, MatrixXu encm_HJ, MatrixXu enc_x_init, int logq) {
		this->encm_FG = encm_FG;
		this->encm_HJ = encm_HJ;
		this->enc_x = enc_x_init;
		this->logq = logq;
		q_ = pow(2, logq) - 1;
		d = ceil((double)logq / nu);
	}
	
	MatrixXu Step(MatrixXu enc_y) {
		MatrixXu enc_xy = mergeByRow(enc_x, enc_y);
		MatrixXu split_enc_xy = splitm(enc_xy);
		startCount();

		MatrixXu enc_u = mult(encm_HJ, split_enc_xy);  // controller output
		enc_x = mult(encm_FG, split_enc_xy); // controller state

		calculateInterval();
		return enc_u;
	}
	MatrixXu mergeByRow(MatrixXu a, MatrixXu b) {
		MatrixXu result(a.rows() + b.rows(), a.cols());
		for (int i = 0;i < a.rows();i++) {
			for (int j = 0;j < a.cols();j++) {
				result(i, j) = a(i, j);
			}
		}
		for (int i = 0;i < b.rows();i++) {
			for (int j = 0;j < b.cols();j++) {
				result(a.rows() + i, j) = b(i, j);
			}
		}
		return result;
	}
	MatrixXu mult(MatrixXu encm, MatrixXu split_enc) {
		int size_i = encm.rows();
		int size_k = split_enc.rows();
		int size_j = split_enc.cols();
		MatrixXu result(size_i, size_j);
		result.noalias() = encm * split_enc;
		for (int i = 0; i < size_i;i++) {
			for (int j = 0;j < size_j;j++) {
				result(i, j) &= q_;
			}
		}
		return result;
	}
	MatrixXu splitm(MatrixXu c) {
		MatrixXu result(c.rows() * d, c.cols());
		MatrixXu c_temp(c.rows(), c.cols());
		for (int i = 0;i < c.rows(); i++) {
			for (int j = 0;j < c.cols();j++) {
				c_temp(i, j) = c(i, j);
			}
		}

		for (int piece = 0;piece < d;piece++) {
			for (int i = 0;i < c.rows();i++) {
				for (int j = 0;j < c.cols();j++) {
					result(piece * c.rows() + i, j) = c_temp(i, j) & nu_;
					c_temp(i, j) = c_temp(i, j) >> (nu);
				}
			}
		}
		return result;
	}
};

class Plant {

private:
	MatrixXd FG;
	MatrixXd HJ;
	MatrixXd x;
	MatrixXd x_quan;
	MatrixXd x_prime;
	MatrixXd u;
	MatrixXd u_quan;
	MatrixXd u_prime;
	MatrixXd y;
	MatrixXd y_quan;
	MatrixXd y_prime;
	MatrixXd r;
	int s_1_inverse;
	int s_2_inverse;
	int r_y_inverse;
	int r_u_inverse;
	int M;
	int L_inverse;
	int mod_e = 3;
	double T_s;
	PIDController *controller;
	PIDControllerOrigin *controllerOrigin;
	PIDQuantizedController *quantizedController;
	EncrypterDecrypter *encdec;

	string filePath = "parameters.txt";
	string writeFilePath = "result.txt";
	ifstream ifs;
	ofstream ofs;

public:
	Plant() {
		ifs.open(filePath.data());
		if (!ifs.is_open()) {
			cout << "No input file" << endl;
			return;
		}
		string line;
		getline(ifs, line, '\n');
		FG = readMatrix("CD");
		HJ = readMatrix("x");
		x = readMatrix("u");
		x_prime = x;
		x_quan = x;
		u = readMatrix("y");
		u_prime = u;
		u_quan = u;
		y = readMatrix("r");
		y_prime = y;
		y_quan = y;
		r = readMatrix("PID");
		getline(ifs, line, '\n');
		size_t pos = 0;
		size_t point_pos;
		pos = line.find('\t');
		double k_p = stod(line.substr(0, pos));
		int k_p_decimal;
		point_pos = line.substr(0, pos).find('.');
		if (point_pos == string::npos)
			k_p_decimal = 0;
		else
			k_p_decimal = pos - (point_pos + 1);
		line.erase(0, pos + 1);
		pos = line.find('\t');
		double k_i = stod(line.substr(0, pos));
		int k_i_decimal;
		point_pos = line.substr(0, pos).find('.');
		if (point_pos == string::npos)
			k_i_decimal = 0;
		else
			k_i_decimal = pos - (point_pos + 1);
		line.erase(0, pos + 1);
		pos = line.find('\t');
		double k_d = stod(line.substr(0, pos));
		int k_d_decimal;
		point_pos = line.substr(0, pos).find('.');
		if (point_pos == string::npos)
			k_d_decimal = 0;
		else
			k_d_decimal = pos - (point_pos + 1);
		line.erase(0, pos + 1);
		pos = line.find('\t');
		int N_d = stoi(line.substr(0, pos));
		line.erase(0, pos + 1);
		T_s = stod(line);
		int T_s_decimal;
		point_pos = line.find('.');
		if (point_pos == string::npos)
			T_s_decimal = 0;
		else
			T_s_decimal = line.length() - (point_pos + 1);

		getline(ifs, line, '\n');
		getline(ifs, line, '\n');
		pos = line.find('\t');
		string token = line.substr(0, pos);
		r_y_inverse = stoi(token);
		line.erase(0, pos + 1);
		token = line;
		r_u_inverse = stoi(token);

		ifs.close();

		printf("plant initial state x=\n");
		cout << x << endl;
		printf("plant initial output y=\n");
		cout << y << endl;
		printf("reference signal r=\n");
		cout << r << endl;
		printf("plant matrix AB=\n");
		cout << FG << endl;
		printf("plant matrix CD=\n");
		cout << HJ << endl;
		cout << "PID parameters:\n" << "k_p=" << k_p << ", k_i=" << k_i << ", k_d=" << k_d << ", N_d=" << N_d << ", T_s=" << T_s << endl;
		printf("---------------\n");

		int mod_e = 3;

		//cout << "delta_Enc: " << delta_Enc() << ", delta_Mult: " << delta_Mult() << endl;
		//printf("---------------\n");

		buildPIDController(k_p, k_i, k_d, N_d, T_s, k_p_decimal, k_i_decimal, k_d_decimal, T_s_decimal);
	}

	void buildPIDController(double k_p, double k_i, double k_d, int N_d, double T_s, int k_p_decimal, int k_i_decimal, int k_d_decimal, int T_s_decimal) {
		// s_1_inverse: scaling factor for controller matrix G
		s_1_inverse = 1;
		// s_2_inverse: scaling factor for controller matrices H and J
		s_2_inverse = pow(10, max(k_i_decimal + T_s_decimal, k_p_decimal, k_d_decimal + T_s_decimal));
		M = pow(2, 11); // plaintext range for signal y, u, and controller state x
		
		Eigen::Matrix<double, 2, 2> F;
		F << 2 - (double)N_d, (double)N_d - 1,
			1, 0;
		Eigen::Matrix<double, 2, 1> G;
		G << 1,
			0;
		int x_row = F.rows();
		int y_row = G.cols();
		MatrixXd FG = MatrixXd(x_row, x_row + y_row);
		for (int row = 0;row < x_row;row++) {
			for (int col = 0;col < x_row;col++) {
				FG(row, col) = F(row, col);
			}
			for (int col = x_row;col < x_row + y_row;col++) {
				FG(row, col) = G(row, col - x_row);
			}
		}
		MatrixXd FG_scaled = MatrixXd(x_row, x_row + y_row);
		for (int row = 0;row < x_row;row++) {
			for (int col = 0;col < x_row;col++) {
				FG_scaled(row, col) = FG(row, col);
			}
			for (int col = x_row;col < x_row + y_row;col++) {
				FG_scaled(row, col) = round(FG(row, col) * s_1_inverse);
			}
		}

		Eigen::Matrix<double, 1, 2> H;
		H << k_i * T_s - k_d * N_d * N_d / T_s, k_i * T_s * N_d - k_i * T_s + k_d * N_d * N_d / T_s;
		Eigen::Matrix<double, 1, 1> J;
		J << k_p + k_d * N_d / T_s;
		int u_row = H.rows();
		MatrixXd HJ = MatrixXd(u_row, x_row + y_row);
		for (int row = 0;row < u_row;row++) {
			for (int col = 0;col < x_row;col++) {
				HJ(row, col) = H(row, col);
			}
			for (int col = x_row;col < x_row + y_row;col++) {
				HJ(row, col) = J(row, col - x_row);
			}
		}
		MatrixXd HJ_scaled = MatrixXd(u_row, x_row + y_row);
		for (int row = 0;row < u_row;row++) {
			for (int col = 0;col < x_row;col++) {
				HJ_scaled(row, col) = round(HJ(row, col) * s_2_inverse);
			}
			for (int col = x_row;col < x_row + y_row;col++) {
				HJ_scaled(row, col) = round(HJ(row, col) * s_2_inverse * s_1_inverse);
			}
		}

		int n = 450; // change

		cout << "delta_Enc: "<<delta_Enc()<<" delta_Mult: "<< delta_Mult(n) << endl;

		double G_norm = getInfiniteNorm(G);
		double J_norm = getInfiniteNorm(J);
		int degrade_bound = 6;
		while (true) {
			double L1 = (G_norm * delta_Enc() + (x.rows() + y.rows()) * delta_Mult(n) / s_1_inverse) / (double)(r_y_inverse / 2 * (G_norm)) * pow(10, degrade_bound);
			double L2 = (J_norm * delta_Enc() + (x.rows() + y.rows()) * delta_Mult(n) / s_1_inverse / (double)s_2_inverse) / (r_y_inverse / 2 * J_norm + r_u_inverse / 2) * pow(10, degrade_bound);
			double L3 = 2 * delta_Enc() * pow(10, degrade_bound);
			L_inverse = pow(2, ceil(log2(max(r_y_inverse, max(max(L1, L2), L3)))));
			cout << "(Beta1 - Alpha1) / Alpha1 = " << (G_norm * delta_Enc() / L_inverse + (x.rows() + y.rows()) * delta_Mult(n) / L_inverse / s_1_inverse) / (double)(r_y_inverse / 2 * (G_norm)) << endl;
			cout << "(Beta2 - Alpha2) / Alpha2 = " << (J_norm * delta_Enc() / L_inverse + (x.rows() + y.rows()) * delta_Mult(n) / L_inverse / s_1_inverse / (double)s_2_inverse) / (r_y_inverse / 2 * (J_norm + 1)) << endl;
			cout << "(Beta3 - Alpha3) / Alpha3 = " << 2 * (double)delta_Enc() / L_inverse << endl;
			cout << "Alpha1 = " << (double)((G_norm)/r_y_inverse/2) << endl;
			cout << "Alpha2 = " << (double)(J_norm + 1) / s_2_inverse / r_y_inverse / 2 << endl;
			cout << "Alpha3 = " << 0.5 / s_1_inverse << endl;

			encdec = new EncrypterDecrypter(r_y_inverse, s_1_inverse, s_2_inverse, M, L_inverse, mod_e);
			if (encdec->Getq() > 48)
				degrade_bound--;
			else
				break;
		}

		MatrixXu encm_FG = encdec->Encm(FG_scaled);
		MatrixXu encm_HJ = encdec->Encm(HJ_scaled);

		Eigen::Matrix<double, 2, 1> x_init_con;
		x_init_con << 0, 
			0;

		MatrixXu enc_x_init_con = encdec->Enc(x_init_con, false);

		controllerOrigin = new PIDControllerOrigin(FG, HJ, x_init_con);

		quantizedController = new PIDQuantizedController(FG, HJ, x_init_con, r_y_inverse);

		controller = new PIDController(encm_FG, encm_HJ, enc_x_init_con, encdec->Getq());

		controller->Step(encdec->Enc(Substraction(r, y), true));
		encdec->Set_n(time_span, T_s);
		encdec->PrintSecurityLevel();

		encm_FG = encdec->Encm(FG_scaled);
		encm_HJ = encdec->Encm(HJ_scaled);

		enc_x_init_con = encdec->Enc(x_init_con, false);

		controller = new PIDController(encm_FG, encm_HJ, enc_x_init_con, encdec->Getq());
	}

	void getOutput() {
		MatrixXd xu = mergeByRow(x, u);
		y = HJ * xu; // plant output
		MatrixXd xu_prime = mergeByRow(x_prime, u_prime);
		y_prime = HJ * xu_prime;
		MatrixXd xu_quan = mergeByRow(x_quan, u_quan);
		y_quan = HJ * xu_quan;
	}
	void updateState() {
		MatrixXd xu = mergeByRow(x, u);
		x = FG * xu; // plant state
		MatrixXd xu_prime = mergeByRow(x_prime, u_prime);
		x_prime = FG * xu_prime;
		MatrixXd xu_quan = mergeByRow(x_quan, u_quan);
		x_quan = FG * xu_quan;
	}
	void loop() {
		//startCount();
		ofs.open(writeFilePath.data());
		if (!ofs.is_open()) {
			cout << "No output file" << endl;
			return;
		}
		for (int t = 0;t <= 120 / T_s;t++) {
			getOutput();
			if (t % 100 == 0 && t > 0) {
				cout << "step=" << t << endl;
				cout << "r - y_prime=\t\t(reference signal vs. original system)" << endl;
				cout << (Substraction(r, y_prime)) << endl;
				cout << "y_quan - y_prime=\t(quantized system vs. original system)" << endl;
				cout << (Substraction(y_quan, y_prime)) << endl;
				cout << "y - y_prime=\t\t(encrypted system vs. original system)" << endl;
				cout << (Substraction(y, y_prime)) << endl;
				/*cout << "u - u_prime=\t(error between u of encrypted system and u_prime of original system)" << endl;
				cout << (Substraction(u, u_prime)) << endl;*/
				ofs << (int)(T_s*t) << " " << (Substraction(r, y_prime)) << " " << (Substraction(r, y)) << endl;
			}
			u = truncateMatrix(encdec->Dec_signal(controller->Step(encdec->Enc(Substraction(r, y), true))), r_u_inverse);
			u_prime = truncateMatrix(controllerOrigin->Step(Substraction(r, y_prime)), r_u_inverse);
			u_quan = truncateMatrix(quantizedController->Step(Substraction(r, y_quan)), r_u_inverse);
			updateState();
		}
		errors.push_back(Substraction(y, y_prime)(0, 0));
		ofs.close();
		//calculateInterval();
	}

	MatrixXd mergeByRow(MatrixXd a, MatrixXd b) {
		MatrixXd result(a.rows() + b.rows(), a.cols());
		for (int i = 0;i < a.rows();i++) {
			for (int j = 0;j < a.cols();j++) {
				result(i, j) = a(i, j);
			}
		}
		for (int i = 0;i < b.rows();i++) {
			for (int j = 0;j < b.cols();j++) {
				result(a.rows() + i, j) = b(i, j);
			}
		}
		return result;
	}
	MatrixXd truncateMatrix(MatrixXd mtx, int truncator) {
		MatrixXd result(mtx.rows(), mtx.cols());
		for (int i = 0;i < mtx.rows(); i++) {
			for (int j = 0;j < mtx.cols();j++) {
				result(i, j) = round(mtx(i, j) * truncator) / truncator;
			}
		}
		return result;
	}
	MatrixXd Substraction(MatrixXd mtx_left, MatrixXd mtx_right) {
		MatrixXd result(mtx_left.rows(), mtx_left.cols());
		for (int i = 0;i < mtx_left.rows(); i++) {
			for (int j = 0;j < mtx_left.cols();j++) {
				result(i, j) = mtx_left(i, j) - mtx_right(i, j);
			}
		}
		return result;
	}
	double getInfiniteNorm(MatrixXd m) {
		double max = 0;
		for (int i = 0;i < m.rows();i++) {
			double sum = 0;
			for (int j = 0;j < m.cols();j++) {
				sum += abs(m(i, j));
			}
			if (sum > max)
				max = sum;
		}
		return max;
	}
	int delta_Enc() {
		return round((mod_e - 1) / 2);
	}
	int delta_Mult(int n) {
		int d = 3;
		int nu = 16;
		return round(d*(n + 1)*(mod_e - 1) / 2 * nu) + 1;
	}
	MatrixXd readMatrix(string end) {
		vector<vector<double>> m_;
		string line;
		int i = 0;
		while (getline(ifs, line, '\n')) {
			if (line.compare(end) == 0)
				break;
			m_.push_back(vector<double>());
			size_t pos = 0;
			string token;
			while ((pos = line.find('\t')) != std::string::npos) {
				token = line.substr(0, pos);
				m_[i].push_back(stod(token));
				line.erase(0, pos + 1);
			}
			token = line;
			m_[i].push_back(stod(token));
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
	EncrypterDecrypter *encdec = new EncrypterDecrypter(100, 1, 1, 16384, 1, 7);

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
	MatrixXu A_enc_mult_scalar = encdec->scalar_mult(3, A_enc);
	MatrixXu A_enc_add_B_enc = encdec->add_enc(A_enc, B_enc);
	cout << "Enc(A):\n";
	cout << (A_enc) << endl;
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
	EncrypterDecrypter *encdec = new EncrypterDecrypter(1, 1, 1, 16384, 1, 7);

	int AB_row = 2;
	int AB_col = 3;
	MatrixXd FG(2, 3);
	FG << -4, 3, 1,
		4, 11, 0;
	int xy_row = AB_col;
	int xy_col = 2;
	MatrixXd xu(3, 2);
	xu << -5, 1,
		-2.5, 0.125,
		2, 0;

	printf("AB:\n");
	cout << (FG) << endl;
	printf("xu:\n");
	cout << (xu) << endl;

	MatrixXu encm_FG = encdec->Encm(FG);
	MatrixXu enc_xu = encdec->Enc(xu, false);
	MatrixXu split_enc_xu = encdec->splitm(enc_xu);
	MatrixXu enc_mult = encdec->multMxM(encm_FG, split_enc_xu);
	MatrixXd answer = encdec->Dec(enc_mult);
	printf("Dec(Enc(AB) x Enc(xu)):\n");
	cout << (answer) << endl;
}

int main()
{
#if defined(__LP64__)
	printf("LP64\n");
	// LP64 machine, OS X or Linux
#elif defined(_WIN64)
	printf("Win64\n");
	// LLP64 machine, Windows
#else
	printf("32-bit\n");
#endif
	srand(time(NULL));
	//LWECheck();
	//GSWCheck();
	int step = 1;
	double total = 0;
	for (int i = 0;i < step;i++) {
		Plant* plant = new Plant();
		plant->loop();
		total += abs(errors.back());
	}
	cout << (total/(double)step) << endl;
	return 0;
}
