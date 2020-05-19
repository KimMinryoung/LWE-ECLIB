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

#define EIGEN_VECTORIZE_SSE4_2
#include <Eigen/Dense>
using Eigen::MatrixXd;
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;
typedef Eigen::Matrix<unsigned __int64, Eigen::Dynamic, 1> VectorXu;
typedef Eigen::Matrix<unsigned __int64, 1, Eigen::Dynamic> RowVectorXu;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;
typedef Eigen::Matrix<double, 2, 1> Vector2d;

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

class OriginalController  {

private:
	MatrixXd FGR;
	MatrixXd HJ;
	MatrixXd x;
	MatrixXd u;

public:
	OriginalController(MatrixXd FGR, MatrixXd HJ, MatrixXd x_init) {
		this->FGR = FGR;
		this->HJ = HJ;
		this->x = x_init;
	}

	MatrixXd Step(MatrixXd y) {
		MatrixXd xy = MergeByRow(x, y);
		u = HJ * xy;
		MatrixXd xyu = MergeByRow(xy, u);
		x = FGR * xyu;
		return u;
	}
	MatrixXd MergeByRow(MatrixXd a, MatrixXd b) {
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
class QuantizedController {

private:
	MatrixXd FGR;
	MatrixXd HJ;
	MatrixXd x;
	MatrixXd u;
	int r_y_inverse;
	int r_u_inverse;

public:
	QuantizedController(MatrixXd FGR, MatrixXd HJ, MatrixXd x_init, int r_y_inverse, int r_u_inverse) {
		this->FGR = FGR;
		this->HJ = HJ;
		this->x = x_init;
		this->r_y_inverse = r_y_inverse;
		this->r_u_inverse = r_u_inverse;
	}

	MatrixXd Step(MatrixXd y) {
		MatrixXd xy = MergeByRow(x, y, r_y_inverse);
		u = HJ * xy;
		MatrixXd xyu = MergeByRow(xy, u, r_u_inverse);
		x = FGR * xyu;
		return u;
	}
	MatrixXd MergeByRow(MatrixXd a, MatrixXd b, int quantFactor) {
		MatrixXd result(a.rows() + b.rows(), a.cols());
		for (int i = 0;i < a.rows();i++)
			for (int j = 0;j < a.cols();j++)
				result(i, j) = a(i, j);
		for (int i = 0;i < b.rows();i++)
			for (int j = 0;j < b.cols();j++)
				result(a.rows() + i, j) = round(b(i, j) * quantFactor) / quantFactor;
		return result;
	}
};
class EncrypterDecrypter {
private:
	RowVectorXu secretKey;
	unsigned __int64 s; // secret key range
	unsigned __int64 q; // ciphertext modulus
	unsigned __int64 N; // plaintext modulus
	int error_range; // error_range=3 -> noise = -1,0,1
	int n;
	int n_;

	int r_y_inverse;
	int L_inverse;
	//int r_dividedby_L;
	double r_dividedby_L;
	int s_1_inverse;
	int s_2_inverse;
	int U;
	int nu;
	int d;
	int logN;
	int logq;
	double security_level;

	unsigned __int64 q_dividedby_N;
	unsigned __int64 nu_;
	unsigned __int64 q_;
	unsigned __int64 N_;

public:
	EncrypterDecrypter(int r_y_inverse, int s_1_inverse, int s_2_inverse, int U, int L_inverse, int error_range, int n) {
		srand(time(NULL));

		if (n == -1)
			this->n = 50; // temporal ciphertext dimension for speed measuring
		else
			this->n = n;
		this->n_ = this->n + 1;
		s = 256; // secret key range

		// n elements of the secret key are randomly chosen in range s
		secretKey.resize(this->n);
		for (int i = 0;i < this->n;i++) {
			secretKey(i) = rand() % s;
		}

		this->r_y_inverse = r_y_inverse; // resolution of controller input signal y
		this->L_inverse = L_inverse; // scaling factor for signal y
		this->r_dividedby_L = (double)L_inverse / (double)r_y_inverse;
		this->s_1_inverse = s_1_inverse; // scaling factor for matrix G
		this->s_2_inverse = s_2_inverse; // scaling factor for matrix H and J
		this->U = (int)ceil(U * r_dividedby_L); // range of u
		logN = ceil(log2(this->U) + log2(r_y_inverse) + log2(s_1_inverse) + log2(s_2_inverse));
		N = (unsigned __int64)pow(2, logN); // plaintext modulus
		logq = logN; // q:=N
		q = (unsigned __int64)pow(2, logq); // ciphertext modulus
		nu = 16;
		d = (int)ceil((double)logq / nu);
		this->error_range = error_range; // error range must be odd number(ex: error_range=3 -> e=-1,0,1)

		if (n != -1) {
			cout << "parameters of the LWE cryptosystem" << endl;
			cout << "q=N=2^" << logq << ", nu=2^" << nu << ", d=" << d << ",\nn=" << this->n << ", sigma=" << (error_range - 1) / 2 << endl;
			printf("---------------\n");
			cout << "parameters for quantization:" << endl;
			cout << "1/L=" << L_inverse << ", 1/s_1=" << s_1_inverse << ", 1/s_2=" << s_2_inverse << ", 1/r_y=" << r_y_inverse << ", U=" << U << endl;
			printf("---------------\n");
		}
		PrintSecurityLevel();

		q_dividedby_N = pow(2, logq - logN);
		q_ = q - 1; // q_ and N_ are for bitwise operations which substitutes modulus operations
		N_ = N - 1;
		nu_ = (unsigned __int64)pow(2, nu) - 1;
	}
	int Set_n(duration<double> currentTimeSpan, double T_s) {
		std::cout << "Time span test(by n=" << n << ") result: " << currentTimeSpan.count() << " seconds" << endl;
		n_ = (int)floor(n_ * sqrt(T_s / 2 / max(0.0000001, currentTimeSpan.count())));
		n = n_ - 1;
		if (n < 1) {
			cout << "Impossible to implement since T_s is too small" << endl;
			return -1;
		}
		else {
			cout << "new n=" << n << endl;
			printf("---------------\n");
			secretKey.resize(n);
			for (int i = 0;i < n;i++) {
				secretKey(i) = rand() % s;
			}
		}
		return n;
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
				unsigned __int64 m = temp & N_;
				signed __int64 real_m = m - (m >= N / 2) * N;
				if (signal)
					y(i, j) = (double)real_m / (double)L_inverse / (double)scaling;
				else
					y(i, j) = (double)real_m / (double)r_y_inverse / (double)scaling;
			}
		}
		return y;
	}
	// Decrypt non-scaled matrix(not used for control system)
	MatrixXd Dec(MatrixXu c) {
		return Dec(c, 1, false);
	}
	// Decrypt control signal and auto-rescale
	MatrixXd Dec_u(MatrixXu c) {
		return Dec(c, s_1_inverse*s_2_inverse, true);
	}
	// Decrypt control state and auto-rescale
	MatrixXd Dec_state(MatrixXu x) {
		return Dec(x, s_1_inverse, true);
	}

	MatrixXu Enc(MatrixXd m, bool signal) {
		MatrixXu c(n_*m.rows(), m.cols());

		for (int i_m = 0;i_m < m.rows();i_m++) {
			// a
			MatrixXu a(n, m.cols());

			for (int i_a = 0; i_a < n; i_a++) { // vector a's length=n
				for (int j_a = 0;j_a < m.cols();j_a++) {
					// uniformly randomly generate a entry
					a(i_a, j_a) = distribution(generator) & q_;
					c(i_m*n_ + (i_a + 1), j_a) = a(i_a, j_a);
				}
			}

			// b
			VectorXu s_mult_a = MultMxV(a, secretKey); // a*sk

			for (int j_c = 0; j_c < m.cols(); j_c++) {
				double _noise = normal_random(generator); // _noise ~ N(0,1)
				int noise = min((error_range - 1) / 2, max((1 - error_range) / 2, round(_noise))); // noise=-1,0,1
				unsigned __int64 e = q + noise; // To make noise positive number, Add q
				// b=(e+m-a*sk) mod q
				signed __int64 quantized_m = (signed __int64)round(m(i_m, j_c) * r_y_inverse);
				if (signal) { // If m is signal, scale by L
					signed __int64 scaled_m = (signed __int64)(r_dividedby_L * (double)quantized_m);
					c(i_m*n_, j_c) = (unsigned __int64)(e + scaled_m - s_mult_a(j_c)) & q_;
				}
				else { // Else, scale by r_y
					c(i_m*n_, j_c) = (unsigned __int64)(e + quantized_m - s_mult_a(j_c)) & q_;
				}
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
							c(i_c_real, j_c_real) = (c(i_c_real, j_c_real) + (i_c == j_c) * m_temp(i_m, j_m) + enc_zeros(i_c, j_c)) & q_;
						}
					}
				}
			}
			for (int i_m = 0;i_m < m.rows(); i_m++) {
				for (int j_m = 0;j_m < m.cols();j_m++) {
					m_temp(i_m, j_m) = (m_temp(i_m, j_m) << nu) & q_;
				}
			}
			piece_start += n_ * m.cols();
		}
		return c;
	}
	MatrixXu SplitMtx(MatrixXu m) {
		MatrixXu result(m.rows() * d, m.cols());
		MatrixXu c_temp(m.rows(), m.cols());
		c_temp = m;

		for (int piece = 0; piece < d; piece++) {
			for (int i = 0; i < m.rows(); i++) {
				for (int j = 0; j < m.cols(); j++) {
					result(piece * m.rows() + i, j) = c_temp(i, j) & nu_;
					c_temp(i, j) = c_temp(i, j) >> (nu);
				}
			}
		}
		return result;
	}
	MatrixXu MultMxM(MatrixXu encm, MatrixXu split_enc) {
		int size_i = encm.rows();
		int size_k = split_enc.rows();
		int size_j = split_enc.cols();
		MatrixXu result(size_i, size_j);
		//MatrixXu result = MatrixXu::Zero(size_i, size_j);
		result.noalias() = encm * split_enc;

		for (int i = 0; i < size_i;i++)
			for (int j = 0; j < size_j; j++) {
				//cout << result(i, j) <<" ";
				result(i, j) &= q_;
				//cout << result(i, j) << endl;
			}
		
		/*for (int i = 0; i < size_i;i++)
			for (int j = 0;j < size_j;j++)
				for (int k = 0; k < size_k; k++) {
					result(i, j) = (result(i, j) + encm(i, k) * split_enc(k, j)) & q_;
					cout << k<<":"<<result(i, j) << endl;
				}*/

		return result;
	}
	VectorXu MultMxV(MatrixXu m, RowVectorXu vec) {
		int size_i = m.rows();
		int size_ii = m.cols();
		VectorXu y = VectorXu::Zero(size_ii);

		y = (vec * m);
		for (int i = 0;i < size_ii;i++)
			y(i) &= q_;
		/*for (int i = 0; i < size_ii;i++)
			for (int k = 0;k < size_i;k++)
				y(i) = (y(i) + vec(k) * m(k, i)) & q_;
				*/
		return y;
	}
	MatrixXu ScalarMult(unsigned __int64 scalar, MatrixXu c) {
		MatrixXu result(c.rows(), c.cols());
		result = scalar * c;
		for (int i = 0; i < c.rows(); i++) {
			for (int j = 0; j < c.cols(); j++) {
				result(i, j) = result(i, j) & q_;
			}
		}
		return result;
	}
	MatrixXu Add(MatrixXu c1, MatrixXu c2) {
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
class EncryptedController {

private:
	MatrixXu encm_FGR;
	MatrixXu encm_HJ;
	MatrixXu enc_xy;
	int logq = 48;
	unsigned __int64 q_ = pow(2, logq) - 1;
	const int nu = 16;
	const unsigned __int64 nu_ = pow(2, nu) - 1;
	int d = logq / nu;

public:
	MatrixXu enc_x;
	EncryptedController(MatrixXu encm_FGR, MatrixXu encm_HJ, MatrixXu enc_x_init, int logq) {
		this->encm_FGR = encm_FGR;
		this->encm_HJ = encm_HJ;
		this->enc_x = enc_x_init;
		this->logq = logq;
		q_ = pow(2, logq) - 1;
		d = ceil((double)logq / nu);
	}
	
	MatrixXu GetOutput(MatrixXu enc_y) { // calculate and send u to plant
		enc_xy = MergeByRow(enc_x, enc_y);
		MatrixXu split_enc_xy = SplitMtx(enc_xy);
		startCount();
		MatrixXu enc_u = MultMxM(encm_HJ, split_enc_xy);  // controller output
		return enc_u;
	}
	MatrixXu UpdateState(MatrixXu enc_y, MatrixXu enc_u_prime) { // enc_u_prime: rearrived version of output u(has the same scale with signal y)
		MatrixXu enc_xyu = MergeByRow(enc_xy, enc_u_prime);
		MatrixXu split_enc_xyu = SplitMtx(enc_xyu);
		enc_x = MultMxM(encm_FGR, split_enc_xyu); // controller state
		calculateInterval();
		return enc_x;
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
	MatrixXu MultMxM(MatrixXu encm, MatrixXu split_enc) {
		int size_i = encm.rows();
		int size_k = split_enc.rows();
		int size_j = split_enc.cols();
		MatrixXu result(size_i, size_j);
		result.noalias() = encm * split_enc;
		for (int i = 0; i < size_i;i++)
			for (int j = 0;j < size_j;j++)
				result(i, j) &= q_;
		
		/*for (int i = 0; i < size_i;i++)
		for (int j = 0;j < size_j;j++)
		for (int k = 0;k < size_k;k++)
		result(i, j) = (result(i, j) + encm(i, k) * split_enc(k, j)) & q_;*/
		return result;
	}
	MatrixXu SplitMtx(MatrixXu m) {
		MatrixXu result(m.rows() * d, m.cols());
		MatrixXu c_temp(m.rows(), m.cols());
		for (int i = 0;i < m.rows(); i++) {
			for (int j = 0;j < m.cols();j++) {
				c_temp(i, j) = m(i, j);
			}
		}

		for (int piece = 0;piece < d;piece++) {
			for (int i = 0;i < m.rows();i++) {
				for (int j = 0;j < m.cols();j++) {
					result(piece * m.rows() + i, j) = c_temp(i, j) & nu_;
					c_temp(i, j) = c_temp(i, j) >> (nu);
				}
			}
		}
		return result;
	}
};

class Plant {

private:
	MatrixXd AB;
	MatrixXd CD;
	MatrixXd F;
	MatrixXd G;
	MatrixXd R;
	MatrixXd H;
	MatrixXd J;
	MatrixXd x; // controller state of encrypted system
	MatrixXd x_quan;
	MatrixXd x_prime;
	MatrixXd u;
	MatrixXd u_quan;
	MatrixXd u_prime;
	MatrixXd y; // plant output of encrypted system
	MatrixXd y_quan; // plant output of quantized system
	MatrixXd y_prime; // plant output of original system
	MatrixXd r;
	int s_1_inverse;
	int s_2_inverse;
	int r_y_inverse;
	int r_u_inverse;
	int U;
	int L_inverse;
	int error_range = 3;
	double T_s;
	EncryptedController *controller;
	OriginalController *controllerOrigin;
	QuantizedController *quantizedController;
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
		AB = ReadMatrix("CD");
		CD = ReadMatrix("F");
		F = ReadMatrix("G");
		int F_precision = needed_precision;
		G = ReadMatrix("H");
		int G_precision = needed_precision;
		H = ReadMatrix("J");
		int H_precision = needed_precision;
		J = ReadMatrix("x");
		int J_precision = needed_precision;
		x = ReadMatrix("u");
		x_prime = x;
		x_quan = x;
		u = ReadMatrix("y");
		u_prime = u;
		u_quan = u;
		y = ReadMatrix("r");
		y_prime = y;
		y_quan = y;
		r = ReadMatrix("T_s");
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

		ifs.close();

		cout << ">> plant <<" << endl;
		cout << "x+ = AB * xy" << endl;
		cout << "y = CD * xy" << endl;
		printf("plant initial state x=\n");
		cout << x << endl;
		printf("plant initial output y=\n");
		cout << y << endl;
		printf("reference signal r=\n");
		cout << r << endl;
		printf("plant matrix AB=\n");
		cout << AB << endl;
		printf("plant matrix CD=\n");
		cout << CD << endl;
		cout << "sampling time:\n" "T_s=" << T_s << endl;
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
		// TODO: fix scaling factor decision
		// s_1_inverse: scaling factor for controller matrix G and R
		s_1_inverse = pow(10, max(H_precision + G_precision + F_precision, 2 * F_precision + J_precision));
		// s_2_inverse: scaling factor for controller matrices H and J
		s_2_inverse = pow(10, J_precision);

		int x_row = F.rows();
		int y_row = G.cols();
		int u_row = H.rows(); // u_row should be 1
		VectorXd a_(x_row);
		VectorXd b_1(x_row);
		double b_0;

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

		// temporal encdec and controller for computing time simulation
		encdec = new EncrypterDecrypter(r_y_inverse, s_1_inverse, s_2_inverse, U, r_y_inverse, error_range, -1);
		MatrixXu encm_FGR = encdec->Encm(FGR_scaled);
		MatrixXu encm_HJ = encdec->Encm(HJ_scaled);
		MatrixXu enc_x_init_con = encdec->Enc(x_init_con, false);
		controller = new EncryptedController(encm_FGR, encm_HJ, enc_x_init_con, encdec->Getq());
		MatrixXu __u = controller->GetOutput(encdec->Enc(Substraction(r, y), true));
		controller->UpdateState(encdec->Enc(Substraction(r, y), true), __u);
		int n = encdec->Set_n(time_span, T_s);

		cout << "delta_Enc: "<<DeltaEnc()<<" delta_Mult: "<< DeltaMult(n) << endl;

		double G_norm = GetInfiniteNorm(G);
		double R_norm = GetInfiniteNorm(R);
		double J_norm = GetInfiniteNorm(J);
		int degrade_bound = 6;
		while (true) {
			double L1 = ((G_norm + R_norm) * DeltaEnc() + (x_row + y_row + u_row) * DeltaMult(n) / s_1_inverse) / (double)(r_y_inverse / 2 * (G_norm + R_norm)) * pow(10, degrade_bound);
			double L2 = (J_norm * DeltaEnc() + (x_row + y_row) * DeltaMult(n) / (double)s_1_inverse / (double)s_2_inverse) / (r_y_inverse / 2 * J_norm + r_u_inverse / 2) * pow(10, degrade_bound);
			double L3 = 2 * DeltaEnc() * pow(10, degrade_bound);
			L_inverse = pow(2, ceil(log2(max(r_y_inverse, max(max(L1, L2), L3)))));

			encdec = new EncrypterDecrypter(r_y_inverse, s_1_inverse, s_2_inverse, U, L_inverse, error_range, n);
			if (encdec->Getq() > 48)
				degrade_bound--;
			else
				break;
		}

		encm_FGR = encdec->Encm(FGR_scaled);
		encm_HJ = encdec->Encm(HJ_scaled);
		enc_x_init_con = encdec->Enc(x_init_con, true);

		controllerOrigin = new OriginalController(FGR, HJ, x_init_con);
		quantizedController = new QuantizedController(FGR, HJ, x_init_con, r_y_inverse, r_u_inverse);
		controller = new EncryptedController(encm_FGR, encm_HJ, enc_x_init_con, encdec->Getq());
	}

	void GetOutput() {
		MatrixXd xu = MergeByRow(x, u);
		y = CD * xu;
		MatrixXd xu_prime = MergeByRow(x_prime, u_prime);
		y_prime = CD * xu_prime;
		MatrixXd xu_quan = MergeByRow(x_quan, u_quan);
		y_quan = CD * xu_quan;
	}
	void UpdateState() {
		MatrixXd xu = MergeByRow(x, u);
		x = AB * xu;
		MatrixXd xu_prime = MergeByRow(x_prime, u_prime);
		x_prime = AB * xu_prime;
		MatrixXd xu_quan = MergeByRow(x_quan, u_quan);
		x_quan = AB * xu_quan;
	}
	void ControlLoop() {
		ofs.open(writeFilePath.data());
		if (!ofs.is_open()) {
			cout << "No output file" << endl;
			return;
		}
		for (int t = 0;t <= 120 / T_s;t++) {
			GetOutput();
			if (t % 50 == 0 && t > 0) {
				cout << "step=" << t << endl;
				cout << "r - y_prime=\t\t(reference signal vs. original system)" << endl;
				cout << (Substraction(r, y_prime)) << endl;
				cout << "y_quan - y_prime=\t(quantized system vs. original system)" << endl;
				cout << (Substraction(y_quan, y_prime)) << endl;
				cout << "y - y_prime=\t\t(encrypted system vs. original system)" << endl;
				cout << (Substraction(y, y_prime)) << endl;
				cout << "u - u_prime=\t(error between u of encrypted system and u_prime of original system)" << endl;
				cout << (Substraction(u, u_prime)) << endl;
				//ofs << (int)(T_s*t) << " " << (Substraction(y_quan, y_prime)) << " " << (Substraction(r, y)) << endl;
				ofs << (int)(T_s*t) << " " << r << " " << y_prime << " " << y_quan << " " << y_prime << endl;
			}
			u = encdec->Dec_u(controller->GetOutput(encdec->Enc(Substraction(r, y), true)));
			u = TruncateMatrix(u, r_u_inverse);
			controller->UpdateState(encdec->Enc(Substraction(r, y), true), encdec->Enc(u, true));
			u_prime = TruncateMatrix(controllerOrigin->Step(Substraction(r, y_prime)), r_u_inverse);
			u_quan = TruncateMatrix(quantizedController->Step(Substraction(r, y_quan)), r_u_inverse);
			UpdateState();
		}
		errors.push_back(Substraction(y, y_prime)(0, 0));
		ofs.close();
		//calculateInterval();
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
	MatrixXd TruncateMatrix(MatrixXd mtx, int truncator) {
		MatrixXd result(mtx.rows(), mtx.cols());
		for (int i = 0;i < mtx.rows(); i++)
			for (int j = 0;j < mtx.cols();j++)
				result(i, j) = round(mtx(i, j) * truncator) / truncator;
		return result;
	}
	MatrixXd Substraction(MatrixXd mtx_left, MatrixXd mtx_right) {
		MatrixXd result(mtx_left.rows(), mtx_left.cols());
		for (int i = 0;i < mtx_left.rows(); i++)
			for (int j = 0;j < mtx_left.cols();j++)
				result(i, j) = mtx_left(i, j) - mtx_right(i, j);
		return result;
	}
	double GetInfiniteNorm(MatrixXd m) {
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
	int DeltaEnc() {
		return round((error_range - 1) / 2);
	}
	int DeltaMult(int n) {
		int d = 3;
		int nu = 16;
		return round(d*(n + 1)*(error_range - 1) / 2 * nu) + 1;
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
	EncrypterDecrypter *encdec = new EncrypterDecrypter(1000, 1, 1, 10000, 1000, 3, 1);

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
	EncrypterDecrypter *encdec = new EncrypterDecrypter(100, 1, 1, 8000, 1000, 3, 1);

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
		plant->ControlLoop();
		total += abs(errors.back());
	}
	cout << (total/(double)step) << endl;
	return 0;
}
