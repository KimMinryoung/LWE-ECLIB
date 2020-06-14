#include "stdafx.h"
#include "Encrypter.h"

static std::random_device rnd;
static std::mt19937_64 generator(rnd());
static std::uniform_int_distribution<unsigned __int64> distribution_;
Encrypter::Encrypter(int r_y_inverse, int s_1_inverse, int s_2_inverse, int U, int L_inverse, double sigma, int n) {
	srand(time(NULL));

	if (n == -1)
		this->n = 100; // temporal ciphertext dimension for speed measuring
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
	this->U = U; // range of u
	logN = ceil(log2(this->U) + log2(L_inverse) + log2(s_1_inverse) + log2(s_2_inverse));
	N = (unsigned __int64)pow(2, logN); // plaintext modulus
	logq = logN; // q:=N
	q = (unsigned __int64)pow(2, logq); // ciphertext modulus
	nu = 16;
	d = (int)ceil((double)logq / nu);
	this->sigma = sigma;

	if (n != -1) {
		cout << "parameters of the LWE cryptosystem" << endl;
		cout << "q=N=2^" << logq << ", nu=2^" << nu << ", d=" << d << ",\nn=" << this->n << ", sigma=" << this->sigma << endl;
		printf("---------------\n");
		cout << "parameters for quantization:" << endl;
		cout << "1/L=" << L_inverse << ", 1/s_1=" << s_1_inverse << ", 1/s_2=" << s_2_inverse << ", 1/r_y=" << r_y_inverse << ", U=" << U << endl;
		printf("---------------\n");
	}
	PrintSecurityLevel();

	q_dividedby_N = pow(2, logq - logN);
	q_ = q - 1; // q_ and N_ are for bitwise operations which substitutes modulus operations
	nu_ = (unsigned __int64)pow(2, nu) - 1;
}
int Encrypter::Set_n(double currentTimeSpan, double T_s, double bandwidth) {
	std::cout << "Time span test(by n=" << n << ") result: " << currentTimeSpan << " seconds" << endl;
	// to solve inequality a(n+1)^2 + b(n+1) + c <= 0
	double a = currentTimeSpan / (n_) / (n_);
	double b = 4 * 64 / bandwidth * pow(10, -6);
	double c = -0.9 * T_s;
	n_ = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
	cout << n_ << endl;
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
Decrypter* Encrypter::GenerateDecrypter() {
	Decrypter* decrypter = new Decrypter(secretKey, r_y_inverse, s_1_inverse, s_2_inverse, L_inverse, logq, n);
	return decrypter;
}
void Encrypter::PrintSecurityLevel() {
	if (n <= 500) {
		security_level = 25.094 * pow(n, 0.0503);
	}
	else {
		security_level = 0.00009 * n * n - 0.0431 * n + 31.237 + 3.0 * (48 - logq);
	}
	cout << "security level: rop=2^" << security_level << endl;
	printf("---------------\n");
}

MatrixXu Encrypter::Enc(MatrixXd m, bool signal) {
	MatrixXu c(n_*m.rows(), m.cols());
	std::normal_distribution<> normal_random{ 0, sigma };

	for (int i_m = 0;i_m < m.rows();i_m++) {
		// a
		MatrixXu a(n, m.cols());

		for (int i_a = 0; i_a < n; i_a++) { // vector a's length=n
			for (int j_a = 0;j_a < m.cols();j_a++) {
				// uniform-randomly generate a entry
				a(i_a, j_a) = distribution_(generator) & q_;
				c(i_m*n_ + (i_a + 1), j_a) = a(i_a, j_a);
			}
		}

		// b
		VectorXu s_mult_a = MultMxV(a, secretKey); // a*sk

		for (int j_c = 0; j_c < m.cols(); j_c++) {
			double _noise = normal_random(generator); // inject _noise ~ N(0, sigma)
			int noise = round(_noise);
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
MatrixXu Encrypter::Encm(MatrixXd m) {
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
MatrixXu Encrypter::Add(MatrixXu c1, MatrixXu c2) {
	MatrixXu result(c1.rows(), c1.cols());
	result = c1 + c2;
	for (int i = 0; i < c1.rows(); i++) {
		for (int j = 0; j < c1.cols(); j++) {
			result(i, j) = result(i, j) & q_;
		}
	}
	return result;
}
MatrixXu Encrypter::ScalarMult(unsigned __int64 scalar, MatrixXu c) {
	MatrixXu result(c.rows(), c.cols());
	result = scalar * c;
	for (int i = 0; i < c.rows(); i++) {
		for (int j = 0; j < c.cols(); j++) {
			result(i, j) = result(i, j) & q_;
		}
	}
	return result;
}
MatrixXu Encrypter::MultMxM(MatrixXu encm, MatrixXu split_enc) {
	int size_i = encm.rows();
	int size_k = split_enc.rows();
	int size_j = split_enc.cols();
	MatrixXu result(size_i, size_j);
	result.noalias() = encm * split_enc;

	for (int i = 0; i < size_i;i++)
		for (int j = 0; j < size_j; j++)
			result(i, j) &= q_;

	return result;
}
VectorXu Encrypter::MultMxV(MatrixXu m, RowVectorXu vec) {
	int size_i = m.rows();
	int size_ii = m.cols();
	VectorXu y = VectorXu::Zero(size_ii);

	y = (vec * m);
	for (int i = 0;i < size_ii;i++)
		y(i) &= q_;
	return y;
}
MatrixXu Encrypter::SplitMtx(MatrixXu c) {
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
int Encrypter::Getq() {
	return logq;
}