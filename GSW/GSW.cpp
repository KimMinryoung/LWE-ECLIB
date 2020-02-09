// GSW.cpp: 콘솔 응용 프로그램의 진입점을 정의합니다.
//

#include "stdafx.h"
#include "time.h"
#include "chrono"
#include "iostream"
#include "stdlib.h"
#include "math.h"
#include "random"
#include "vector"
#include "fstream"
#include "string"

#include "windows.h"
#include "sysinfoapi.h"
long int beginTime;

using namespace std;

unsigned __int64 mod_q_ = pow(2, 48) - 1;
vector<double> errors = vector<double>();

std::random_device random;
std::mt19937_64 generator(random());
std::uniform_int_distribution<unsigned __int64> distribution;

void print(vector<vector<unsigned __int64>> m) {
	for (int i = 0;i < m.size();i++) {
		for (int j = 0;j < m[0].size();j++) {
			printf("%I64u\t", m[i][j]);
		}
		printf("\n");
	}
	printf("---------------\n");
}
void print(vector<vector<double>> m) {
	for (int i = 0;i < m.size();i++) {
		for (int j = 0;j < m[i].size();j++) {
			printf("%.10g\t", m[i][j]);
		}
		printf("\n");
	}
	printf("---------------\n");
}

// >>>>>> For GSW
vector<vector<unsigned __int64>> splitm(vector<vector<unsigned __int64>> c) {
	int q = 48;
	int nu = 16;
	int d = ceil((double)q / nu);
	unsigned __int64 nu_ = pow(2, nu) - 1;
	vector<vector<unsigned __int64>> result(c.size() * d, vector<unsigned __int64>(c[0].size(), 0));
	vector<vector<unsigned __int64>> c_temp(c.size(), vector<unsigned __int64>(c[0].size(), 0));
	for (int i = 0;i < c.size(); i++) {
		for (int j = 0;j < c[0].size();j++) {
			c_temp[i][j] = c[i][j];
		}
	}

	for (int piece = 0;piece < d;piece++) {
		for (int i = 0;i < c.size();i++) {
			for (int j = 0;j < c[0].size();j++) {
				result[piece * c.size() + i][j] = c_temp[i][j] & nu_;
				c_temp[i][j] = c_temp[i][j] >> (nu);
			}
		}
	}
	return result;
}
vector<vector<unsigned __int64>> mult(vector<vector<unsigned __int64>> encm, vector<vector<unsigned __int64>> split_enc) {
	vector<vector<unsigned __int64>> result(encm.size(), vector<unsigned __int64>(split_enc[0].size(), 0));

	int size_i = encm.size();
	int size_k = split_enc.size();
	int size_j = split_enc[0].size();

	for (int i = 0; i < size_i;i++) {
		for (int k = 0;k < size_k;k++) {
			for (int j = 0;j < size_j;j++) {
				result[i][j] = (result[i][j] + encm[i][k] * split_enc[k][j]) & mod_q_;
			}
		}
	}

	return result;
}
// <<<<<< For GSW

// >>>>>> For LWE
vector<vector<unsigned __int64>> scalar_mult(unsigned __int64 scalar, vector<vector<unsigned __int64>> c) {
	vector<vector<unsigned __int64>> result(c.size(), vector<unsigned __int64>(c[0].size(), 0));
	for (int i = 0;i < c.size();i++) {
		for (int j = 0;j < c[0].size();j++) {
			result[i][j] = (c[i][j] * scalar) & mod_q_;
		}
	}
	return result;
}
vector<vector<unsigned __int64>> add_enc(vector<vector<unsigned __int64>> c1, vector<vector<unsigned __int64>> c2) {
	vector<vector<unsigned __int64>> result(c1.size(), vector<unsigned __int64>(c1[0].size(), 0));
	for (int i = 0;i < c1.size();i++) {
		for (int j = 0;j < c1[0].size();j++) {
			result[i][j] = (c1[i][j] + c2[i][j]) & mod_q_;
		}
	}
	return result;
}
// <<<<<< For LWE

void startCount() {
	beginTime = GetTickCount();
}
void printInterval() {
	cout << GetTickCount() - beginTime << "\t[ms]"
		<< " ( " << (double)(GetTickCount() - beginTime) / 1000 << "\t[s] )" << endl;
}


class PIDControllerOrigin  {

private:
	vector<vector<double>> FG;
	vector<vector<double>> HJ;
	vector<vector<double>> x;
	vector<vector<double>> u;

public:
	PIDControllerOrigin(vector<vector<double>> FG, vector<vector<double>> HJ, vector<vector<double>> x_init) {
		this->FG = FG;
		this->HJ = HJ;
		this->x = x_init;
	}

	vector<vector<double>> Step(vector<vector<double>> y) {
		vector<vector<double>> xy = mergeByRow(x, y);
		u = multPlain(HJ, xy);
		x = multPlain(FG, xy);
		return u;
	}
	vector<vector<double>> mergeByRow(vector<vector<double>> a, vector<vector<double>> b) {
		vector<vector<double>> result(a.size() + b.size(), vector<double>(a[0].size(), 0));
		for (int i = 0;i < a.size();i++) {
			for (int j = 0;j < a[0].size();j++) {
				result[i][j] = a[i][j];
			}
		}
		for (int i = 0;i < b.size();i++) {
			for (int j = 0;j < b[0].size();j++) {
				result[a.size() + i][j] = b[i][j];
			}
		}
		return result;
	}
	vector<vector<double>> multPlain(vector<vector<double>> mtx_left, vector<vector<double>> mtx_right) {
		vector<vector<double>> result(mtx_left.size(), vector<double>(mtx_right[0].size(), 0));
		for (int i = 0;i < mtx_left.size(); i++) {
			for (int j = 0;j < mtx_right[0].size();j++) {
				for (int k = 0;k < mtx_right.size();k++) {
					result[i][j] += mtx_left[i][k] * mtx_right[k][j];
				}
			}
		}
		return result;
	}
};
class EncrypterDecrypter {
private:
	vector<unsigned __int64> secretKey;
	unsigned __int64 mod_s;
	unsigned __int64 mod_q; // ciphertext modulus
	unsigned __int64 mod_N; // plaintext modulus
	unsigned __int64 mod_e;
	int n;
	int n_;

	int R_y;
	int L;
	//int L_dividedby_r;
	double L_dividedby_r;
	int s_1;
	int s_2;
	int M;
	int nu;
	int d;
	int N;
	int q;

	unsigned __int64 mod_qe;
	unsigned __int64 mod_qN;
	unsigned __int64 nu_;
	unsigned __int64 mod_q_;
	unsigned __int64 mod_N_;

public:
	EncrypterDecrypter(int R_y, int s_1, int s_2, int M, int L, int mod_e) {
		srand(time(NULL));

		//n = 249; // ciphertext dimension
		n = 1;
		n_ = n + 1;
		mod_s = 256; // secret key range

		for (int i = 0;i < n;i++) {
			secretKey.push_back(rand() % mod_s);
		}

		this->R_y = R_y; // signal resolution
		this->L = L; // scaling factor for signal
		this->L_dividedby_r = (double)L / (double)R_y;
		this->s_1 = s_1; // scaling factor for matrix G
		this->s_2 = s_2; // scaling factor for matrix H and J
		this->M = ceil(M * L_dividedby_r);
		N = ceil(log2(this->M) + log2(R_y) + log2(s_1) + log2(s_2));
		mod_N = pow(2, N); // plaintext modulus
		q = N;
		mod_q = pow(2, q); // ciphertext modulus
		nu = 16;
		d = ceil((double)q / nu);

		printf("---------------\n");
		cout << "parameters of the LWE cryptosystem" << endl;
		cout << "q=N=2^" << q << ", nu=2^" << nu << ", d=" << d << ",\nn=" << n << ", sigma=" << (mod_e - 1) / 2 << endl;
		printf("---------------\n");
		cout << "parameters for quantization:" << endl;
		cout << "L=" << L << ", s_1=" << s_1 << ", s_2=" << s_2 << ", r=" << R_y << endl;
		printf("---------------\n");

		this->mod_e = mod_e; // error range -> odd number
		mod_qe = mod_q - (mod_e - 1) / 2;
		mod_qN = pow(2, q - N);
		mod_q_ = mod_q - 1;
		mod_N_ = mod_N - 1;
		nu_ = pow(2, nu) - 1;
	}
	vector<vector<double>> Dec(vector<vector<unsigned __int64>> c, int scaling, bool signal) {
		int l = c.size() / n_;
		vector<vector<double>> y(l, vector<double>(c[0].size(), 0));

		for (int j = 0;j < c[0].size();j++) {
			for (int i = 0;i < l;i++) {
				unsigned __int64 temp = c[i*n_][j];
				for (int k = 0;k < n;k++) {
					temp += secretKey[k] * c[i*n_ + k + 1][j];
				}
				unsigned __int64 m = (unsigned __int64)round(temp) & (mod_N_);
				signed __int64 real_m = m - (m >= mod_N / 2) * mod_N;

				if (signal)
					y[i][j] = (double)real_m / (double)L / (double)scaling;
				else
					y[i][j] = real_m / (double)R_y / (double)scaling;
			}
		}
		return y;
	}
	vector<vector<double>> Dec(vector<vector<unsigned __int64>> c) {
		return Dec(c, 1, false);
	}

	vector<vector<double>> Dec_signal(vector<vector<unsigned __int64>> c) {
		return Dec(c, s_1*s_2, true);
	}

	vector<vector<unsigned __int64>> Enc(vector<vector<double>> m, bool signal) {
		vector<vector<unsigned __int64>> c(n_*m.size(), vector<unsigned __int64>(m[0].size(), 0));

		for (int i_m = 0;i_m < m.size();i_m++) {
			// a
			vector<vector<unsigned __int64>> a(n, vector<unsigned __int64>(m[0].size(), 0));

			for (int i_a = 0; i_a < n; i_a++) {
				for (int j_a = 0;j_a < m[0].size();j_a++) {
					a[i_a][j_a] = distribution(generator) % mod_q;
					c[i_m*n_ + (i_a + 1)][j_a] = a[i_a][j_a];
				}
			}

			// b
			vector<unsigned __int64> s_mult_a = mult(secretKey, a);
			for (int j_c = 0;j_c < m[0].size();j_c++) {
				int e = rand() % mod_e + mod_qe;
				int quantized = round(m[i_m][j_c] * R_y);
				if (signal) {
					//signed __int64 scaled = (signed __int64)L_dividedby_r * (signed __int64)quantized;
					signed __int64 scaled = round(L_dividedby_r * quantized);
					c[i_m*n_][j_c] = (unsigned __int64)(mod_q - s_mult_a[j_c] + (mod_N + scaled) + e) & mod_q_;
				}
				else
					c[i_m*n_][j_c] = (unsigned __int64)(mod_q - s_mult_a[j_c] + (mod_N + quantized) + e) & mod_q_;
			}

		}
		return c;
	}
	vector<vector<unsigned __int64>> Encm(vector<vector<double>> m) {
		vector<vector<unsigned __int64>> c(n_ * m.size(), vector<unsigned __int64>(n_ * m[0].size() * d, 0));
		vector<vector<unsigned __int64>> m_temp(m.size(), vector<unsigned __int64>(m[0].size(), 0));
		for (int i = 0;i < m.size(); i++) {
			for (int j = 0;j < m[0].size();j++) {
				m_temp[i][j] = mod_q + round(m[i][j]);
			}
		}

		int piece_start = 0;
		vector<vector<double>> zeros(1, vector<double>(n_, 0));
		vector<vector<unsigned __int64>> enc_zeros;

		for (int piece = 0;piece < d;piece++) {
			for (int i_m = 0;i_m < m.size();i_m++) {
				for (int j_m = 0;j_m < m[0].size();j_m++) {
					enc_zeros = Enc(zeros, false);
					for (int i_c = 0;i_c < n_;i_c++) {
						int i_c_real = i_m * n_ + i_c;
						for (int j_c = 0;j_c < n_;j_c++) {
							int j_c_real = piece_start + j_m * n_ + j_c;
							c[i_c_real][j_c_real] = (c[i_c_real][j_c_real] + (i_c == j_c) * m_temp[i_m][j_m]) & mod_q_;
							c[i_c_real][j_c_real] = (c[i_c_real][j_c_real] + enc_zeros[i_c][j_c]) & mod_q_;
						}
					}

				}
			}

			for (int i_m = 0;i_m < m.size(); i_m++) {
				for (int j_m = 0;j_m < m[0].size();j_m++) {
					m_temp[i_m][j_m] = (m_temp[i_m][j_m] << nu) & mod_q_;
				}
			}
			piece_start = piece_start + (n_)* m[0].size();
		}

		return c;
	}
	/*
	int binary_search(f, start, stop, param, predicate = lambda x, best: x <= best, *arg, **kwds) {
		//Searches for the best value in the interval[start, stop] depending on the given predicate.

			: param start : start of range to search
			: param stop : stop of range to search(exclusive)
			: param param : the parameter to modify when calling `f`
			: param predicate : comparison is performed by evaluating ``predicate(current, best)``
			"""
			kwds[param] = stop
			D = {}
			D[stop] = f(*arg, **kwds)
			best = D[stop]
			b = ceil((start + stop) / 2)
			direction = 0
			while True:
		if b not in D :
		kwds[param] = b
			D[b] = f(*arg, **kwds)
			if b == start :
				best = D[start]
				break
				if not predicate(D[b], best) :
					if direction == 0 :
						start = b
						b = ceil((stop + b) / 2)
					else :
						stop = b
						b = floor((start + b) / 2)
				else:
		best = D[b]
			logging.getLogger("binsearch").debug(u"%4d, %s" % (b, best))
			if b - 1 not in D :
		kwds[param] = b - 1
			D[b - 1] = f(*arg, **kwds)
			if predicate(D[b - 1], best) :
				stop = b
				b = floor((b + start) / 2)
				direction = 0
			else:
		if b + 1 not in D :
		kwds[param] = b + 1
			D[b + 1] = f(*arg, **kwds)
			if not predicate(D[b + 1], best) :
				break
			else :
				start = b
				b = ceil((stop + b) / 2)
				direction = 1
				return best
	}
	double getSecurityLevel() {

		//n, alpha, q, success_probability

		scale = _primal_scale_factor(secret_distribution, alpha, q, n);

		kwds = { "n": n, "alpha" : alpha, "q" : q,
			"kannan_coeff" : kannan_coeff, "d" : d,
			"reduction_cost_model" : reduction_cost_model,
			"m" : m, "scale" : scale };

		double cost = binary_search(_primal_usvp, start = 40, stop = 2 * n, param = "block_size",
				predicate = lambda x, best: x["red"] <= best["red"], **kwds);

		for block_size in range(32, cost["beta"] + 1)[:: - 1]:
			t = _primal_usvp(block_size = block_size, **kwds)
				if t["red"] == oo :
					break;
					cost = t;

		cost["m"] = cost["d"] - 1;

		return cost;
	}*/
	/*
	def primal_usvp(n, alpha, q, secret_distribution = True,
		m = oo, success_probability = 0.99,
		kannan_coeff = None, d = None,
		reduction_cost_model = reduction_default_cost, **kwds) :
		u"""
		Estimate cost of solving LWE using primal attack(uSVP version)

		: param n : LWE dimension `n > 0`
		:param alpha : noise rate `0 ≤ α < 1`, noise will have standard deviation `αq / \\sqrt{ 2π }`
		:param q : modulus `0 < q`
		:param secret_distribution : distribution of secret, see module level documentation for details
		: param m : number of LWE samples `m > 0`
		:param success_probability : targeted success probability < 1
		: param reduction_cost_model : cost model for lattice reduction

			n, alpha, q, success_probability = Param.preprocess(n, alpha, q, success_probability)


				scale = _primal_scale_factor(secret_distribution, alpha, q, n)

				kwds = { "n": n, "alpha" : alpha, "q" : q,
				"kannan_coeff" : kannan_coeff, "d" : d,
				"reduction_cost_model" : reduction_cost_model,
				"m" : m, "scale" : scale }

				cost = binary_search(_primal_usvp, start = 40, stop = 2 * n, param = "block_size",
					predicate = lambda x, best: x["red"] <= best["red"], **kwds)

				for block_size in range(32, cost["beta"] + 1)[:: - 1]:
	t = _primal_usvp(block_size = block_size, **kwds)
		if t["red"] == oo :
			break
			cost = t

	cost["m"] = cost["d"] - 1

		return cost
		*/
	vector<unsigned __int64> mult(vector<unsigned __int64> c, vector<vector<unsigned __int64>> HJ) {
		vector<unsigned __int64> y(HJ[0].size(), 0);

		int size_i = HJ.size();
		int size_ii = HJ[0].size();

		for (int i = 0;i < size_i;i++) {
			unsigned __int64 c_i = c[i];
			for (int ii = 0;ii < size_ii;ii++) {
				y[ii] = (y[ii] + c_i * HJ[i][ii]) & mod_q_;
			}
		}

		return y;
	}
	int Getq() {
		return q;
	}
};
class PIDController {

private:
	vector<vector<unsigned __int64>> encm_FG;
	vector<vector<unsigned __int64>> encm_HJ;
	int q = 48;
	unsigned __int64 mod_q_ = pow(2, q) - 1;
	const int nu = 16;
	const unsigned __int64 nu_ = pow(2, nu) - 1;
	int d = q / nu;

public:
	vector<vector<unsigned __int64>> enc_x;
	PIDController(vector<vector<unsigned __int64>> encm_FG, vector<vector<unsigned __int64>> encm_HJ, vector<vector<unsigned __int64>> enc_x_init, int q) {
		this->encm_FG = encm_FG;
		this->encm_HJ = encm_HJ;
		this->enc_x = enc_x_init;
		this->q = q;
		mod_q_ = pow(2, q) - 1;
		d = ceil((double)q / nu);
	}
	
	vector<vector<unsigned __int64>> Step(vector<vector<unsigned __int64>> enc_y) {
		vector<vector<unsigned __int64>> enc_xy = mergeByRow(enc_x, enc_y);
		vector<vector<unsigned __int64>> split_enc_xy = splitm(enc_xy);
		//startCount();
		vector<vector<unsigned __int64>> enc_u = mult(encm_HJ, split_enc_xy);  // controller output
		//printInterval();

		//startCount();
		enc_x = mult(encm_FG, split_enc_xy); // controller state
		//printInterval();
		return enc_u;
	}
	vector<vector<unsigned __int64>> mergeByRow(vector<vector<unsigned __int64>> a, vector<vector<unsigned __int64>> b) {
		vector<vector<unsigned __int64>> result(a.size() + b.size(), vector<unsigned __int64>(a[0].size(), 0));
		for (int i = 0;i < a.size();i++) {
			for (int j = 0;j < a[0].size();j++) {
				result[i][j] = a[i][j];
			}
		}
		for (int i = 0;i < b.size();i++) {
			for (int j = 0;j < b[0].size();j++) {
				result[a.size() + i][j] = b[i][j];
			}
		}
		return result;
	}
	vector<vector<unsigned __int64>> mult(vector<vector<unsigned __int64>> encm, vector<vector<unsigned __int64>> split_enc) {
		int size_i = encm.size();
		int size_k = split_enc.size();
		int size_j = split_enc[0].size();
		//cout << "size_j "<<size_j << endl;
		vector<unsigned __int64> result(size_i*size_j, 0);
		for (int i = 0; i < size_i;i++) {
			vector<unsigned __int64> encm_i = encm[i];
			for (int k = 0;k < size_k;k++) {
				for (int j = 0;j < size_j;j++) {
					result[size_j * i + j] = (result[size_j * i + j] + encm_i[k] * split_enc[k][j]) & mod_q_;
				}
			}
		}
		vector<vector<unsigned __int64>> result_2d(size_i, vector<unsigned __int64>(size_j, 0));
		for (int i = 0; i < size_i;i++) {
			for (int j = 0;j < size_j;j++) {
				result_2d[i][j] = result[size_j * i + j];
			}
		}
		return result_2d;
	}
	vector<vector<unsigned __int64>> splitm(vector<vector<unsigned __int64>> c) {
		vector<vector<unsigned __int64>> result(c.size() * d, vector<unsigned __int64>(c[0].size(), 0));
		vector<vector<unsigned __int64>> c_temp(c.size(), vector<unsigned __int64>(c[0].size(), 0));
		for (int i = 0;i < c.size(); i++) {
			for (int j = 0;j < c[0].size();j++) {
				c_temp[i][j] = c[i][j];
			}
		}

		for (int piece = 0;piece < d;piece++) {
			for (int i = 0;i < c.size();i++) {
				for (int j = 0;j < c[0].size();j++) {
					result[piece * c.size() + i][j] = c_temp[i][j] & nu_;
					c_temp[i][j] = c_temp[i][j] >> (nu);
				}
			}
		}
		return result;
	}
};

class Plant {

private:
	vector<vector<double>> FG;
	vector<vector<double>> HJ;
	vector<vector<double>> x;
	vector<vector<double>> x_prime;
	vector<vector<double>> u;
	vector<vector<double>> u_prime;
	vector<vector<double>> y;
	vector<vector<double>> y_prime;
	vector<vector<double>> r;
	int s_1;
	int s_2;
	int R_y;
	int R_u;
	int M;
	int L;
	int mod_e = 3;
	PIDController *controller;
	PIDControllerOrigin *controllerOrigin;
	EncrypterDecrypter *encdec;

	string filePath = "parameters.txt";
	ifstream ifs;

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
		u = readMatrix("y");
		u_prime = u;
		y = readMatrix("r");
		y_prime = y;
		r = readMatrix("PID");
		getline(ifs, line, '\n');
		size_t pos = 0;
		pos = line.find('\t');
		double k_p = stod(line.substr(0, pos));
		line.erase(0, pos + 1);
		pos = line.find('\t');
		double k_i = stod(line.substr(0, pos));
		line.erase(0, pos + 1);
		pos = line.find('\t');
		double k_d = stod(line.substr(0, pos));
		line.erase(0, pos + 1);
		pos = line.find('\t');
		int N_d = stoi(line.substr(0, pos));
		line.erase(0, pos + 1);
		double T_s = stod(line);
		ifs.close();

		printf("plant initial state x=\n");
		print(x);
		printf("plant initial output y=\n");
		print(y);
		printf("reference signal r=\n");
		print(r);
		printf("plant matrix AB=\n");
		print(FG);
		printf("plant matrix CD=\n");
		print(HJ);
		cout << "PID parameters:\n" << "k_p=" << k_p << ", k_i=" << k_i << ", k_d=" << k_d << ", N_d=" << N_d << ", T_s=" << T_s << endl;
		printf("---------------\n");

		//R_y = pow(2,9); // scaling factor for signal
		R_y = pow(2, 10); // scaling factor for signal
		R_u = pow(2, 8); // scaling factor for signal
		s_1 = 1; // scaling factor for controller matrix G
		//s_2 = pow(2,9); // scaling factor for controller matrices H and J
		s_2 = pow(2, 10);
		M = pow(2, 11); // plaintext range for signal y, u, and controller state x
		int mod_e = 3;

		cout << "delta_Enc: " << delta_Enc() << ", delta_Mult: " << delta_Mult() << endl;
		printf("---------------\n");

		buildPIDController(k_p, k_i, k_d, N_d, T_s);
	}

	void buildPIDController(double k_p, double k_i, double k_d, int N_d, double T_s) {
		vector<vector<double>> F = { { 2 - (double)N_d, (double)N_d - 1 },{ 1,0 } };
		vector<vector<double>> G = { { 1 },{ 0 } };
		int x_row = F.size();
		int y_row = G[0].size();
		vector<vector<double>> FG = vector<vector<double>>(x_row, vector<double>(x_row + y_row));
		for (int row = 0;row < x_row;row++) {
			for (int col = 0;col < x_row;col++) {
				FG[row][col] = F[row][col];
			}
			for (int col = x_row;col < x_row + y_row;col++) {
				FG[row][col] = G[row][col - x_row];
			}
		}
		vector<vector<double>> FG_scaled = vector<vector<double>>(x_row, vector<double>(x_row + y_row));
		for (int row = 0;row < x_row;row++) {
			for (int col = 0;col < x_row;col++) {
				FG_scaled[row][col] = FG[row][col];
			}
			for (int col = x_row;col < x_row + y_row;col++) {
				FG_scaled[row][col] = round(FG[row][col] * s_1);
			}
		}

		vector<vector<double>> H = { { k_i * T_s - k_d * N_d * N_d / T_s, k_i * T_s * N_d - k_i * T_s + k_d * N_d * N_d / T_s } };
		vector<vector<double>> J = { { k_p + k_d * N_d / T_s } };
		int u_row = H.size();
		vector<vector<double>> HJ = vector<vector<double>>(u_row, vector<double>(x_row + y_row));
		for (int row = 0;row < u_row;row++) {
			for (int col = 0;col < x_row;col++) {
				HJ[row][col] = H[row][col];
			}
			for (int col = x_row;col < x_row + y_row;col++) {
				HJ[row][col] = J[row][col - x_row];
			}
		}
		vector<vector<double>> HJ_scaled = vector<vector<double>>(u_row, vector<double>(x_row + y_row));
		for (int row = 0;row < u_row;row++) {
			for (int col = 0;col < x_row;col++) {
				HJ_scaled[row][col] = round(HJ[row][col] * s_2);
			}
			for (int col = x_row;col < x_row + y_row;col++) {
				HJ_scaled[row][col] = round(HJ[row][col] * s_2 * s_1);
			}
		}

		//R_y = pow(2,10); // scaling factor for signal
		R_y = 1000; // scaling factor for signal
		s_1 = 1; // scaling factor for controller matrix G
				 //s_2 = pow(2,10); // scaling factor for controller matrices H and J
		s_2 = 1000;
		M = pow(2, 11); // plaintext range for signal y, u, and controller state x

		double G_norm = getInfiniteNorm(G);
		double J_norm = getInfiniteNorm(J);
		int degrade_bound = 7;
		double L1 = (G_norm * delta_Enc() + (x.size() + y.size() + r.size()) * delta_Mult() / s_1) / (double)(R_y / 2 * (G_norm)) * pow(10, degrade_bound);
		double L2 = (J_norm * delta_Enc() + (x.size() + y.size() + r.size()) * delta_Mult() / s_1 / (double)s_2) / (R_y / 2 * J_norm + R_u / 2) * pow(10, degrade_bound);
		double L3 = 2 * delta_Enc() * pow(10, degrade_bound);
		//L = pow(10, ceil(log10(max(R_y, max(max(L1, L2), L3)))));
		L = pow(2, ceil(log2(max(R_y, max(max(L1, L2), L3)))));
		cout << "(Beta1 - Alpha1) / Alpha1 = " << (G_norm * delta_Enc() / L + (x.size() + y.size() + r.size()) * delta_Mult() / L / s_1) / (double)(R_y / 2 * (G_norm)) << endl;
		cout << "(Beta2 - Alpha2) / Alpha2 = " << (J_norm * delta_Enc() / L + (x.size() + y.size() + r.size()) * delta_Mult() / L / s_1 / (double)s_2) / (R_y / 2 * (J_norm + 1)) << endl;
		cout << "(Beta3 - Alpha3) / Alpha3 = " << 2 * (double)delta_Enc() / L << endl;

		encdec = new EncrypterDecrypter(R_y, s_1, s_2, M, L, mod_e);

		vector<vector<unsigned __int64>> encm_FG = encdec->Encm(FG_scaled);
		vector<vector<unsigned __int64>> encm_HJ = encdec->Encm(HJ_scaled);

		vector<vector<double>> x_init_con = { { 0 }, { 0 } };
		vector<vector<unsigned __int64>> enc_x_init_con = encdec->Enc(x_init_con, false);

		controllerOrigin = new PIDControllerOrigin(FG, HJ, x_init_con);
		controller = new PIDController(encm_FG, encm_HJ, enc_x_init_con, encdec->Getq());
	}

	void getOutput() {
		vector<vector<double>> xu = mergeByRow(x, u);
		y = multPlain(HJ, xu); // plant output
		vector<vector<double>> xu_prime = mergeByRow(x_prime, u_prime);
		y_prime = multPlain(HJ, xu_prime);
	}
	void updateState() {
		vector<vector<double>> xu = mergeByRow(x, u);
		x = multPlain(FG, xu); // plant state
		vector<vector<double>> xu_prime = mergeByRow(x_prime, u_prime);
		x_prime = multPlain(FG, xu_prime);
	}
	void loop() {
		startCount();
		for (int t = 0;t <= 3000;t++) {
			getOutput();
			/*if (t % 100 == 0) {
				printf("t=%d\n", t);
				cout << "r-y=" << endl;
				print(Substraction(r, y));
				//startCount();
			}*/
			u = truncateMatrix(encdec->Dec_signal(controller->Step(encdec->Enc(Substraction(r, y), true))), R_u);
			u_prime = truncateMatrix(controllerOrigin->Step(Substraction(r, y_prime)), R_u);
			updateState();
		}
		cout << "y-y_prime=" << endl;
		print(Substraction(y, y_prime));
		print(Substraction(r, y_prime));
		cout << "u-u_prime=" << endl;
		print(Substraction(u, u_prime));
		errors.push_back(Substraction(y, y_prime)[0][0]);
		//printInterval();
	}

	vector<vector<double>> mergeByRow(vector<vector<double>> a, vector<vector<double>> b) {
		vector<vector<double>> result(a.size() + b.size(), vector<double>(a[0].size(), 0));
		for (int i = 0;i < a.size();i++) {
			for (int j = 0;j < a[0].size();j++) {
				result[i][j] = a[i][j];
			}
		}
		for (int i = 0;i < b.size();i++) {
			for (int j = 0;j < b[0].size();j++) {
				result[a.size() + i][j] = b[i][j];
			}
		}
		return result;
	}
	vector<vector<double>> multPlain(vector<vector<double>> mtx_left, vector<vector<double>> mtx_right) {
		vector<vector<double>> result(mtx_left.size(), vector<double>(mtx_right[0].size(), 0));
		for (int i = 0;i < mtx_left.size(); i++) {
			for (int j = 0;j < mtx_right[0].size();j++) {
				for (int k = 0;k < mtx_right.size();k++) {
					result[i][j] += mtx_left[i][k] * mtx_right[k][j];
				}
			}
		}
		return result;
	}
	vector<vector<double>> truncateMatrix(vector<vector<double>> mtx, int truncator) {
		vector<vector<double>> result(mtx.size(), vector<double>(mtx[0].size(), 0));
		for (int i = 0;i < mtx.size(); i++) {
			for (int j = 0;j < mtx[0].size();j++) {
				result[i][j] = round(mtx[i][j] * truncator) / truncator;
			}
		}
		return result;
	}
	vector<vector<double>> Substraction(vector<vector<double>> mtx_left, vector<vector<double>> mtx_right) {
		vector<vector<double>> result(mtx_left.size(), vector<double>(mtx_left[0].size(), 0));
		for (int i = 0;i < mtx_left.size(); i++) {
			for (int j = 0;j < mtx_left[0].size();j++) {
				result[i][j] = mtx_left[i][j] - mtx_right[i][j];
			}
		}
		return result;
	}
	double getInfiniteNorm(vector<vector<double>> m) {
		double max = 0;
		for (int i = 0;i < m.size();i++) {
			double sum = 0;
			for (int j = 0;j < m[i].size();j++) {
				sum += abs(m[i][j]);
			}
			if (sum > max)
				max = sum;
		}
		return max;
	}
	int delta_Enc() {
		return round((mod_e - 1) / 2);
	}
	int delta_Mult() {
		int d = 3;
		int n = 1;
		int nu = 16;
		return round(d*(n + 1)*(mod_e - 1) / 2 * nu) + 1;
	}
	vector<vector<double>> readMatrix(string end) {
		vector<vector<double>> m;
		string line;
		int i = 0;
		while (getline(ifs, line, '\n')) {
			if (line.compare(end) == 0)
				break;
			m.push_back(vector<double>());
			size_t pos = 0;
			string token;
			while ((pos = line.find('\t')) != std::string::npos) {
				token = line.substr(0, pos);
				m[i].push_back(stod(token));
				line.erase(0, pos + 1);
			}
			token = line;
			m[i].push_back(stod(token));
			i++;
		}
		return m;
	}
};

//------------------------------------
void LWECheck() {
	EncrypterDecrypter *encdec = new EncrypterDecrypter(32, 1, 1, 16384, 1, 7);

	vector<vector<double>> A = { {1.25, 0.125, -10}, {-100, -5, 1000} };
	printf("A:\n");
	print(A);

	vector<vector<double>> B = { {4, 521, -10}, {150, 1, 2000} };
	printf("B:\n");
	print(B);

	vector<vector<unsigned __int64>> A_enc = encdec->Enc(A, false);
	vector<vector<unsigned __int64>> B_enc = encdec->Enc(B, false);
	vector<vector<unsigned __int64>> A_enc_mult_scalar = scalar_mult(3, A_enc);
	vector<vector<unsigned __int64>> A_enc_add_B_enc = add_enc(A_enc, B_enc);
	printf("Enc(A):\n");
	print(A_enc);
	vector<vector<double>> dec1 = encdec->Dec(A_enc);
	printf("Dec(Enc(A)):\n");
	print(dec1);
	vector<vector<double>> dec2 = encdec->Dec(A_enc_add_B_enc);
	printf("Dec(Enc(A) + Enc(B)):\n");
	print(dec2);
	vector<vector<double>> dec3 = encdec->Dec(A_enc_mult_scalar);
	printf("Dec(Enc(A)*3):\n");
	print(dec3);
}
void GSWCheck() {
	EncrypterDecrypter *encdec = new EncrypterDecrypter(8, 1, 1, 16384, 1, 7);

	int AB_row = 2;
	int AB_col = 3;
	vector<vector<double>> FG = { { -4, 3, 1 },{ 4,11,0 } };
	int xy_row = AB_col;
	int xy_col = 2;
	vector<vector<double>> xu = { { -5, 1 },{ -2.5, 0.125 },{ 2, 0 } };

	printf("AB:\n");
	print(FG);
	printf("xu:\n");
	print(xu);

	vector<vector<unsigned __int64>> encm_FG = encdec->Encm(FG);
	vector<vector<unsigned __int64>> enc_xu = encdec->Enc(xu, false);
	vector<vector<unsigned __int64>> split_enc_xu = splitm(enc_xu);
	vector<vector<unsigned __int64>> enc_mult = mult(encm_FG, split_enc_xu);
	vector<vector<double>> answer = encdec->Dec(enc_mult);
	printf("Dec(Enc(AB) x Enc(xu)):\n");
	print(answer);
}

int main()
{
	srand(time(NULL));
	//LWECheck();
	//GSWCheck();
	int step = 10;
	double total = 0;
	for (int i = 0;i < step;i++) {
		Plant* plant = new Plant();
		plant->loop();
		total += abs(errors.back());
	}
	cout << (total/(double)step) << endl;
    return 0;
}
