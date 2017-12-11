#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;
double dt = 0.12;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Both the reference cross track and orientation errors are 0.
double ref_cte = 0.0;
double ref_epsi = 0.0;
double ref_v = 80;          // reference velocity

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


// cost component weights
const double cost_w_cte = 2500;        // cte
const double cost_w_epsi = 2500;       // epsi
const double cost_w_v = 1;             // speed
const double cost_w_delta = 100;	   // steer
const double cost_w_a = 5;             // throttle
const double cost_w_d_delta = 2500;    // steer change
const double cost_w_d_a = 10;          // throttle change

class FG_eval {
public:
	Eigen::VectorXd coeffs;
	// Coefficients of the fitted polynomial.
	FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	// `fg` is a vector containing the cost and constraints.
	// `vars` is a vector containing the variable values (state & actuators).
	void operator()(ADvector& fg, const ADvector& vars) {
		// The cost is stored is the first element of `fg`.
		// Any additions to the cost should be added to `fg[0]`.
		fg[0] = 0;

		// The part of the cost based on the reference state.
		for (int t = 0; t < N; t++) {
			fg[0] += cost_w_cte * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
			fg[0] += cost_w_epsi * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
			fg[0] += cost_w_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
		}

		// Minimize the use of actuators.
		for (int t = 0; t < N - 1; t++) {
			fg[0] += cost_w_delta * CppAD::pow(vars[delta_start + t], 2);
			fg[0] += cost_w_a * CppAD::pow(vars[a_start + t], 2);
		}

		// Minimize the value gap between sequential actuations.
		for (int t = 0; t < N - 2; t++) {
			fg[0] += cost_w_d_delta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
			fg[0] += cost_w_d_a * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
		}

		// Setup Constraints

		// Initial constraints
		//
		// Add 1 to each of the starting indices due to cost being located at
		// index 0 of `fg`.
		// This bumps up the position of all the other values.
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];

		// The rest of the constraints
		for (int t = 1; t < N; t++) {
			// The state at time t+1 .
			AD<double> x1 = vars[x_start + t];
			AD<double> y1 = vars[y_start + t];
			AD<double> psi1 = vars[psi_start + t];
			AD<double> v1 = vars[v_start + t];
			AD<double> cte1 = vars[cte_start + t];
			AD<double> epsi1 = vars[epsi_start + t];

			// The state at time t.
			AD<double> x0 = vars[x_start + t - 1];
			AD<double> y0 = vars[y_start + t - 1];
			AD<double> psi0 = vars[psi_start + t - 1];
			AD<double> v0 = vars[v_start + t - 1];
			AD<double> cte0 = vars[cte_start + t - 1];
			AD<double> epsi0 = vars[epsi_start + t - 1];

			// Only consider the actuation at time t.
			AD<double> delta0 = vars[delta_start + t - 1];
			AD<double> a0 = vars[a_start + t - 1];

			// cache x^2 and x^3
			AD<double> x0_2 = x0 * x0;
			AD<double> x0_3 = x0_2 * x0;

			// third order poly
			AD<double> f0 = coeffs(1) * x0
							+ coeffs(2) * x0_2
							+ coeffs(3) * x0_3
							+ coeffs(0);

			AD<double> psides0 = CppAD::atan(coeffs(1) + 2 * coeffs(2) * x0
											 + 3 * coeffs(3) * x0_2);

			// Model equations:
			// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
			// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
			// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
			// v_[t+1] = v[t] + a[t] * dt
			// cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
			// epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

			fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
			fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
			fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

		}
	}
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, vector<double>& mpc_x_vals, vector<double>& mpc_y_vals) {
	bool ok = true;
	size_t i;
	typedef CPPAD_TESTVECTOR(double) Dvector;

	// Set the number of model variables (includes both states and inputs).
	size_t n_vars = N * 6 + (N - 1) * 2;
	// Set the number of constraints
	size_t n_constraints = N * 6;

	// Initial value of the independent variables.
	// SHOULD BE 0 besides initial state.
	Dvector vars(n_vars);
	for (int i = 0; i < n_vars; i++) {
		vars[i] = 0;
	}

	// Set the initial state to the current state
	vars[x_start] = state[0];
	vars[y_start] = state[1];
	vars[psi_start] = state[2];
	vars[v_start] = state[3];
	vars[cte_start] = state[4];
	vars[epsi_start] = state[5];

	// Lower and upper limits for vars
	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);

	for (int i = 0; i < delta_start; i++) {
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] = 1.0e19;
	}
	// Limit steering delta (max value 0.436332 (25 degrees in radians))
	const double max_steering_delta = 0.3;
	for (int i = delta_start; i < a_start; i++) {
		vars_lowerbound[i] = -max_steering_delta;
		vars_upperbound[i] = max_steering_delta;
	}
	// Throttle limits
	for (int i = a_start; i < n_vars; i++) {
		vars_lowerbound[i] = -0.1;
		vars_upperbound[i] = 0.8;
	}

	// Lower and upper limits for constraints
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
	for (int i = 0; i < n_constraints; i++) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}

	constraints_lowerbound[x_start] = state[0];
	constraints_lowerbound[y_start] = state[1];
	constraints_lowerbound[psi_start] = state[2];
	constraints_lowerbound[v_start] = state[3];
	constraints_lowerbound[cte_start] = state[4];
	constraints_lowerbound[epsi_start] = state[5];

	constraints_upperbound[x_start] = state[0];
	constraints_upperbound[y_start] = state[1];
	constraints_upperbound[psi_start] = state[2];
	constraints_upperbound[v_start] = state[3];
	constraints_upperbound[cte_start] = state[4];
	constraints_upperbound[epsi_start] = state[5];

	// object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	// options for IPOPT solver
	std::string options;
	// uncomment to print more information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// max time limit for solver
	options += "Numeric max_cpu_time          0.25\n";

	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
		options,
		vars,
		vars_lowerbound,
		vars_upperbound,
		constraints_lowerbound,
		constraints_upperbound,
		fg_eval,
		solution);

	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
	if (!ok) cout << "ERROR: Solver failed!";

	// Cost
	auto cost = solution.obj_value;
	std::cout << "Cost " << cost << std::endl;

	// resize MPC x,y storage vectors to fit N steps
	mpc_x_vals.resize(N - 1);
	mpc_y_vals.resize(N - 1);

	// copy solution values
	for (int i = 0; i < N - 1; i++) {
		mpc_x_vals[i] = solution.x[x_start + i + 1];
		mpc_y_vals[i] = solution.x[y_start + i + 1];
	}

	return { -solution.x[delta_start], solution.x[a_start] };

}
