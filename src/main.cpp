#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using namespace std::chrono;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// number of waypoints to visualize
#define N_VIS_POINTS 11

// process latency 100ms
#define PROCESS_LATENCY 0.1

// NOTE: to turn on visualization of preferred path/fitted curve, set this to true
bool visualize = true;

// set this to false to collect data and write to file for one lap
bool is_lap_finished = true;
vector<vector<double>> data_collected;

#define LERP(A,B,C) ((A)*(1-(C))+(B)*(C))

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.rfind("}]");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

// calculate distance from end of track
double dist_end_track(double px, double py) {
	double dx = -32.906 - px;
	double dy = 113.271 - py;
	return sqrt(dx * dx + dy * dy);
}

// write collected data to csv file
void write_collected_data() {
	cout << "write collected data to file" << endl;
	ofstream myfile;
	myfile.open("data.csv");
	myfile << "x,y,psi,v,cte,epsi,steer,throttle,cost,solver,px,py," << endl;
	for (auto i : data_collected) {
		for (auto o : i) {
			myfile << o << ",";
		}
		myfile << endl;
	}
	myfile.close();
}

// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (int i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}

	for (int j = 0; j < xvals.size(); j++) {
		for (int i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
}

double solver_latency = 0;

int main() {
	uWS::Hub h;

	// MPC is initialized here!
	MPC mpc;

#ifdef USE_LATEST_UWS
	h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
	uWS::OpCode opCode) {
#else
	h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
	uWS::OpCode opCode) {
#endif
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		string sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
			string s = hasData(sdata);
			if (s != "") {
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry") {

					auto start_time = high_resolution_clock::now();

					// j[1] is the data JSON object
					// psi and steering angle in radians
					// speed in mph
					vector<double> ptsx = j[1]["ptsx"];
					vector<double> ptsy = j[1]["ptsy"];
					double px = j[1]["x"];
					double py = j[1]["y"];
					double psi = j[1]["psi"];
					double v = j[1]["speed"];
					double steering_angle = j[1]["steering_angle"];
					double throttle = j[1]["throttle"];

					double latency = PROCESS_LATENCY + solver_latency;

					double px_current = px;
					double py_current = py;

					// move position/psi forward to account for latency
					double vms = v * 0.44704;            // v in m/s
					px = px + latency * vms * cos(psi);
					py = py + latency * vms * sin(psi);
					psi = psi - latency * vms * steering_angle / 2.67;    // Lf=2.67


					// convert waypoints into car space
					size_t num_waypoints = ptsx.size();
					Eigen::VectorXd wp_x(num_waypoints);
					Eigen::VectorXd wp_y(num_waypoints);
					double cos_psi = cos(-psi);  // psi reversed
					double sin_psi = sin(-psi);
					for (int i = 0; i < num_waypoints; ++i) {
						double dx, dy;
						dx = ptsx[i] - px;
						dy = ptsy[i] - py;
						wp_x[i] = dx * cos_psi - dy * sin_psi;
						wp_y[i] = dy * cos_psi + dx * sin_psi;
					}

					// fit 3rd degree poly to waypoints
					Eigen::VectorXd coeffs = polyfit(wp_x, wp_y, 3);

					// prepare state for solver
					Eigen::VectorXd state(6);
					state << 0,                // px
						  0,                   // py
						  0,                   // psi
						  v,                   // speed
						  coeffs[0],           // CTE
						  -atan(coeffs[1]);    // ePsi


					// containers for MPC predicted points
					vector<double> mpc_x_vals;
					vector<double> mpc_y_vals;

					// solve for steering angle and throttle
					vector<double> solution = mpc.Solve(state, coeffs, mpc_x_vals, mpc_y_vals);

					double steer_value = solution[6];
					double throttle_value = solution[7];

					// Prepare control packet to send to simulator
					json msgJson;

					msgJson["steering_angle"] = steer_value;	// limited to -25..25 degrees in solver
					msgJson["throttle"] = throttle_value;

					if (visualize) {
						// predicted values from MPC (green line)
						msgJson["mpc_x"] = mpc_x_vals;
						msgJson["mpc_y"] = mpc_y_vals;

						// evaluate waypoint poly (yellow line)
						vector<double> next_x(N_VIS_POINTS);
						vector<double> next_y(N_VIS_POINTS);
						const double vis_point_distance = 5;
						for (size_t i = 1; i <= N_VIS_POINTS; ++i) {
							next_x[i - 1] = vis_point_distance * i;
							next_y[i - 1] = polyeval(coeffs, vis_point_distance * i);
						}
						msgJson["next_x"] = next_x;
						msgJson["next_y"] = next_y;
					} else {
						msgJson["mpc_x"] = "";
						msgJson["mpc_y"] = "";
						msgJson["next_x"] = "";
						msgJson["next_y"] = "";
					}

					auto msg = "42[\"steer\"," + msgJson.dump() + "]";

					// calculate solver duration for this step
					auto current_time = high_resolution_clock::now();
					duration<double, std::milli> solver_duration = current_time - start_time;
					double solver_duration_sec = 0.001 * solver_duration.count();

					// add solver latency and current position to collected data
					solution.push_back(solver_duration_sec);
					solution.push_back(px_current);
					solution.push_back(py_current);

					// collect data
					if (!is_lap_finished) {
						data_collected.push_back(solution);
					}

					// update solver latency
					solver_latency = LERP(solver_latency, solver_duration_sec, 0.25);

					// get distance from end of track (one lap of data collected)
					double dist_from_end = dist_end_track(px_current, py_current);

					//cout << "solver latency: " << solver_latency << endl;
					//cout << "px:" << px_current << endl;
					//cout << "py:" << py_current << endl;
					//cout << "distance from end of track: " << dist_from_end << endl;

					// if close to waypoint at end of first lap, write collected data to csv file
					if (!is_lap_finished && dist_from_end < 8) {
						//cout << "-------------- end of track -----------------" << endl;
						is_lap_finished = true;      // stop collecting data
						write_collected_data();
					}


					this_thread::sleep_for(chrono::milliseconds(int(PROCESS_LATENCY * 1000)));
#ifdef USE_LATEST_UWS
					ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
#ifdef USE_LATEST_UWS
				ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse * res, uWS::HttpRequest req, char *data,
	size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

#ifdef USE_LATEST_UWS
	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
	char *message, size_t length) {
		ws->close();
		std::cout << "Disconnected" << std::endl;
	});

#else
	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
	char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});
#endif

	auto host = "127.0.0.1";
	int port = 4567;
	if (h.listen(host, port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
