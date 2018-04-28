#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include <Eigen/Core>

#include "tools.hpp"


using namespace std;
using CppAD::AD;

class FG_eval {
	public:
		typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

		FG_eval(Eigen::VectorXd coeffs, const double steering_smoothness = 500) :
		    coeffs(coeffs), steering_smoothness(steering_smoothness) { }

		void operator()(ADvector& fg, const ADvector& vars);

	private:
	  // Fitted polynomial coefficients
	  Eigen::VectorXd coeffs;
	  double steering_smoothness;
	  Moving_Average<double, double, 20> avg_radius;
};


class MPC {
 public:
  // Default constructor
  MPC() : max_steer_rad(0.436332), min_accel(-0.5), max_accel(1.0), steering_smoothness(500) {}

	// Parameterized constructor
  MPC(double max_steer_rad, double min_accel, double max_accel, double steering_smoothness) :
  	max_steer_rad(max_steer_rad), min_accel(min_accel), max_accel(max_accel), steering_smoothness(steering_smoothness) {}

  ~MPC() {}

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state,
											 Eigen::VectorXd coeffs,
											 std::vector<double>& coords_x,
											 std::vector<double>& coords_y,
											 bool& success);

 private:
  const double max_steer_rad;
  const double min_accel;
  const double max_accel;
  double steering_smoothness;
};

#endif /* MPC_H */
