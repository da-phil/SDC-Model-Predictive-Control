#include "MPC.h"

using CppAD::AD;


// Amount of timesteps used for MPC
int N = 10;
// Time delta between actuations
double dt = 0.1;


// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
const size_t x_start      = 0;
const size_t y_start      = x_start + N;
const size_t psi_start    = y_start + N;
const size_t v_start      = psi_start + N;
const size_t cte_start    = v_start + N;
const size_t epsi_start   = cte_start + N;
const size_t delta_start  = epsi_start + N;
const size_t a_start      = delta_start + N - 1;

// Number of independent variables for optimizer
// N timesteps => N - 1 actuations
// 6 state variables, 2 actuation variables
const size_t n_constraints = N*6;  
const size_t n_vars = n_constraints + (N-1)*2;



void FG_eval::operator()(ADvector& fg, const ADvector& vars) {
  double radius;
  const double max_radius = 1e3;
  double steering_cost_weight;

  // Calculate radius and catch division by zero
  if (fabs(coeffs[2]) > std::numeric_limits<double>::epsilon()) {
      // y(x)   = a*x^3 + b*x^2 + c*x + d
      // y'(x)  = 3*a*x^2 + 2*b*x + c,    y''(x) = 6*a*x + 2*b
      // coeffs = [d, c, b, a]
      // R(x)   = ((1 + f'(x)^2)^1.5) / abs(f''(x))
      //        = ((1 + (3*a*x^2 + 2*b*x +c)^2)^1.5 / abs(6*a*x+2*b)
      // R(0)   = ((1 + c^2)^1.5 / abs(2*b)
      radius = pow(1.0 + pow(coeffs[1], 2), 1.5) / fabs(2.*coeffs[2]);
      if (radius > max_radius)
        radius = max_radius;
  } else {
      radius = max_radius;
  }

  avg_radius(radius);
  steering_cost_weight = steering_smoothness * (max_radius / avg_radius);
  std::cout << "radius: " << avg_radius << ", " << 
               "steering_cost_weight: " << steering_cost_weight <<std::endl;

  // Reference State Cost
  // Define the cost related to the reference state and

  // The cost is stored is the first element of `fg`.
  // Any additions to the cost should be added to `fg[0]`.
  fg[0] = 0;

  // The part of the cost based on the deviation from the reference state:
  // - penalize increasing CTE
  // - penalize heading error (epsi)
  // - penalize deviation from reference velocity (ref_v)
  for (int t = 0; t < N; t++) {
    fg[0] += steering_cost_weight * CppAD::pow(vars[cte_start + t], 2);
    fg[0] += CppAD::pow(vars[epsi_start + t], 2);
    fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
  }

  // Minimize the use of actuators, for all sampling points except last one (N-1).
  for (int t = 0; t < N - 1; t++) {
    fg[0] += steering_cost_weight * CppAD::pow(vars[delta_start + t], 2);
    fg[0] += CppAD::pow(vars[a_start + t], 2);
  }

  // Minimize the value gap (derivative) between sequential actuations,
  // for all sampling points except the last two (N-2).
  for (int t = 0; t < N - 2; t++) {
    fg[0] += steering_cost_weight * CppAD::pow((vars[delta_start + t + 1] - vars[delta_start + t]), 2);
    fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
  }

  // Initial constraints
  //
  // We add 1 to each of the starting indices due to cost being located at index 0 of fg.
  // This bumps up the position of all the other values.
  fg[1 + x_start]     = vars[x_start];
  fg[1 + y_start]     = vars[y_start];
  fg[1 + psi_start]   = vars[psi_start];
  fg[1 + v_start]     = vars[v_start];
  fg[1 + cte_start]   = vars[cte_start];
  fg[1 + epsi_start]  = vars[epsi_start];

  // The rest of the constraints
  for (int t = 1; t < N; t++) {
    // The state at time t+1 .
    AD<double> x1     = vars[x_start + t];
    AD<double> y1     = vars[y_start + t];
    AD<double> psi1   = vars[psi_start + t];
    AD<double> v1     = vars[v_start + t];
    AD<double> cte1   = vars[cte_start + t];
    AD<double> epsi1  = vars[epsi_start + t];

    // The state at time t
    AD<double> x0     = vars[x_start + t - 1];
    AD<double> psi0   = vars[psi_start + t - 1];
    AD<double> v0     = vars[v_start + t - 1];
    AD<double> y0     = vars[y_start + t - 1];
    AD<double> cte0   = vars[cte_start + t - 1];
    AD<double> epsi0  = vars[epsi_start + t - 1];

    // Only consider the actuation at time t.
    AD<double> delta0 = vars[delta_start + t - 1];
    AD<double> a0     = vars[a_start + t - 1];

    // Evaluate polynomial
    AD<double> f0 = coeffs[0] +
                    coeffs[1]*x0 +
                    coeffs[2]*x0*x0 +
                    coeffs[3]*x0*x0*x0;
    AD<double> psides0 = CppAD::atan(    coeffs[1] +
                                     2.0*coeffs[2]*x0 +
                                     3.0*coeffs[3]*x0*x0);

    // The idea here is to constraint this value to be 0.
    //
    // NOTE: The use of `AD<double>` and use of `CppAD`!
    // This is also CppAD can compute derivatives and pass
    // these to the solver.
    fg[1 + x_start + t]     = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
    fg[1 + y_start + t]     = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
    fg[1 + psi_start + t]   = psi1 - (psi0 + v0 * delta0 / Lf * dt);  
    fg[1 + v_start + t]     = v1 - (v0 + a0 * dt);
    fg[1 + cte_start + t]   = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
    fg[1 + epsi_start + t]  = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
  }
}


//
// MPC class definition implementation.
//
vector<double> MPC::Solve(Eigen::VectorXd state,
                          Eigen::VectorXd coeffs,
                          std::vector<double>& coords_x,
                          std::vector<double>& coords_y,
                          bool& success) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (int i = 0; i < (int) n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial state variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;


  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < (int) delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_start; i < (int) a_start; i++) {
    vars_lowerbound[i] = -max_steer_rad;
    vars_upperbound[i] =  max_steer_rad;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < (int) n_vars; i++) {
    vars_lowerbound[i] = min_accel;
    vars_upperbound[i] = max_accel;
  }

  // Lower and upper limits for constraints.
  // All of these should be 0 except the initial state indices.
  Dvector constr_lowerbound(n_constraints);
  Dvector constr_upperbound(n_constraints);
  for (int i = 0; i < (int) n_constraints; i++) {
    constr_lowerbound[i] = constr_upperbound[i] = 0;
  }
  constr_lowerbound[x_start]    = constr_upperbound[x_start]    = x;
  constr_lowerbound[y_start]    = constr_upperbound[y_start]    = y;
  constr_lowerbound[psi_start]  = constr_upperbound[psi_start]  = psi;
  constr_lowerbound[v_start]    = constr_upperbound[v_start]    = v;
  constr_lowerbound[cte_start]  = constr_upperbound[cte_start]  = cte;
  constr_lowerbound[epsi_start] = constr_upperbound[epsi_start] = epsi;


  // Options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time   1.0\n";

  // Place to return solution into
  CppAD::ipopt::solve_result<Dvector> solution;

  // Object that computes objective and constraints
  FG_eval fg_eval(ref_v, Lf, coeffs, steering_smoothness);

  // Solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars,
                                        vars_lowerbound, vars_upperbound,
                                        constr_lowerbound, constr_upperbound,
                                        fg_eval, solution);

  // Check if optimization was successful
  success = (solution.status == CppAD::ipopt::solve_result<Dvector>::success);

  // Cost
  //auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;

  // Make sure vectors are large enough already
  coords_x.resize(N);
  coords_y.resize(N); 
  for (int t = 0; t < N; t++) {
        coords_x[t] = solution.x[x_start + t];
        coords_y[t] = solution.x[y_start + t];
  }

  // Return the first actuator values.
  return {solution.x[delta_start], solution.x[a_start]};
}
