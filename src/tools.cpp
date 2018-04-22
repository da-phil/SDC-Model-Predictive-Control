#include <Eigen/Core>
#include <Eigen/QR>

#include "tools.hpp"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}

double deg2rad(double x) {
  return x * pi() / 180;
}

double rad2deg(double x) {
  return x * 180 / pi();
}

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

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
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


// 2D affine transformation for a list of points 
void Tranform2d(std::vector<double>       &dst_x,
                std::vector<double>       &dst_y,
                const std::vector<double> &src_x,
                const std::vector<double> &src_y,
                double translation_x, double translation_y, double psi) {

  // make sure destination vectors have same length as source vectors
  dst_x.resize(src_x.size());
  dst_y.resize(src_y.size());

  for (int idx = 0; idx < (int) src_x.size(); idx++) {
    double dx = src_x[idx] - translation_x;
    double dy = src_y[idx] - translation_y;
    dst_x[idx] = dx*cos(psi) - dy*sin(psi);
    dst_y[idx] = dx*sin(psi) + dy*cos(psi);
  }
}