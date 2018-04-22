#include <vector>

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();

double deg2rad(double x);
double rad2deg(double x);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

// 2D affine transformation for a list of points
void Tranform2d(std::vector<double>       &dst_x,
                std::vector<double>       &dst_y,
                const std::vector<double> &src_x,
                const std::vector<double> &src_y,
                double translation_x, double translation_y, double psi);