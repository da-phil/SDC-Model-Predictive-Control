#ifndef _TOOLS_HPP
#define _TOOLS_HPP

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


// Efficient moving average implementation, 
// avoiding costly O(N) traversal to calculate the total sum.
// https://stackoverflow.com/questions/10990618/calculate-rolling-moving-average-in-c#10990893
template <typename T, typename Total, int N>
class Moving_Average
{
  public:
    Moving_Average() : num_samples_(0), total_(0) { }

    void operator()(T sample) {
        if (num_samples_ < N) {
            samples_[num_samples_++] = sample;
            total_ += sample;
        } else {
            T& oldest = samples_[num_samples_++ % N];
            total_ += sample - oldest;
            oldest = sample;
        }
    }

    operator T() const {
    	return total_ / std::min(num_samples_, N);
    }

  private:
    T samples_[N];
    int num_samples_;
    Total total_;
};

#endif  // _TOOLS_HPP