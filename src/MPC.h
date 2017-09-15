#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// For converting back and forth between radians and degrees.

class MPC {

 public:
    
    static constexpr double max_delta = 25 * M_PI / 180.0;
    static constexpr double Lf = 2.67;
    
    MPC();
    
    virtual ~MPC();
    
    
    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
    double steeringValue() const { return steering_delta_; }
    double throttleValue() const { return a_; }
    
    std::vector<double> path_x;
    std::vector<double> path_y;
    
  private:
    double steering_delta_;
    double a_;
};

#endif /* MPC_H */
