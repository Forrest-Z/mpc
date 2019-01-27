#pragma once

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);


double polyeval(Eigen::VectorXd coeffs, double x);


// Defined constants as functions
double Lf();

double ref_v();

double delta_constraint();

class MPC {
public:
    MPC(size_t steps_ahead, double dt);

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

private:
    size_t steps_ahead;
    double dt;

    // Indexes
    size_t x_start;
    size_t y_start;
    size_t psi_start;
    size_t v_start;
    size_t cte_start;
    size_t epsi_start;
    size_t delta_start;
    size_t a_start;
};
