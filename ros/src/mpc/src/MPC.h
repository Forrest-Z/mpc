#pragma once

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


Eigen::VectorXd polyfit(const Eigen::VectorXd & xvals, const Eigen::VectorXd & yvals, int order);


double polyeval(const Eigen::VectorXd coeffs, double x);


double polyeval_diff(const Eigen::VectorXd coeffs, double x);


struct Params {
    size_t steps_ahead;
    double dt;
    double ref_v;

    double latency;

    double cte_coeff;
    double epsi_coeff;
    double speed_coeff;
    double steer_coeff;

    double consec_speed_coeff;
    double consec_steer_coeff;

    bool debug;
};


struct Indexes {
    size_t x_start;
    size_t y_start;
    size_t psi_start;

    size_t cte_start;
    size_t epsi_start;
    size_t delta_start;
    size_t v_start;
};


// Defined constants as functions
double Lf();

double delta_constraint();

class MPC {
public:
    MPC(const Params & p);

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

private:
    Params m_params;
    Indexes m_indexes;

    ///* Upper bound
    const double SPEED_UPPERBOUND = 2.0;
};
