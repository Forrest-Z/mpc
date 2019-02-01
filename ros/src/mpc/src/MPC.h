#pragma once

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


Eigen::VectorXd polyfit(Eigen::VectorXd & xvals, Eigen::VectorXd & yvals, int order);


double polyeval(Eigen::VectorXd coeffs, double x);


struct Params {
    size_t steps_ahead;
    double dt;

    double latency;

    double cte_coeff;
    double epsi_coeff;
    double speed_coeff;
    double acc_coeff;
    double steer_coeff;

    double consec_acc_coeff;
    double consec_steer_coeff;

    bool debug;
};


struct Indexes {
    size_t x_start;
    size_t y_start;
    size_t psi_start;
    size_t v_start;
    size_t cte_start;
    size_t epsi_start;
    size_t delta_start;
    size_t a_start;
};


// Defined constants as functions
double Lf();

double ref_v();

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
};
