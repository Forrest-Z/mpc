#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>


using CppAD::AD;


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


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}


// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
double Lf() { return 0.325; }

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
double ref_v() { return 90; }

// Delta constraint (on the steering angle)
double delta_constraint() { return 25; }


class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    Params m_params;
    Indexes m_indexes;

    FG_eval(Eigen::VectorXd coeffs, const Params & params, const Indexes & indexes)
        : m_params(params), m_indexes(indexes)
    {
        this->coeffs = coeffs;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {
        // The cost is stored in the first element of fg.
        // Any additions to the cost should be added to fg[0].
        fg[0] = 0;

        // The part of the cost based on the reference state.
        for (size_t t = 0; t < m_params.steps_ahead; t++) {
            fg[0] += 4000 * CppAD::pow(vars[m_indexes.cte_start + t], 2);
            fg[0] += 4000 * CppAD::pow(vars[m_indexes.epsi_start + t], 2);
            fg[0] += 2 * CppAD::pow(vars[m_indexes.v_start + t] - ref_v(), 2);
        }

        // Minimize the use of actuators.
        for (size_t t = 0; t < m_params.steps_ahead - 1; t++) {
            fg[0] += CppAD::pow(vars[m_indexes.delta_start + t], 2);
            fg[0] += CppAD::pow(vars[m_indexes.a_start + t], 2);
        }

        // Minimize the value gap between sequential actuations.
        for (size_t t = 0; t < m_params.steps_ahead - 2; t++) {
            fg[0] += 150 * CppAD::pow(vars[m_indexes.delta_start + t + 1] - vars[m_indexes.delta_start + t], 2);
            fg[0] += 5 * CppAD::pow(vars[m_indexes.a_start + t + 1] - vars[m_indexes.a_start + t], 2);
        }

        //
        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + m_indexes.x_start] = vars[m_indexes.x_start];
        fg[1 + m_indexes.y_start] = vars[m_indexes.y_start];
        fg[1 + m_indexes.psi_start] = vars[m_indexes.psi_start];
        fg[1 + m_indexes.v_start] = vars[m_indexes.v_start];
        fg[1 + m_indexes.cte_start] = vars[m_indexes.cte_start];
        fg[1 + m_indexes.epsi_start] = vars[m_indexes.epsi_start];

        // The rest of the constraints
        for (size_t t = 1; t < m_params.steps_ahead; t++) {
            // The state at time t+1 .
            AD<double> x1 = vars[m_indexes.x_start + t];
            AD<double> y1 = vars[m_indexes.y_start + t];
            AD<double> psi1 = vars[m_indexes.psi_start + t];
            AD<double> v1 = vars[m_indexes.v_start + t];
            AD<double> cte1 = vars[m_indexes.cte_start + t];
            AD<double> epsi1 = vars[m_indexes.epsi_start + t];

            // The state at time t.
            AD<double> x0 = vars[m_indexes.x_start + t - 1];
            AD<double> y0 = vars[m_indexes.y_start + t - 1];
            AD<double> psi0 = vars[m_indexes.psi_start + t - 1];
            AD<double> v0 = vars[m_indexes.v_start + t - 1];
            AD<double> cte0 = vars[m_indexes.cte_start + t - 1];
            AD<double> epsi0 = vars[m_indexes.epsi_start + t - 1];

            // Only consider the actuation at time t.
            AD<double> delta0 = vars[m_indexes.delta_start + t - 1];
            AD<double> a0 = vars[m_indexes.a_start + t - 1];

            AD<double> f0 = coeffs[0] +
                            coeffs[1] * x0 +
                            coeffs[2] * x0 * x0 +
                            coeffs[3] * x0 * x0 * x0;
            AD<double> psides0 = CppAD::atan(
                    coeffs[1] +
                    coeffs[2] * 2 * x0 +
                    coeffs[3] * 3 * x0 * x0
            );

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // Recall the equations for the model:
            // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
            // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
            // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
            // v_[t+1] = v[t] + a[t] * dt
            // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
            fg[1 + m_indexes.x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * m_params.dt);
            fg[1 + m_indexes.y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * m_params.dt);
            // "... - (psi0 ..." in contrast to the quizzes
            fg[1 + m_indexes.psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf() * m_params.dt);
            fg[1 + m_indexes.v_start + t] = v1 - (v0 + a0 * m_params.dt);
            fg[1 + m_indexes.cte_start + t] = cte1 - (f0 - y0 + (v0 * CppAD::sin(epsi0) * m_params.dt));
            // "... - v0 ..." in contrast to the quizzes
            fg[1 + m_indexes.epsi_start + t] = epsi1 - (psi0 - psides0 - v0 * delta0 / Lf() * m_params.dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC(const Params & params) : m_params(params) {
    m_indexes.x_start = 0;
    m_indexes.y_start = m_indexes.x_start + params.steps_ahead;
    m_indexes.psi_start = m_indexes.y_start + params.steps_ahead;
    m_indexes.v_start = m_indexes.psi_start + params.steps_ahead;
    m_indexes.cte_start = m_indexes.v_start + params.steps_ahead;
    m_indexes.epsi_start = m_indexes.cte_start + params.steps_ahead;
    m_indexes.delta_start = m_indexes.epsi_start + params.steps_ahead;
    m_indexes.a_start = m_indexes.delta_start + params.steps_ahead - 1;
}

MPC::~MPC() {}

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    size_t n_vars = m_params.steps_ahead * 6 + (m_params.steps_ahead - 1) * 2;
    size_t n_constraints = m_params.steps_ahead * 6;

    // Initial value of the independent variables.
    // Should be 0 besides initial state.
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++)
        vars[i] = 0;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    for (size_t i = 0; i < m_indexes.delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // BEGIN: CONSTRAINTS ON THE ACTUATORS
    for (size_t i = m_indexes.delta_start; i < m_indexes.a_start; i++) {
        // 1 degree = 0.017453 radians
        vars_lowerbound[i] = -0.017453 * delta_constraint();
        vars_upperbound[i] = 0.017453 * delta_constraint();
    }
    for (size_t i = m_indexes.a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }
    // END: CONSTRAINTS ON THE ACTUATORS

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (size_t i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[m_indexes.x_start] = x;
    constraints_lowerbound[m_indexes.y_start] = y;
    constraints_lowerbound[m_indexes.psi_start] = psi;
    constraints_lowerbound[m_indexes.v_start] = v;
    constraints_lowerbound[m_indexes.cte_start] = cte;
    constraints_lowerbound[m_indexes.epsi_start] = epsi;

    constraints_upperbound[m_indexes.x_start] = x;
    constraints_upperbound[m_indexes.y_start] = y;
    constraints_upperbound[m_indexes.psi_start] = psi;
    constraints_upperbound[m_indexes.v_start] = v;
    constraints_upperbound[m_indexes.cte_start] = cte;
    constraints_upperbound[m_indexes.epsi_start] = epsi;

    // Object that computes objective and constraints
    FG_eval fg_eval(coeffs, m_params, m_indexes);

    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
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
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result <Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= (solution.status == CppAD::ipopt::solve_result<Dvector>::success);

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << "\n";

    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.
    return {
            solution.x[m_indexes.delta_start], solution.x[m_indexes.a_start],
            solution.x[m_indexes.x_start], solution.x[m_indexes.y_start],
            solution.x[m_indexes.x_start + 1], solution.x[m_indexes.y_start + 1],
            solution.x[m_indexes.x_start + 2], solution.x[m_indexes.y_start + 2],
            solution.x[m_indexes.x_start + 3], solution.x[m_indexes.y_start + 3],
            solution.x[m_indexes.x_start + 4], solution.x[m_indexes.y_start + 4],
            solution.x[m_indexes.x_start + 5], solution.x[m_indexes.y_start + 5],
    };
}
