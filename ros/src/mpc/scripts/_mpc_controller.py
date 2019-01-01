#!/usr/bin/env python
import numpy as np
from scipy.optimize import minimize
from scipy.interpolate import splprep, splev
import sympy as sym
from sympy.tensor.array import derive_by_array
sym.init_printing()

import config


class _EqualityConstraints(object):
    """Class for storing equality constraints in the MPC controller."""

    def __init__(self, N, state_vars):
        self.dict = {}
        for symbol in state_vars:
            self.dict[symbol] = N*[None]

    def __getitem__(self, key):
        return self.dict[key]

    def __setitem__(self, key, value):
        self.dict[key] = value


class MPCController:
    def __init__(self, target_speed, steps_ahead, dt):
        self.target_speed = target_speed
        self.state_vars = ('x', 'y', 'v', 'psi', 'cte', 'epsi')

        self.steps_ahead = steps_ahead
        self.dt = dt

        # Cost function coefficients
        # TODO(MD): take outside, preferably to a config
        self.cte_coeff = config.CTE_COEFF
        self.epsi_coeff = config.EPSI_COEF
        self.speed_coeff = config.SPEED_COEFF
        self.acc_coeff = config.ACC_COEFF
        self.steer_coeff = config.STEER_COEFF
        self.consec_acc_coeff = config.CONSEC_ACC_COEFF
        self.consec_steer_coeff = config.CONSEC_STEER_COEFF

        # Front wheel L
        self.Lf = config.Lf

        # How the polynomial fitting the desired curve is fitted
        self.poly_degree = 3

        # Bounds for the optimizer
        self.bounds = (
            6*self.steps_ahead * [(None, None)]
            + self.steps_ahead * [(-config.THROTTLE_LOWER_BOUND, config.THROTTLE_UPPER_BOUND)]
            + self.steps_ahead * [(-config.STEER_BOUND, config.STEER_BOUND)]
        )

        # State 0 placeholder
        num_vars = (len(self.state_vars) + 2)  # State variables and two actuators
        self.state0 = np.zeros(self.steps_ahead*num_vars)

        # Lambdify and minimize stuff
        self.evaluator = 'numpy'
        self.tolerance = config.TOLERANCE
        self.cost_func, self.cost_grad_func, self.constr_funcs = self.get_func_constraints_and_bounds()

        # To keep the previous state
        self.steer = None
        self.throttle = None
        self.next_pos = None

    def get_func_constraints_and_bounds(self):
        """The most important method of this class, defining the MPC's cost
        function and constraints.
        """
        # Polynomial coefficients will also be symbolic variables
        poly = self.create_array_of_symbols('poly', self.poly_degree+1)

        # Initialize the initial state
        x_init = sym.symbols('x_init')
        y_init = sym.symbols('y_init')
        psi_init = sym.symbols('psi_init')
        v_init = sym.symbols('v_init')
        cte_init = sym.symbols('cte_init')
        epsi_init = sym.symbols('epsi_init')

        init = (x_init, y_init, psi_init, v_init, cte_init, epsi_init)

        # State variables
        x = self.create_array_of_symbols('x', self.steps_ahead)
        y = self.create_array_of_symbols('y', self.steps_ahead)
        psi = self.create_array_of_symbols('psi', self.steps_ahead)
        v = self.create_array_of_symbols('v', self.steps_ahead)
        cte = self.create_array_of_symbols('cte', self.steps_ahead)
        epsi = self.create_array_of_symbols('epsi', self.steps_ahead)

        # Actuators
        a = self.create_array_of_symbols('a', self.steps_ahead)
        delta = self.create_array_of_symbols('delta', self.steps_ahead)

        vars_ = sum([
            # Symbolic arrays (but NOT actuators)
            x, y, psi, v, cte, epsi,

            # Symbolic arrays (actuators)
            a, delta,
        ], ())

        cost = 0
        for t in range(self.steps_ahead):
            cost += (
                # Reference state penalties
                self.cte_coeff * cte[t]**2
                + self.epsi_coeff * epsi[t]**2 +
                + self.speed_coeff * (v[t] - self.target_speed)**2

                # # Actuator penalties
                + self.acc_coeff * a[t]**2
                + self.steer_coeff * delta[t]**2
            )

        # Penalty for differences in consecutive actuators
        for t in range(self.steps_ahead-1):
            cost += (
                self.consec_acc_coeff * (a[t+1] - a[t])**2
                + self.consec_steer_coeff * (delta[t+1] - delta[t])**2
            )

        # Initialize constraints
        eq_constr = _EqualityConstraints(self.steps_ahead, self.state_vars)
        eq_constr['x'][0] = x[0] - x_init
        eq_constr['y'][0] = y[0] - y_init
        eq_constr['psi'][0] = psi[0] - psi_init
        eq_constr['v'][0] = v[0] - v_init
        eq_constr['cte'][0] = cte[0] - cte_init
        eq_constr['epsi'][0] = epsi[0] - epsi_init

        for t in range(1, self.steps_ahead):
            curve = sum(poly[-(i+1)] * x[t-1]**i for i in range(len(poly)))
            # The desired psi is equal to the derivative of the polynomial curve at
            #  point x[t-1]
            psides = sum(poly[-(i+1)] * i*x[t-1]**(i-1) for i in range(1, len(poly)))

            eq_constr['x'][t] = x[t] - (x[t-1] + v[t-1] * sym.cos(psi[t-1]) * self.dt)
            eq_constr['y'][t] = y[t] - (y[t-1] + v[t-1] * sym.sin(psi[t-1]) * self.dt)
            eq_constr['psi'][t] = psi[t] - (psi[t-1] - v[t-1] * delta[t-1] / self.Lf * self.dt)
            eq_constr['v'][t] = v[t] - (v[t-1] + a[t-1] * self.dt)
            eq_constr['cte'][t] = cte[t] - (curve - y[t-1] + v[t-1] * sym.sin(epsi[t-1]) * self.dt)
            eq_constr['epsi'][t] = epsi[t] - (psi[t-1] - psides - v[t-1] * delta[t-1] / self.Lf * self.dt)

        # Generate actual functions from
        cost_func = self.generate_fun(cost, vars_, init, poly)
        cost_grad_func = self.generate_grad(cost, vars_, init, poly)

        constr_funcs = []
        for symbol in self.state_vars:
            for t in range(self.steps_ahead):
                func = self.generate_fun(eq_constr[symbol][t], vars_, init, poly)
                grad_func = self.generate_grad(eq_constr[symbol][t], vars_, init, poly)
                constr_funcs.append(
                    {'type': 'eq', 'fun': func, 'jac': grad_func, 'args': None},
                )

        return cost_func, cost_grad_func, constr_funcs

    def _find_closest(self, pts_2D, position):
        dists = np.linalg.norm(pts_2D - position, axis=1)
        return np.argmin(dists), dists

    def control(self, pts_2D, v, position, psi):
        which_closest, _ = self._find_closest(pts_2D, position)

        indeces = which_closest + config.STEPS_POLY*np.arange(self.poly_degree+1)
        indeces = indeces % pts_2D.shape[0]
        pts = pts_2D[indeces]

        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)

        pts_car = MPCController.transform_into_cars_coordinate_system(pts, position, cos_psi, sin_psi)

        poly = np.polyfit(pts_car[:, 0], pts_car[:, 1], self.poly_degree)

        cte = poly[-1]
        epsi = -np.arctan(poly[-2])

        init = (0, 0, 0, v, cte, epsi) + tuple(poly)
        self.state0 = self.get_state0(0, 0, 0, v, cte, epsi, self.steer, self.throttle, poly)
        result = self.minimize_cost(self.bounds, self.state0, init)


        if 'success' in result.message:
            next_x = result.x[:self.steps_ahead]
            next_y = result.x[self.steps_ahead:2*self.steps_ahead]
            self.next_pos = np.c_[next_x, next_y]
            self.steer = result.x[-self.steps_ahead]
            self.throttle = result.x[-2*self.steps_ahead]
        else:
            print('Unsuccessful optimization')

        return {
            'steer': self.steer,
            'throttle': self.throttle,
            'cost': result.fun,
            'next_pos': self.next_pos,
        }

    def get_state0(self, x, y, psi, v, cte, epsi, a, delta, poly):
        a = a or 0
        delta = delta or 0
        self.state0[:self.steps_ahead] = x
        self.state0[self.steps_ahead:2*self.steps_ahead] = np.polyval(poly, x)
        self.state0[2*self.steps_ahead:3*self.steps_ahead] = psi
        self.state0[3*self.steps_ahead:4*self.steps_ahead] = v
        self.state0[4*self.steps_ahead:5*self.steps_ahead] = cte
        self.state0[5*self.steps_ahead:6*self.steps_ahead] = epsi
        self.state0[6*self.steps_ahead:7*self.steps_ahead] = a
        self.state0[7*self.steps_ahead:8*self.steps_ahead] = delta
        return self.state0

    def generate_fun(self, symb_fun, vars_, init, poly):
        '''This function generates a function of the form `fun(x, *args)` because
        that's what the scipy `minimize` API expects (if we don't want to minimize
        over certain variables, we pass them as `args`)
        '''
        args = init + poly
        return sym.lambdify((vars_,) + args, symb_fun, self.evaluator)
        # Equivalent to (but faster than):
        # func = sym.lambdify(vars_+init+poly, symb_fun, evaluator)
        # return lambda x, *args: func(*np.r_[x, args])

    def generate_grad(self, symb_fun, vars_, init, poly):
        args = init + poly
        return sym.lambdify(
            (vars_,) + args,
            derive_by_array(symb_fun, vars_+args)[:len(vars_)],
            self.evaluator
        )
        # Equivalent to (but faster than):
        # cost_grad_funcs = [
        #     generate_fun(symb_fun.diff(var), vars_, init, poly)
        #     for var in vars_
        # ]
        # return lambda x, *args: [
        #     grad_func(np.r_[x, args]) for grad_func in cost_grad_funcs
        # ]

    def minimize_cost(self, bounds, x0, init):
        # TODO: this is a bit retarded, but hey -- that's scipy API's fault ;)
        for constr_func in self.constr_funcs:
            constr_func['args'] = init

        return minimize(
            fun=self.cost_func,
            x0=x0,
            args=init,
            jac=self.cost_grad_func,
            bounds=bounds,
            constraints=self.constr_funcs,
            method='SLSQP',
            tol=self.tolerance,
        )

    @staticmethod
    def create_array_of_symbols(str_symbol, N):
        return sym.symbols('{symbol}0:{N}'.format(symbol=str_symbol, N=N))

    @staticmethod
    def transform_into_cars_coordinate_system(pts, position, cos_psi, sin_psi):
        diff = pts - position
        pts_car = np.zeros_like(diff)
        pts_car[:, 0] =  cos_psi * diff[:, 0] + sin_psi * diff[:, 1]
        pts_car[:, 1] = -sin_psi * diff[:, 0] + cos_psi * diff[:, 1]
        return pts_car


if __name__ == '__main__':
    # For testing purposes
    mpc_controller = MPCController(
        target_speed=10,
        steps_ahead=10,
        dt=0.1
    )

    # Values for testing
    pts_2D = np.array([
        [-34.41720415,  60.01667464],
        [-38.59669945,  56.94990656],
        [-43.33353477,  54.53909335],
        [-48.72895121,  52.83095266]
    ])
    orientation = (0.7072, 0.7072)
    psi = np.arctan2(orientation[1], orientation[0])
    location = (-4.33, 25.15)
    curr_speed = 12

    steer, throttle, cost = mpc_controller.control(pts_2D, curr_speed, location, psi)
    print('steer: {:.2f}, throttle: {:.2f}'.format(steer, throttle))
