#!/usr/bin/env python
from __future__ import print_function
import numpy as np


class Storage:
    def __init__(self):
        self.curr = None
        self.prev = None

    def step(self, value):
        self.prev = self.curr if self.curr is not None else value
        self.curr = value

    @property
    def derivative(self):
        return self.curr - self.prev

    @property
    def integral(self):
        # TODO: finish this
        return 0



class PIDController:
    def __init__(
        self,
        debug,
        Kp_cte, Ki_cte, Kd_cte,
        Kp_ePsi, Ki_ePsi, Kd_ePsi,
        Lf,
        poly_degree, poly_steps,
        logg
    ):
        self.debug = debug

        self.kp_cte = Kp_cte
        self.ki_cte = Ki_cte
        self.kd_cte = Kd_cte

        self.kp_ePsi = Kp_ePsi
        self.ki_ePsi = Ki_ePsi
        self.kd_ePsi = Kd_ePsi

        self.Lf = Lf

        self.poly_degree = poly_degree
        self.poly_steps = poly_steps

        self.logg = logg

        # To keep the previous state
        self.cte_storage = Storage()
        self.ePsi_storage = Storage()

        self.next_pos = None

    def _find_closest(self, pts_2D, position):
        dists = np.linalg.norm(pts_2D - position, axis=1)
        return np.argmin(dists), dists

    def control(self, pts_2D, v, position, psi):
        which_closest, _ = self._find_closest(pts_2D, position)

        which_closest_shifted = which_closest - 5
        # NOTE: `which_closest_shifted` might become < 0, but the modulo operation below fixes that
        indeces = which_closest_shifted + np.arange(self.poly_steps+1)
        indeces = indeces % pts_2D.shape[0]
        pts = pts_2D[indeces]

        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        pts_car = PIDController.transform_into_cars_coordinate_system(pts, position, cos_psi, sin_psi)

        poly = np.polyfit(pts_car[:, 0], pts_car[:, 1], self.poly_degree)

        cte = poly[-1]
        ePsi = -np.arctan(poly[-2])
        self.logg('cte={:.2f}, ePsi={:.2f}, psi={:.2f}'.format(cte, ePsi, psi))

        # This is where the magic happens
        self.cte_storage.step(cte)
        self.ePsi_storage.step(ePsi)
        steer = (
            self.kp_cte * self.cte_storage.curr
            + self.ki_cte * self.cte_storage.integral
            + self.kd_cte * self.cte_storage.derivative

            - self.kp_ePsi * self.ePsi_storage.curr
            - self.ki_ePsi * self.ePsi_storage.integral
            - self.kd_ePsi * self.ePsi_storage.derivative
        )

        x = np.arange(0, 2, 0.2)
        self.next_pos = np.c_[np.cos(steer)*x, np.sin(steer)*x]
        self.poly = np.c_[x, np.polyval(poly, x)]

        return {
            'steer': steer,
            'next_pos': self.next_pos,
            'poly': self.poly,
        }

    @staticmethod
    def transform_into_cars_coordinate_system(pts, position, cos_psi, sin_psi):
        diff = pts - position
        pts_car = np.zeros_like(diff)
        pts_car[:, 0] =  cos_psi * diff[:, 0] + sin_psi * diff[:, 1]
        pts_car[:, 1] = -sin_psi * diff[:, 0] + cos_psi * diff[:, 1]
        return pts_car
