import math
import numpy as np


class StuckController():
    # This is a modified version of the lane following controller
    # used for rescuing a bot from a stuck with infrastructure case

    def __init__(self, k_P, k_I, c1, c2):

        # C matrix of LTI system
        self.c1 = c1
        self.c2 = c2

        # Gains for controller
        self.k_P = k_P
        self.k_I = k_I

        # Variable for integral
        self.C_I = 0

    # Inputs:   d_est   Estimation of distance from lane center (positive when
    #                   offset to the left of driving direction) [m]
    #           phi_est Estimation of angle of bot (positive when angle to the
    #                   left of driving direction) [deg]
    #           d_ref   Reference of d (d_ref = 0) [m]
    #           v_ref   Reference of velocity [m/s]
    #           dt_last Time it took from last processing to current [s]

    # Output:   v_out       velocity of Duckiebot [m/s]
    #           omega_out   angular velocity of Duckiebot [rad/s]

    def getControlOutput(self, d_est, phi_est, d_ref, phi_ref, v_ref, dt_last):

        # Convert phi_est to rad
        phi_est = phi_est/180*math.pi

        # Calculate the output y
        ref =   (self.c1 * d_ref + self.c2 * phi_ref)
        y =     (self.c1 * d_est + self.c2 * phi_est)
        err = ref - y

        # PI-Controller
        C_P = self.k_P * err
        omega = C_P + self.C_I

        # Calculate new value of integral while considering the anti-windup
        self.C_I = self.C_I + dt_last * self.k_I * err

        # Declaring return values
        omega_out = omega
        v_out = -v_ref
        return (v_out, omega_out)

    def updateParams(self, k_P, k_I, u_sat, k_t, c1, c2):
        self.k_P = k_P
        self.k_I = k_I
        self.c1 = c1
        self.c2 = c2