import math
import numpy as np


class StuckController():
    # This is a modified version of the old lane following controller
    # used for rescuing a bot from a distress situation

    def __init__(self, k_P, k_I, c1, c2):
        """Initialize stuck_controller
        Nicely working parameters in comments of source code

        """

        # C matrix of LTI system
        self.c1 = c1 # -5.0
        self.c2 = c2 # 1.0

        # Gains for controller
        self.k_P = k_P # 6.0
        self.k_I = k_I # 0.1

        # Initialize integral
        self.C_I = 0

    def getControlOutput(self, d_est, phi_est, d_ref, phi_ref, v_ref, dt_last):
        """Get output of stuck_controller

        Note:
            Requires the Tile Object
            TODO: Unnecessary to pass map, could use class variable

        Args:
            d_est: Estimation of position (x, y) [m]
            phi_est: Estimation of angle of bot [deg]
            d_ref: Reference position (x,y) [m]
            phi_ref: Reference angle [deg]
            v_ref: Reference of velocity [m/s]
            dt_last: Time it took from last processing to current [s]

        Returns:
            v_out: velocity of Duckiebot [m/s]
            omega_out: angular velocity of Duckiebot [rad/s]

        """

        # Convert phi_est to rad
        phi_est = phi_est/180*math.pi
        phi_ref = phi_ref/180*math.pi

        # Calculate the distance error and account for sign
        delta_d = math.sqrt((d_ref[0] - d_est[0])**2 + (d_ref[1] - d_est[1])**2)
        if sum(d_ref) < sum(d_est): delta_d *= -1
        # Calculate the heading error and account for sign
        delta_phi = phi_ref - phi_est
        if abs(delta_phi) > math.pi: delta_phi = -math.copysign(1.0, 2*math.pi-abs(delta_phi))
        # Calculate the output error ref - y of linear system
        err = self.c1 * delta_d + self.c2 * delta_phi
        print("delta_d = {}, delta_phi = {}, err = {}".format(delta_d, delta_phi, err))

        # PI-Controller
        C_P = self.k_P * err
        omega = C_P + self.C_I

        # Calculate new value of integral
        self.C_I = self.C_I + dt_last * self.k_I * err

        # Return values
        omega_out = omega
        v_out = -v_ref
        return (v_out, omega_out)

    def updateParams(self, k_P, k_I, u_sat, k_t, c1, c2):
        """Update Parameters of stuck_controller
        Nicely working parameters in comments of source code

        """

        self.k_P = k_P
        self.k_I = k_I
        self.c1 = c1
        self.c2 = c2