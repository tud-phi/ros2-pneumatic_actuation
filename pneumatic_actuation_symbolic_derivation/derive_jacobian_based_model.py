from math import pi
import matplotlib.pyplot as plt
import sympy
from sympy.geometry import line
from sympy.plotting import plot

"""
This script derives the symbolic model of the pneumatic actuation system for a planar one segment robot.
"""

class JacobianBasedPneumaticActuationModel:
    def __init__(self, L_0_i: float = 1., d_i: float = 1., r_p: float = 1.) -> None:
        # Parameters
        self.L_0_i = L_0_i # Unextended length of the arm segment
        self.d_i = d_i # diameter of robot used for kinematic description
        self.r_p = r_p # radius of the center of pressure of the chamber
        ## Perpendicular force at the tip of the segment
        self.f_i = 1 # Scalar force acting at the tip of the segment [N] perpendicular to center-line

        # Symbolic variables
        self.Delta_i, self.delta_L_i, self.s, self.r = sympy.symbols('Delta_i delta_L_i s r')
        self.q = sympy.Array([self.Delta_i, self.delta_L_i])

        # Scale q with s
        self.q_s = self.s * self.q
        self.L_0_i_s = self.s * self.L_0_i

        self.R = sympy.Matrix([[sympy.cos(self.q_s[0]/self.d_i), -sympy.sin(self.q_s[0]/self.d_i)],
                               [sympy.sin(self.q_s[0]/self.d_i), sympy.cos(self.q_s[0]/self.d_i)]])
        self.t = self.d_i * (self.L_0_i_s+self.q_s[1]) * sympy.Array([sympy.sin(self.q_s[0]/self.d_i) / self.q_s[0],
                                                                     (1-sympy.cos(self.q_s[0]/self.d_i)) / self.q_s[0]])
        # Add radial translation
        radial_offset = sympy.Array(self.R @ sympy.Matrix([0, self.r]))
        self.t += radial_offset.reshape(radial_offset.shape[0])

        # Compute positional Jacobian
        self.J = sympy.Matrix(self.t).jacobian(self.q)

    def perpendicular_force_at_tip(self):
        """
        Computes and plots perpendicular force at the tip of the segment.
        :return:
        """
        # define normal vector at the tip of the segment
        n_i = self.R @ sympy.Matrix([0, 1])

        tau_i = (self.J.T @ sympy.Matrix(n_i) * self.f_i).subs([(self.s, 1), (self.r, 0)])

        line1, line2, line3 = plot(tau_i[0].subs(self.delta_L_i, 1.0*self.L_0_i), 
                                   tau_i[0].subs(self.delta_L_i, 1.1*self.L_0_i), 
                                   tau_i[0].subs(self.delta_L_i, 1.2*self.L_0_i), 
                                   (self.Delta_i, -45/180*pi*self.d_i, 45/180*pi*self.d_i), 
                                   title="Perpendicular force at the tip of the segment", 
                                   xlabel=r'$\Delta_i$', ylabel=r"$\tau_0$", show=False)
        x1, y1 = line1.get_points()
        x2, y2 = line2.get_points()
        x3, y3 = line3.get_points()
        plt.plot(x1, y1, x2, y2, x3, y3)
        plt.title("Perpendicular force at the tip of the segment")
        plt.xlabel(r'$\Delta_i$')
        plt.ylabel(r"$\tau_0$")
        plt.legend([r"$\delta L_i=0 \%$", r"$\delta L_i=10 \%$", r"$\delta L_i=20 \%$"])
        plt.show()

        line1 = plot(tau_i[1], (self.Delta_i, -45/180*pi*self.d_i, 45/180*pi*self.d_i), 
                     title="Perpendicular force at the tip of the segment", 
                     xlabel=r'$\Delta_i$', ylabel=r"$\tau_1$", show=True)
    
        tau_i = (self.J.T @ sympy.Matrix(n_i) * self.f_i).subs(self.s, 1)

        line1, line2, line3 = plot(tau_i[0].subs(self.delta_L_i, 1.0*self.L_0_i), 
                                   tau_i[0].subs(self.delta_L_i, 1.1*self.L_0_i), 
                                   tau_i[0].subs(self.delta_L_i, 1.2*self.L_0_i), 
                                   (self.Delta_i, -45/180*pi*self.d_i, 45/180*pi*self.d_i), title="Perpendicular force at the tip of the segment", 
                                   xlabel=r'$\Delta_i$', ylabel=r"$\tau_0$", show=False)
        x1, y1 = line1.get_points()
        x2, y2 = line2.get_points()
        x3, y3 = line3.get_points()
        plt.plot(x1, y1, x2, y2, x3, y3)
        plt.title("Perpendicular force at the tip of the segment")
        plt.xlabel(r'$\Delta_i$')
        plt.ylabel(r"$\tau_0$")
        plt.legend([r"$\delta L_i=0 \%$", r"$\delta L_i=10 \%$", r"$\delta L_i=20 \%$"])
        plt.show()

        line1 = plot(tau_i[1], (self.Delta_i, -45/180*pi*self.d_i, 45/180*pi*self.d_i), title="Perpendicular force at the tip of the segment", 
                    xlabel=r'$\Delta_i$', ylabel=r"$\tau_1$", show=True)

if __name__ == '__main__':
    model = JacobianBasedPneumaticActuationModel()
    model.perpendicular_force_at_tip()
