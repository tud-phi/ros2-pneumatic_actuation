from math import pi
import matplotlib.pyplot as plt
import sympy
from sympy.geometry import line
from sympy.plotting import plot

"""
This script derives the symbolic model of the pneumatic actuation system for a planar one segment robot.
"""

if __name__ == '__main__':
    # Parameters
    d_i = 1
    L_0_i = 1

    # Symbolic variables
    s, Delta_i, delta_L_i = sympy.symbols('s Delta_i delta_L_i')
    q = sympy.Array([Delta_i, delta_L_i])

    # Scale q with s
    q_s = s * q
    L_0_i_s = s * L_0_i
    
    R = sympy.Matrix([[sympy.cos(q_s[0]/d_i), -sympy.sin(q_s[0]/d_i)],
                      [sympy.sin(q_s[0]/d_i), sympy.cos(q_s[0]/d_i)]])
    t = d_i * (L_0_i_s+q_s[1]) * sympy.Array([sympy.sin(q_s[0]/d_i) / q_s[0],
                                             (1-sympy.cos(q_s[0]/d_i)) / q_s[0]])

    # Compute positional Jacobian
    J = sympy.Matrix(t).jacobian(q)

    print("Jacobian", J)

    ## Perpendicular force at the tip of the segment
    f_i = 1 # Scalar force acting at the tip of the segment [N] perpendicular to center-line

    # define normal vector at the tip of the segment
    n_i = R @ sympy.Matrix([0, 1])

    tau_i = (J.T @ sympy.Matrix(n_i) * f_i).subs(s, 1)

    line1, line2, line3 = plot(tau_i[0].subs(delta_L_i, 1.0*L_0_i), tau_i[0].subs(delta_L_i, 1.1*L_0_i), tau_i[0].subs(delta_L_i, 1.2*L_0_i), 
                               (Delta_i, -45/180*pi*d_i, 45/180*pi*d_i), title="Perpendicular force at the tip of the segment", 
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

    print("tau_i[1]", tau_i[1])
    line1 = plot(tau_i[1], (Delta_i, -45/180*pi*d_i, 45/180*pi*d_i), title="Perpendicular force at the tip of the segment", 
                 xlabel=r'$\Delta_i$', ylabel=r"$\tau_1$", show=True)