"""Physical parameters of the simulated quadrotor + 3-joint arm (shared by the sympy model,
the Drake cross-check and the controllers).  SI units.

Quadrotor numbers follow the platform of the TFL-CBF-QP paper (m = 1.923 kg,
J = diag(1.15, 1.15, 2.18)e-2 kg m^2) and the arm numbers are of the order of an
OpenMANIPULATOR-X (about 0.7 kg, links 0.13 m and 0.15 m).
"""
import numpy as np


def rod_inertia(m, L, r=0.015):
    """slender rod of length L along the local z axis with radius r"""
    Jt = m * (3 * r ** 2 + L ** 2) / 12.0
    Jz = 0.5 * m * r ** 2
    return Jt, Jt, Jz


def default_params():
    p = dict(mq=1.923, Jqx=1.15e-2, Jqy=1.15e-2, Jqz=2.18e-2, dm=0.06, g=9.81)
    # link 1: short turret below the body, rotates about body z
    p['l1'], p['m1'] = 0.05, 0.12
    p['J1x'], p['J1y'], p['J1z'] = rod_inertia(p['m1'], p['l1'], r=0.03)
    # link 2 and 3: pitch links
    p['l2'], p['m2'] = 0.13, 0.20
    p['J2x'], p['J2y'], p['J2z'] = rod_inertia(p['m2'], p['l2'])
    p['l3'], p['m3'] = 0.15, 0.25   # includes the end-effector mass
    p['J3x'], p['J3y'], p['J3z'] = rod_inertia(p['m3'], p['l3'])
    return p


# joint limits used by the barrier functions (rad)
JOINT_LIMITS = dict(th1=(-np.pi / 2, np.pi / 2), th2=(-0.2, 1.9), th3=(-2.4, -0.15))
