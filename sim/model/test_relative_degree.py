"""
Numerical checks behind Proposition 1 (vector relative degree and the decoupling matrix):
  C1  Rdot = R hat(W dPhi)                      (finite differences in Phi)
  C2  d/dt(W) dPhi = Wdot_dPhi                   (finite differences)
  C3  d/dt(J_c) dq = bias_c, d/dt(J_e) dq = bias_e
  C4  PathOutput chain rule vs finite differences of s(p(t)) for a polynomial p(t)
  C5  closed form of det D:
        det D = det(Pi_c) det(Pi_e) f^2 cos(phi) det(J_rho) / (m^3 cos(theta) det(M_rot))   (up to a fixed sign)
      where Pi = Jacobian of (s1, s2, w) w.r.t. p, evaluated at p_c and p_e.
  C6  body-origin output: D_b (rows p_b'' instead of p_c'''' ) is singular (u_d column zero) and the
      candidate relative degree fails.
  C7  the lower derivatives of the virtual output do not depend on u: p_c''' and p_e' are input-free
      (checked by evaluating with random u and comparing).
"""
import os, sys, numpy as np
here = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, here)
import am_model_gen as gm, am_extra_gen as ge
from params import default_params
from am_controller import AerialManipulator, DualPathTFL, PathOutput, line_path, circle_path

rng = np.random.default_rng(1)
p = default_params(); model = AerialManipulator(p); m = model.m
res = []
def check(name, err, tol):
    ok = err < tol; res.append(ok); print(f"{'PASS' if ok else 'FAIL'}  {name:62s} {err:.2e} (tol {tol:.0e})")

def rand_x():
    q = np.concatenate([rng.normal(0, 1, 3), rng.uniform(-0.5, 0.5, 3), [rng.uniform(-1, 1), rng.uniform(0.2, 1.2), rng.uniform(-1.6, -0.4)]])
    dq = np.concatenate([rng.normal(0, 1, 3), rng.normal(0, 1, 3), rng.normal(0, 1, 3)])
    return np.concatenate([q, dq, [rng.uniform(10, 30), rng.normal(0, 5)]])

# C1, C2, C3
e1 = e2 = e3_ = 0
eps = 1e-6
for _ in range(10):
    x = rand_x(); q, dq = x[:9], x[9:18]
    R = gm.rotation(q); W, Wd = gm.body_rate_map(q, dq)
    qp, qm = q + eps * np.r_[np.zeros(9)], q.copy()
    Rdot_fd = (gm.rotation(q + eps * np.r_[0, 0, 0, dq[3:6], 0, 0, 0]) - gm.rotation(q - eps * np.r_[0, 0, 0, dq[3:6], 0, 0, 0])) / (2 * eps)
    om = W @ dq[3:6]
    hat = lambda v: np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    e1 = max(e1, np.linalg.norm(Rdot_fd - R @ hat(om)))
    dPhi = dq[3:6]
    Wp, _ = gm.body_rate_map(q + eps * np.r_[0, 0, 0, dPhi, 0, 0, 0], dq); Wm, _ = gm.body_rate_map(q - eps * np.r_[0, 0, 0, dPhi, 0, 0, 0], dq)
    e2 = max(e2, np.linalg.norm((Wp - Wm) / (2 * eps) @ dPhi - Wd))
    _, Jc_p, _ = gm.com_kin(q + eps * dq, dq, p); _, Jc_m, _ = gm.com_kin(q - eps * dq, dq, p); _, _, bc = gm.com_kin(q, dq, p)
    _, Je_p, _ = gm.ee_kin(q + eps * dq, dq, p); _, Je_m, _ = gm.ee_kin(q - eps * dq, dq, p); _, _, be = gm.ee_kin(q, dq, p)
    e3_ = max(e3_, np.linalg.norm((Jc_p - Jc_m) / (2 * eps) @ dq - bc), np.linalg.norm((Je_p - Je_m) / (2 * eps) @ dq - be))
check("C1 Rdot = R hat(omega)", e1, 1e-7); check("C2 Wdot dPhi identity", e2, 1e-7); check("C3 Jacobian bias identities", e3_, 1e-6)

# C4 path output chain rule vs. polynomial p(t)
po = PathOutput(circle_path(0.3, -0.2, 1.1, "1.0 + 0.25*sin(2*px)"), 4)
c = rng.normal(0, 0.5, (5, 3)); c[0] = [1.2, 0.4, 1.1]
pt = lambda t: sum(c[k] * t ** k for k in range(5))
ders = [c[0], c[1], 2 * c[2], 6 * c[3]]
outs = po(ders)
import sympy as sp
t = sp.Symbol('t'); px, py, pz = [sum(sp.Float(c[k][i]) * t ** k for k in range(5)) for i in range(3)]
exprs = [(px - 0.3) ** 2 + (py + 0.2) ** 2 - 1.1 ** 2, pz - (1.0 + 0.25 * sp.sin(2 * px)), sp.atan2(py + 0.2, px - 0.3)]
err = 0
for i, e in enumerate(exprs):
    chain, drift, grad = outs[i]
    for k in range(4):
        err = max(err, abs(float(e.diff(t, k).subs(t, 0)) - chain[k]))
    d4 = float(e.diff(t, 4).subs(t, 0))
    err = max(err, abs(d4 - (drift + grad @ (24 * c[4]))))
check("C4 path-output chain rule (orders 0..4) vs sympy", err, 1e-8)

# C5 closed form of det D, C6 body-origin failure, C7 input independence of lower derivatives
ctrl = DualPathTFL(model, line_path(0.0, 1.5), circle_path(0.0, 0.0, 0.8, "1.2 + 0.1*px"), 0.3, 0.3, 0.0, mode='nominal')
ratios = []; detb = []; e7 = 0
for _ in range(25):
    x = rand_x()
    D, b0, v, xi, ch = ctrl.tfl_data(x)
    q = x[:9]; f = x[18]
    Pi_c = np.array([g for (_, _, g) in ctrl.pc([ch['p_c'], ch['v_c'], ch['a_c'], ch['j_c']])])
    Pi_e = np.array([g for (_, _, g) in ctrl.pe([ch['p_e'], ch['v_e']])])
    rho, J_rho, dJ, _, _ = ge.arm_singularity(q[6:9], p)
    M_rot = gm.rot_mass_matrix(q, p)
    closed = np.linalg.det(Pi_c) * np.linalg.det(Pi_e) * f ** 2 * np.cos(q[3]) * dJ / (m ** 3 * np.cos(q[4]) * np.linalg.det(M_rot))
    ratios.append(np.linalg.det(D) / closed)
    # C6: replace the CoM rows by body-origin acceleration rows p_b'' = a0[0:3] + Bu[0:3] u
    Db = D.copy(); Db[0:3, :] = ch['Bu'][0:3, :]
    detb.append(abs(np.linalg.det(Db)) / abs(np.linalg.det(D)))
    # C7: lower derivatives are input free: recompute chains with the model dynamics at two random inputs
    u1, u2 = rng.normal(0, 5, 7), rng.normal(0, 5, 7)
    xd1, xd2 = model.dynamics(x, u1), model.dynamics(x, u2)
    # p_c''' depends on (q,dq,f,fd) only; check via the direct finite-difference of a_c along xdot1 vs xdot2
    h = 1e-5
    da1 = (model.chains(x + h * xd1)['a_c'] - model.chains(x - h * xd1)['a_c']) / (2 * h)
    da2 = (model.chains(x + h * xd2)['a_c'] - model.chains(x - h * xd2)['a_c']) / (2 * h)
    e7 = max(e7, np.linalg.norm(da1 - da2) / (1 + np.linalg.norm(da1)), np.linalg.norm(da1 - ch['j_c']) / (1 + np.linalg.norm(da1)))
    dpe1 = (model.chains(x + h * xd1)['p_e'] - model.chains(x - h * xd1)['p_e']) / (2 * h)
    dpe2 = (model.chains(x + h * xd2)['p_e'] - model.chains(x - h * xd2)['p_e']) / (2 * h)
    e7 = max(e7, np.linalg.norm(dpe1 - dpe2) / (1 + np.linalg.norm(dpe1)), np.linalg.norm(dpe1 - ch['v_e']) / (1 + np.linalg.norm(dpe1)))
ratios = np.array(ratios)
print("   det D / closed form, mean %.6f std %.2e" % (ratios.mean(), ratios.std()))
check("C5 det D closed form (constant ratio, |ratio| = 1)", max(abs(abs(ratios) - 1).max(), ratios.std()), 1e-6)
check("C6 body-origin rows make D singular (|det| ratio)", max(detb), 1e-8)
check("C7 lower-order derivatives are input independent (rel.)", e7, 1e-6)
# Relative degree of yaw and the snap rows: u_d appears only in the CoM rows
check("C7b u_d column zero in EE and yaw rows", np.abs(D[3:, 0]).max(), 1e-12)
print("\nALL PASS" if all(res) else "\nSOME FAILED")
