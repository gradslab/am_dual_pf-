"""Symbolic Euler-Lagrange model of the quadrotor with a 3-DOF arm (sympy), written to am_model_gen.py.

Generalized coordinates q = (x, y, z, phi, theta, psi, th1, th2, th3): body origin, ZYX Euler angles and the
waist, shoulder and elbow angles. The Coriolis vector is built from the body Jacobians,
h = sum_i m_i Jv^T Jvdot qd + Jw^T (J Jwdot qd + w x J w), which keeps the expressions small."""
import sympy as sp
import time, sys, os
t0 = time.time()
def log(*a):
    print(f"[{time.time()-t0:7.1f}s]", *a); sys.stdout.flush()

x, y, z, phi, th, psi, t1, t2, t3 = sp.symbols('x y z phi theta psi th1 th2 th3', real=True)
dx, dy, dz, dphi, dth, dpsi, dt1, dt2, dt3 = sp.symbols('dx dy dz dphi dtheta dpsi dth1 dth2 dth3', real=True)
q = sp.Matrix([x, y, z, phi, th, psi, t1, t2, t3]); dq = sp.Matrix([dx, dy, dz, dphi, dth, dpsi, dt1, dt2, dt3])
Phi = sp.Matrix([phi, th, psi]); dPhi = sp.Matrix([dphi, dth, dpsi]); theta = sp.Matrix([t1, t2, t3])
P = dict(
    mq=sp.Symbol('mq', positive=True), Jqx=sp.Symbol('Jqx', positive=True), Jqy=sp.Symbol('Jqy', positive=True), Jqz=sp.Symbol('Jqz', positive=True),
    dm=sp.Symbol('dm', real=True),
    l1=sp.Symbol('l1', positive=True), m1=sp.Symbol('m1', positive=True), J1x=sp.Symbol('J1x', positive=True), J1y=sp.Symbol('J1y', positive=True), J1z=sp.Symbol('J1z', positive=True),
    l2=sp.Symbol('l2', positive=True), m2=sp.Symbol('m2', positive=True), J2x=sp.Symbol('J2x', positive=True), J2y=sp.Symbol('J2y', positive=True), J2z=sp.Symbol('J2z', positive=True),
    l3=sp.Symbol('l3', positive=True), m3=sp.Symbol('m3', positive=True), J3x=sp.Symbol('J3x', positive=True), J3y=sp.Symbol('J3y', positive=True), J3z=sp.Symbol('J3z', positive=True),
    g=sp.Symbol('g', positive=True))
globals().update(P); param_names = list(P.keys())
def Rx(a): return sp.Matrix([[1, 0, 0], [0, sp.cos(a), -sp.sin(a)], [0, sp.sin(a), sp.cos(a)]])
def Ry(a): return sp.Matrix([[sp.cos(a), 0, sp.sin(a)], [0, 1, 0], [-sp.sin(a), 0, sp.cos(a)]])
def Rz(a): return sp.Matrix([[sp.cos(a), -sp.sin(a), 0], [sp.sin(a), sp.cos(a), 0], [0, 0, 1]])
e3 = sp.Matrix([0, 0, 1])
p_b = sp.Matrix([x, y, z]); R_b = Rz(psi) * Ry(th) * Rx(phi)
r_m = sp.Matrix([0, 0, -dm]); R_1 = R_b * Rz(t1); o1 = p_b + R_b * r_m
c1 = o1 + R_1 * sp.Matrix([0, 0, -l1 / 2]); o2 = o1 + R_1 * sp.Matrix([0, 0, -l1])
R_2 = R_1 * Ry(-t2); c2 = o2 + R_2 * sp.Matrix([0, 0, -l2 / 2]); o3 = o2 + R_2 * sp.Matrix([0, 0, -l2])
R_3 = R_2 * Ry(-t3); c3 = o3 + R_3 * sp.Matrix([0, 0, -l3 / 2]); p_e = o3 + R_3 * sp.Matrix([0, 0, -l3])
bodies = [(mq, p_b, R_b, sp.diag(Jqx, Jqy, Jqz)), (m1, c1, R_1, sp.diag(J1x, J1y, J1z)),
          (m2, c2, R_2, sp.diag(J2x, J2y, J2z)), (m3, c3, R_3, sp.diag(J3x, J3y, J3z))]
m_tot = mq + m1 + m2 + m3
p_c = (mq * p_b + m1 * c1 + m2 * c2 + m3 * c3) / m_tot
def ddt(expr): return sum((expr.diff(q[k]) * dq[k] for k in range(9)), sp.zeros(*expr.shape))
def vee(S): return sp.Matrix([S[2, 1], S[0, 2], S[1, 0]])

log("Jacobians")
M = sp.zeros(9, 9); h = sp.zeros(9, 1); KE = 0
for (mi, ci, Ri, Ji) in bodies:
    Jv = ci.jacobian(q)
    wi = vee(Ri.T * ddt(Ri))
    Jw = wi.jacobian(dq)
    M += mi * Jv.T * Jv + Jw.T * Ji * Jw
    Jv_dot_dq = ddt(Jv) * dq
    Jw_dot_dq = ddt(Jw) * dq
    h += mi * Jv.T * Jv_dot_dq + Jw.T * (Ji * Jw_dot_dq + wi.cross(Ji * wi))
    vi = Jv * dq
    KE += sp.Rational(1, 2) * mi * (vi.T * vi)[0, 0] + sp.Rational(1, 2) * (wi.T * Ji * wi)[0, 0]
V = g * (mq * p_b[2] + m1 * c1[2] + m2 * c2[2] + m3 * c3[2])
G = sp.Matrix([V.diff(qi) for qi in q])
log("kinematics")
omega_b = vee(R_b.T * ddt(R_b)); W = omega_b.jacobian(dPhi)
Wdot_dPhi = sum((W.diff(Phi[k]) * dPhi[k] for k in range(3)), sp.zeros(3, 3)) * dPhi
J_c = p_c.jacobian(q); bias_c = ddt(J_c) * dq
J_e = p_e.jacobian(q); bias_e = ddt(J_e) * dq
# body-frame arm geometry: evaluate the kinematic chain with R_b = I (these depend on theta only)
_sub0 = {phi: 0, th: 0, psi: 0, x: 0, y: 0, z: 0}
rho = (p_e - p_c).subs(_sub0); J_rho = rho.jacobian(theta)
r_c = (p_c - p_b).subs(_sub0); J_rc = r_c.jacobian(theta)
log("M_rot")
v_c = J_c * dq
KE_rel0 = (KE - sp.Rational(1, 2) * m_tot * (v_c.T * v_c)[0, 0]).subs({dx: 0, dy: 0, dz: 0})
M_rel_Phi = sp.hessian(KE_rel0, sp.Matrix([dphi, dth, dpsi, dt1, dt2, dt3]))
T6 = sp.diag(W.inv(), sp.eye(3)); M_rot = T6.T * M_rel_Phi * T6
b_f = sp.Matrix.vstack(R_b * e3, sp.zeros(6, 1)); B_tau = sp.Matrix.vstack(sp.zeros(3, 3), W.T, sp.zeros(3, 3)); B_j = sp.Matrix.vstack(sp.zeros(6, 3), sp.eye(3))

log("codegen")
qnames = ['x', 'y', 'z', 'phi', 'theta', 'psi', 'th1', 'th2', 'th3']; dqnames = ['dx', 'dy', 'dz', 'dphi', 'dtheta', 'dpsi', 'dth1', 'dth2', 'dth3']
def gen_function(fname, exprs, args, lines):
    names = list(exprs.keys()); mats = [exprs[n] for n in names]
    flat = []; shapes = []
    for m_ in mats: shapes.append(m_.shape); flat += list(m_)
    reps, red = sp.cse(flat)
    lines.append(f"def {fname}({', '.join(args)}):")
    if 'q' in args: lines.append("    " + ", ".join(qnames) + " = q")
    if 'dq' in args: lines.append("    " + ", ".join(dqnames) + " = dq")
    if 'p' in args:
        for pn in param_names: lines.append(f"    {pn} = p['{pn}']")
    for s, e in reps: lines.append(f"    {s} = {sp.pycode(e, fully_qualified_modules=False)}")
    k = 0; outs = []
    for n, shp in zip(names, shapes):
        cnt = shp[0] * shp[1]; items = [sp.pycode(red[k + i], fully_qualified_modules=False) for i in range(cnt)]; k += cnt
        if shp[1] == 1: lines.append(f"    {n} = np.array([{', '.join(items)}], dtype=float)")
        else:
            rows = ["[" + ", ".join(items[r * shp[1]:(r + 1) * shp[1]]) + "]" for r in range(shp[0])]
            lines.append(f"    {n} = np.array([{', '.join(rows)}], dtype=float)")
        outs.append(n)
    lines.append("    return " + ", ".join(outs)); lines.append("")
lines = ["# AUTO-GENERATED by derive_model_fast.py. Do not edit.", "import numpy as np", "from math import sin, cos, tan, sqrt, pi", f"PARAM_NAMES = {param_names!r}", ""]
gen_function("mass_matrix", {"M": M}, ["q", "p"], lines); log("  M")
gen_function("coriolis", {"h": h}, ["q", "dq", "p"], lines); log("  h")
gen_function("gravity", {"G": G}, ["q", "p"], lines)
gen_function("body_rate_map", {"W": W, "Wdot_dPhi": Wdot_dPhi}, ["q", "dq"], lines)
gen_function("rotation", {"R": R_b}, ["q"], lines)
gen_function("com_kin", {"p_c": p_c, "J_c": J_c, "bias_c": bias_c}, ["q", "dq", "p"], lines); log("  com")
gen_function("ee_kin", {"p_e": p_e, "J_e": J_e, "bias_e": bias_e}, ["q", "dq", "p"], lines); log("  ee")
gen_function("arm_geometry", {"rho": rho, "J_rho": J_rho, "r_c": r_c, "J_rc": J_rc}, ["q", "p"], lines)
gen_function("det_J_rho", {"dJ": sp.Matrix([J_rho.det()])}, ["q", "p"], lines)
gen_function("rot_mass_matrix", {"M_rot": M_rot}, ["q", "p"], lines); log("  M_rot")
gen_function("force_dirs", {"b_f": b_f, "B_tau": B_tau, "B_j": B_j}, ["q"], lines)
gen_function("link_points", {"o1": o1, "o2": o2, "o3": o3, "c1": c1, "c2": c2, "c3": c3}, ["q", "p"], lines)
gen_function("total_mass", {"m": sp.Matrix([m_tot])}, ["p"], lines)
gen_function("kinetic_energy", {"KE": sp.Matrix([KE]), "V": sp.Matrix([V])}, ["q", "dq", "p"], lines)
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "am_model_gen.py")
open(out, "w").write("\n".join(lines)); log("wrote", out)
