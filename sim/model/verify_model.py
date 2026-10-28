"""
Numerical verification of the symbolic model (am_model_gen.py):
  T1  Lagrangian consistency: energy conservation under zero inputs (RK45 short run)
  T2  Centre-of-mass identity  m p_c'' = f R e3 - m g e3  for random states and inputs
  T3  Koenig: KE - 1/2 m |p_c'|^2 is independent of the translational velocity
  T4  Drake MultibodyPlant cross-check: KE, PE, angular acceleration, joint accelerations
      and end-effector acceleration for the same physical state and the same applied forces
  T5  Reduced form: the (omega, dtheta) acceleration response to (tau, tau_j) equals M_rot^{-1}
"""
import os, sys, numpy as np
here = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, here)
import am_model_gen as gm
from params import default_params
from make_urdf import make_urdf

rng = np.random.default_rng(0)
p = default_params()
m = gm.total_mass(p)[0]
g = p['g']
e3 = np.array([0, 0, 1.0])


def forward_dynamics(q, dq, f, tau, tauj, p):
    M = gm.mass_matrix(q, p)
    h = gm.coriolis(q, dq, p)
    G = gm.gravity(q, p)
    b_f, B_tau, B_j = gm.force_dirs(q)
    Q = b_f * f + B_tau @ tau + B_j @ tauj
    ddq = np.linalg.solve(M, Q - h - G)
    return ddq


def random_state():
    q = np.concatenate([rng.normal(0, 1, 3), rng.uniform(-0.6, 0.6, 3), rng.uniform(-1.2, 1.2, 3)])
    dq = np.concatenate([rng.normal(0, 1, 3), rng.normal(0, 1.5, 3), rng.normal(0, 2, 3)])
    return q, dq


report = []
def check(name, err, tol):
    ok = err < tol
    report.append((name, err, tol, ok))
    print(f"{'PASS' if ok else 'FAIL'}  {name:70s} err={err:.3e} tol={tol:.1e}")
    return ok

# ---------------- T2 CoM identity ----------------
errs = []
for _ in range(20):
    q, dq = random_state()
    f, tau, tauj = rng.uniform(5, 30), rng.normal(0, 0.5, 3), rng.normal(0, 0.5, 3)
    ddq = forward_dynamics(q, dq, f, tau, tauj, p)
    p_c, J_c, bias_c = gm.com_kin(q, dq, p)
    acc_c = J_c @ ddq + bias_c
    R = gm.rotation(q)
    errs.append(np.linalg.norm(m * acc_c - (f * R @ e3 - m * g * e3)))
check("T2 CoM identity m*acc_c = f R e3 - m g e3 (random states/inputs)", max(errs), 1e-9)

# ---------------- T3 Koenig ----------------
errs = []
for _ in range(10):
    q, dq = random_state()
    KE1, V1 = (float(v[0]) for v in gm.kinetic_energy(q, dq, p))
    p_c, J_c, _ = gm.com_kin(q, dq, p)
    v1 = J_c @ dq
    rel1 = KE1 - 0.5 * m * v1 @ v1
    dq2 = dq.copy(); dq2[:3] = rng.normal(0, 3, 3)
    KE2 = float(gm.kinetic_energy(q, dq2, p)[0][0])
    v2 = J_c @ dq2
    rel2 = KE2 - 0.5 * m * v2 @ v2
    errs.append(abs(rel1 - rel2))
check("T3 Koenig: relative KE independent of translational velocity", max(errs), 1e-9)

# ---------------- T1 energy conservation ----------------
from scipy.integrate import solve_ivp
q0, dq0 = random_state()
def rhs(t, s):
    q, dq = s[:9], s[9:]
    return np.concatenate([dq, forward_dynamics(q, dq, 0.0, np.zeros(3), np.zeros(3), p)])
sol = solve_ivp(rhs, [0, 1.0], np.concatenate([q0, dq0]), rtol=1e-10, atol=1e-12, dense_output=False)
E = []
for k in range(sol.y.shape[1]):
    KE, V = gm.kinetic_energy(sol.y[:9, k], sol.y[9:, k], p)
    E.append(float(KE[0] + V[0]))
check("T1 energy conservation over 1 s free motion (rel. drift)", (max(E) - min(E)) / abs(E[0]), 1e-7)

# ---------------- T5 reduced rotational mass matrix ----------------
errs = []
for _ in range(10):
    q, dq = random_state()
    M_rot = gm.rot_mass_matrix(q, p)
    W, Wdot_dPhi = gm.body_rate_map(q, dq)
    # response of (omega_dot, ddtheta) to (tau, tau_j): columns of the 6x6 input map
    K = np.zeros((6, 6))
    base = forward_dynamics(q, dq, 10.0, np.zeros(3), np.zeros(3), p)
    for j in range(6):
        tau = np.zeros(3); tauj = np.zeros(3)
        if j < 3: tau[j] = 1.0
        else: tauj[j - 3] = 1.0
        ddq = forward_dynamics(q, dq, 10.0, tau, tauj, p) - base
        K[:3, j] = W @ ddq[3:6]      # omega_dot increment (Wdot term cancels in the difference)
        K[3:, j] = ddq[6:9]
    errs.append(np.linalg.norm(K @ M_rot - np.eye(6)))
    # symmetry and positive definiteness of M_rot
    errs.append(np.linalg.norm(M_rot - M_rot.T))
    assert np.all(np.linalg.eigvalsh(M_rot) > 0)
check("T5 input response of (omega_dot, ddtheta) equals M_rot^{-1}", max(errs), 1e-8)

# ---------------- T4 Drake cross-check ----------------
try:
    from pydrake.all import MultibodyPlant, Parser, RotationMatrix, RollPitchYaw, SpatialVelocity, JacobianWrtVariable
    urdf = make_urdf(p, os.path.join(here, "am_primitive.urdf"))
    plant = MultibodyPlant(time_step=0.0)
    Parser(plant).AddModels(urdf)
    plant.Finalize()
    ctx = plant.CreateDefaultContext()
    base = plant.GetBodyByName("base"); ee = plant.GetFrameByName("ee")
    errs = dict(KE=[], PE=[], wdot=[], ddth=[], acc_e=[], acc_c=[])
    for _ in range(10):
        q, dq = random_state()
        f, tau, tauj = rng.uniform(5, 30), rng.normal(0, 0.5, 3), rng.normal(0, 0.5, 3)
        R = gm.rotation(q)
        W, Wdot_dPhi = gm.body_rate_map(q, dq)
        omega_b = W @ dq[3:6]
        # Drake state: quaternion floating base, v = [omega_W; v_WBo_W; dtheta]
        plant.SetFreeBodyPose(ctx, base, __import__('pydrake.all', fromlist=['RigidTransform']).RigidTransform(RotationMatrix(R), q[:3]))
        plant.SetFreeBodySpatialVelocity(base, SpatialVelocity(R @ omega_b, dq[:3]), ctx)
        for j, nm in enumerate(["joint1", "joint2", "joint3"]):
            plant.GetJointByName(nm).set_angle(ctx, q[6 + j]); plant.GetJointByName(nm).set_angular_rate(ctx, dq[6 + j])
        KEd, PEd = plant.CalcKineticEnergy(ctx), plant.CalcPotentialEnergy(ctx)
        KE, V = gm.kinetic_energy(q, dq, p)
        errs['KE'].append(abs(KEd - float(KE[0]))); errs['PE'].append(abs(PEd - float(V[0])))
        # forward dynamics in Drake with the same forces: generalized force = [R tau; f R e3; tauj]
        Md = plant.CalcMassMatrix(ctx); Cv = plant.CalcBiasTerm(ctx); tg = plant.CalcGravityGeneralizedForces(ctx)
        tau_app = np.concatenate([R @ tau, f * R @ e3, tauj])
        vdot = np.linalg.solve(Md, tau_app + tg - Cv)
        wdot_b_drake = R.T @ vdot[:3]
        ddq = forward_dynamics(q, dq, f, tau, tauj, p)
        wdot_b = W @ ddq[3:6] + Wdot_dPhi
        errs['wdot'].append(np.linalg.norm(wdot_b - wdot_b_drake))
        errs['ddth'].append(np.linalg.norm(ddq[6:] - vdot[6:]))
        Je = plant.CalcJacobianTranslationalVelocity(ctx, JacobianWrtVariable.kV, ee, np.zeros(3), plant.world_frame(), plant.world_frame())
        be = plant.CalcBiasTranslationalAcceleration(ctx, JacobianWrtVariable.kV, ee, np.zeros(3), plant.world_frame(), plant.world_frame())
        acc_e_drake = Je @ vdot + np.asarray(be).ravel()
        p_e, J_e, bias_e = gm.ee_kin(q, dq, p)
        errs['acc_e'].append(np.linalg.norm(J_e @ ddq + bias_e - acc_e_drake))
        pc_d = plant.CalcCenterOfMassPositionInWorld(ctx)
        p_c, J_c, bias_c = gm.com_kin(q, dq, p)
        errs['acc_c'].append(np.linalg.norm(pc_d - p_c))
    for k, v in errs.items():
        check(f"T4 Drake cross-check: {k}", max(v), 1e-8)
except Exception as ex:
    import traceback; traceback.print_exc()
    report.append(("T4 Drake", np.nan, np.nan, False))

print("\nSUMMARY:", "ALL PASS" if all(r[3] for r in report) else "SOME FAILED")
