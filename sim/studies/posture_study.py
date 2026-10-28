"""Postures compatible with both paths, on the dual path-following manifold of the paper's scenario.

On the manifold the posture is a function of the vehicle path coordinate a_c and of the lead
a_e - a_c of the end effector. Two studies:

  A. Feasibility sampling. States with z = 0 at 24 positions, 30 leads and 7 end-effector speeds.
     For each state in the admissible set of every barrier, the reduced program is checked for
     strict feasibility, and the margins of the six vehicle rows are recorded.
  B. Lead window. At 144 positions the admissible lead interval is traced on a 0.0025 rad grid along
     the inverse-kinematics branch of the runs, with the reason for each end. The closed-loop lead of
     the incompatible-speed run after 20 s is compared with the upper end.

Also the singular postures of the arm (elbow angle with det J = 0, and the posture with rho_x = 0).
Writes results/studies/posture_study.json, sampling.npz and lead_window.npz.
"""
import os, sys, json, warnings, numpy as np
warnings.filterwarnings('ignore')
HERE = os.path.dirname(os.path.abspath(__file__)); SIM = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(SIM, 'drake')); sys.path.insert(0, os.path.join(SIM, 'model')); sys.path.insert(0, SIM)
from pydrake.all import JacobianWrtVariable, RigidTransform, RotationMatrix
from drake_model import DrakeAerialManipulator
from am_controller import DualPathTFL
import sim_drake as SD
import metrics as M

OUT = os.path.join(SIM, 'results', 'studies'); os.makedirs(OUT, exist_ok=True)
JL = SD.JOINT_LIMITS; EPS_J = SD.EPS_J; r = SD.R_PATH; zc0, ze0 = SD.Z_C0, SD.Z_E0
VC = SD.V_DES; VE = SD.V_E_SWITCH
NU1 = -10.0                       # the ECBF pole of every barrier

dk = DrakeAerialManipulator()
p, ctx = dk.plant, dk.ctx
m = dk.mass; g = dk.g
ctrl = DualPathTFL(dk, SD.PC_PATH, SD.PE_PATH, VC, VE, SD.PSI_REF, joint_limits=JL, mode='relaxed', gains=SD.GAINS, sing_eps=EPS_J)


def gam_c(b):
    """gamma_c and its first three derivatives in the path coordinate"""
    return (np.array([r * np.cos(b), r * np.sin(b), zc0 + 0.1 * np.sin(2 * b)]),
            np.array([-r * np.sin(b), r * np.cos(b), 0.2 * np.cos(2 * b)]),
            np.array([-r * np.cos(b), -r * np.sin(b), -0.4 * np.sin(2 * b)]),
            np.array([r * np.sin(b), -r * np.cos(b), -0.8 * np.cos(2 * b)]))


def gam_e(b):
    return (np.array([r * np.cos(b), r * np.sin(b), ze0 + 0.06 * np.sin(2 * b + 0.8) + 0.03 * np.sin(8 * b)]),
            np.array([-r * np.sin(b), r * np.cos(b), 0.12 * np.cos(2 * b + 0.8) + 0.24 * np.cos(8 * b)]))


def Rz(a): return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
def Ry(a): return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
def Rx(a): return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])


def arm_J(th, R):
    """rho = R^T (p_e - p_c) and J_rho at the joint angles th, attitude R"""
    for k, j in enumerate(dk.joints): j.set_angle(ctx, th[k])
    rho = R.T @ (p.CalcPointsPositions(ctx, dk.ee, np.zeros(3), dk.W).ravel() - p.CalcCenterOfMassPositionInWorld(ctx))
    J = R.T @ (p.CalcJacobianTranslationalVelocity(ctx, JacobianWrtVariable.kV, dk.ee, np.zeros(3), dk.W, dk.W)[:, dk.jv]
               - p.CalcJacobianCenterOfMassTranslationalVelocity(ctx, JacobianWrtVariable.kV, dk.W, dk.W)[:, dk.jv])
    return rho, J


def vehicle_motion(bc):
    """attitude, thrust, thrust rate and body rates of the nominal vehicle motion at path coordinate bc"""
    pc, dpc, ddpc, dddpc = gam_c(bc); dpc, ddpc, dddpc = dpc * VC, ddpc * VC ** 2, dddpc * VC ** 3
    a = ddpc + g * np.array([0, 0, 1.0]); f = m * np.linalg.norm(a); b3 = a / np.linalg.norm(a)
    psi = bc + np.pi / 2; w = Rz(-psi) @ b3; phi = -np.arcsin(w[1]); th = np.arctan2(w[0], w[2])
    R = Rz(psi) @ Ry(th) @ Rx(phi)
    jr = R.T @ (m * dddpc); fd = jr[2]; om2 = jr[0] / f; om1 = -jr[1] / f
    om3 = (VC * np.cos(th) - np.sin(phi) * om2) / np.cos(phi)
    return pc, dpc, R, f, fd, np.array([om1, om2, om3])


def ik(R, target, th_guess, iters=60):
    """joint angles with rho(th) = target on the branch of th_guess, or None"""
    q = np.array(th_guess, float)
    for _ in range(iters):
        rho, J = arm_J(q, R); res = rho - target
        if np.linalg.norm(res) < 1e-12: break
        try:
            q = q - np.linalg.solve(J, res)
        except np.linalg.LinAlgError:
            return None
    rho, J = arm_J(q, R)
    return q if np.linalg.norm(rho - target) <= 1e-9 else None


def make_state(bc, lead, s2, th_guess):
    """state on the manifold: center of mass on gamma_c at bc with speed VC, end effector on gamma_e at
    bc + lead with path speed s2, posture on the branch of th_guess"""
    pc, dpc, R, f, fd, om = vehicle_motion(bc)
    pe, dpe = gam_e(bc + lead); dpe = dpe * s2
    p.SetFreeBodyPose(ctx, dk.body, RigidTransform(RotationMatrix(R), np.zeros(3))); p.SetVelocities(ctx, np.zeros(9))
    q = ik(R, R.T @ (pe - pc), th_guess)
    if q is None: return None, None
    rho, J = arm_J(q, R)
    dth = np.linalg.solve(J, R.T @ (dpe - dpc) - np.cross(om, rho))
    pc0 = p.CalcCenterOfMassPositionInWorld(ctx)
    p.SetFreeBodyPose(ctx, dk.body, RigidTransform(RotationMatrix(R), pc - pc0))
    v = np.zeros(9); v[0:3] = R @ om; v[dk.jv] = dth; p.SetVelocities(ctx, v)
    vc0 = p.CalcJacobianCenterOfMassTranslationalVelocity(ctx, JacobianWrtVariable.kV, dk.W, dk.W) @ v
    v[3:6] = dpc - vc0; p.SetVelocities(ctx, v)
    return np.concatenate([p.GetPositions(ctx), p.GetVelocities(ctx), [f, fd]]), q


def admissible(c):
    """min over the barriers of min(h, L_F h - nu_1 h): nonnegative iff the state is in every A_h"""
    hs = []
    th, dth = c['th'], c['dth']
    for j, (lo, hi) in enumerate(JL):
        hs += [(th[j] - lo, dth[j]), (hi - th[j], -dth[j])]
    ae = np.pi / 2 - 0.1
    for j in (0, 1): hs += [(ae - c['Phi'][j], -c['dPhi'][j]), (ae + c['Phi'][j], c['dPhi'][j])]
    hs += [(c['f'] - 2.0, c['fd']), (40.0 - c['f'], -c['fd'])]
    hs += [(c['detJ'] - EPS_J, c['gradJ'] @ dth)]
    return min(min(h, hd - NU1 * h) for h, hd in hs)


def reduced_rows(x):
    """coefficients and bounds of the reduced program at x, with the row names"""
    D, b0, v, xi, c = ctrl.tfl_data(x)
    u_tfl = np.linalg.solve(D, v - b0); A, b, names = ctrl.cbf_rows(c)
    Rd = np.linalg.solve(D, ctrl.E); Ab = (A @ Rd)[:, 0]; bb = b - A @ u_tfl
    zres = max(np.abs(xi['alpha_c1']).max(), np.abs(xi['alpha_c2']).max(), abs(xi['beta_c'][1] - VC), np.abs(xi['beta_c'][2:]).max(),
               np.abs(xi['alpha_e1']).max(), np.abs(xi['alpha_e2']).max(), np.abs(xi['psi']).max())
    return Ab, bb, names, c, zres


TH_RUN = np.array([-0.05, -0.75, 0.48])     # posture of the runs at t = 4 s, selects the branch


def study_sampling():
    speeds = [-0.5, 0.0, 0.25, 0.42, 0.6, 1.0, 1.5]
    rows = []
    for bc in np.linspace(0, 2 * np.pi, 25)[:-1]:
        qg = TH_RUN.copy()
        for lead in np.linspace(0.05, 0.34, 30):
            for s2 in speeds:
                x, qn = make_state(bc, lead, s2, qg)
                if x is None: continue
                if s2 == 0.25: qg = qn
                Ab, bb, names, c, zres = reduced_rows(x)
                d, feas, act, lo, hi, ilo, ihi, zm = ctrl.solve_qp_1d(1.0, 0.0, Ab, bb)
                zero = np.abs(Ab) <= 1e-12
                strict = bool((lo < hi) and np.all(bb[zero] < 0))
                veh = {n: -bb[i] for i, n in enumerate(names) if n in ('phi+', 'phi-', 'theta+', 'theta-', 'thrust', 'thrust+')}
                rows.append(dict(bc=bc, lead=lead, s2=s2, detJ=c['detJ'], adm=admissible(c), lo=lo, hi=hi, zm=zm, zres=zres,
                                 feasible=bool(feas), strict=strict, att=min(veh[k] for k in ('phi+', 'phi-', 'theta+', 'theta-')),
                                 thr=min(veh['thrust'], veh['thrust+']), zero_veh=float(np.abs(Ab[[names.index(k) for k in veh]]).max())))
    A = {k: np.array([rw[k] for rw in rows], dtype=float) for k in rows[0]}
    np.savez(os.path.join(OUT, 'sampling.npz'), **A)
    inO = A['adm'] >= 0
    per_speed = {}
    for s2 in speeds:
        k = A['s2'] == s2
        per_speed[str(s2)] = dict(states=int(k.sum()), in_admissible_set=int((k & inO).sum()),
                                  not_strictly_feasible=int((k & inO & (A['strict'] == 0)).sum()),
                                  infeasible=int((k & inO & (A['feasible'] == 0)).sum()))
    return dict(positions=24, leads=30, lead_range=[float(A['lead'].min()), float(A['lead'].max())], speeds=speeds,
                n_states=len(rows), max_z_residual=float(A['zres'].max()), n_in_admissible_set=int(inO.sum()),
                n_not_strictly_feasible=int((inO & (A['strict'] == 0)).sum()),
                max_zero_coefficient_vehicle_rows=float(A['zero_veh'].max()),
                vehicle_row_margin_attitude=float(A['att'].min()), vehicle_row_margin_thrust=float(A['thr'].min()),
                per_speed=per_speed)


REASON = {1: 'joint limit', 2: 'det J below eps_J', 3: 'no posture on the branch'}


def study_window(n_pos=144, step=0.0025, lead_min=-0.6, lead_max=1.0):
    acs = np.linspace(-np.pi, np.pi, n_pos + 1)[:-1]
    leads = np.arange(lead_min, lead_max + step / 2, step)
    k0 = int(np.argmin(np.abs(leads - SD.LEAD0)))
    win = np.zeros((n_pos, len(leads)), int); detJ = np.full((n_pos, len(leads)), np.nan); TH = np.full((n_pos, len(leads), 3), np.nan)
    lo_b = np.full(n_pos, np.nan); hi_b = np.full(n_pos, np.nan); lo_why = np.zeros(n_pos, int); hi_why = np.zeros(n_pos, int)

    def classify(q):
        if q is None: return 3
        if any(not (lo <= qq <= hi) for qq, (lo, hi) in zip(q, JL)): return 1
        if dk._detJ(q) < EPS_J: return 2
        return 0

    for i, bc in enumerate(acs):
        pc, dpc, R, f, fd, om = vehicle_motion(bc)
        p.SetFreeBodyPose(ctx, dk.body, RigidTransform(RotationMatrix(R), np.zeros(3))); p.SetVelocities(ctx, np.zeros(9))
        q0 = ik(R, R.T @ (gam_e(bc + leads[k0])[0] - pc), TH_RUN)
        assert classify(q0) == 0
        for direction in (1, -1):
            q = q0; k = k0
            while True:
                kn = k + direction
                if kn < 0 or kn >= len(leads): break
                qn = ik(R, R.T @ (gam_e(bc + leads[kn])[0] - pc), q)
                why = classify(qn)
                if why:
                    # the branch is continued on a finer grid up to the next coarse point to name the end
                    qf = q
                    for s_ in np.arange(leads[k] + direction * step / 25, leads[kn] + direction * step / 50, direction * step / 25):
                        qn = ik(R, R.T @ (gam_e(bc + s_)[0] - pc), qf)
                        why = classify(qn)
                        if why: break
                        qf = qn
                    (hi_why if direction > 0 else lo_why)[i] = why; break
                k, q = kn, qn
                win[i, k] = 1; detJ[i, k] = dk._detJ(q); TH[i, k] = q
            (hi_b if direction > 0 else lo_b)[i] = leads[k]
        win[i, k0] = 1; detJ[i, k0] = dk._detJ(q0); TH[i, k0] = q0
    np.savez(os.path.join(OUT, 'lead_window.npz'), acs=acs, leads=leads, win=win, detJ=detJ, th=TH, lo_b=lo_b, hi_b=hi_b,
             lo_why=lo_why, hi_why=hi_why, step=step)
    # closed-loop lead of the incompatible-speed run against the upper end
    L = M.load('D4_relaxed'); t = L['t']; late = t > 20.0
    ac = np.arctan2(L['p_c'][late, 1], L['p_c'][late, 0]); ae = np.arctan2(L['p_e'][late, 1], L['p_e'][late, 0])
    lead = (ae - ac + np.pi) % (2 * np.pi) - np.pi
    hi_at = np.interp(ac, np.r_[acs, np.pi], np.r_[hi_b, hi_b[0]])
    width = hi_b - lo_b
    return dict(positions=n_pos, grid_step=step, start_lead=float(leads[k0]),
                width_range=[float(width.min()), float(width.max())], arc_length_cm=[1e2 * r * float(width.min()), 1e2 * r * float(width.max())],
                lower_end_range=[float(lo_b.min()), float(lo_b.max())], upper_end_range=[float(hi_b.min()), float(hi_b.max())],
                lower_end_reason={REASON[k]: int((lo_why == k).sum()) for k in (1, 2, 3) if (lo_why == k).any()},
                upper_end_reason={REASON[k]: int((hi_why == k).sum()) for k in (1, 2, 3) if (hi_why == k).any()},
                detJ_at_upper_end=[float(np.nanmin([detJ[i, np.searchsorted(leads, hi_b[i])] for i in range(n_pos)])),
                                   float(np.nanmax([detJ[i, np.searchsorted(leads, hi_b[i])] for i in range(n_pos)]))],
                run_lead_minus_upper_end_after_20s=[float((lead - hi_at).min()), float((lead - hi_at).max())],
                run_lead_after_20s=[float(lead.min()), float(lead.max())])


def singular_postures():
    """elbow angle with det J = 0 for every shoulder angle, and the shoulder angle with rho_x = 0 at th3 = 0.1"""
    from scipy.optimize import brentq
    f3 = lambda th3, th2: dk._detJ(np.array([0.0, th2, th3]))
    th3_star = [brentq(f3, -1.6, -1.1, args=(th2,)) for th2 in (-1.0, 0.0, 1.0)]
    f2 = lambda th2: dk._detJ(np.array([0.0, th2, 0.1]))
    th2_rho = brentq(f2, -1.5, -1.0)
    return dict(th3_star=float(np.mean(th3_star)), th3_star_spread=float(np.ptp(th3_star)), th2_rho_x_zero_at_th3_0p1=float(th2_rho),
                elbow_limit_to_singularity=float(JL[2][0] - np.mean(th3_star)),
                airframe_mass=float(p.GetBodyByName('x2').default_mass()), total_mass=float(m))


if __name__ == '__main__':
    out = dict(singular=singular_postures())
    print('singular postures', out['singular'])
    out['sampling'] = study_sampling(); print('sampling', json.dumps(out['sampling'], indent=1))
    out['window'] = study_window(); print('window', json.dumps(out['window'], indent=1))
    json.dump(out, open(os.path.join(OUT, 'posture_study.json'), 'w'), indent=1)
    print('wrote', os.path.join(OUT, 'posture_study.json'))
