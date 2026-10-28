"""Dual path following for a quadrotor with a 3-DOF arm: nominal TFL, the relaxed TFL-QP and the
safety-filtered cascade.

Extended input u = (u_d, tau_x, tau_y, tau_z, tau_1, tau_2, tau_3) with u_d the second derivative of
the thrust. Virtual output (alpha_c1, alpha_c2, beta_c, alpha_e1, alpha_e2, beta_e, psi) with vector
relative degree (4, 4, 4, 2, 2, 2, 2). The controller uses a model object with a method chains(x) that
returns the drift and input terms of the output chains (see AerialManipulator.chains).
"""
import numpy as np
import sympy as sp

E3 = np.array([0.0, 0.0, 1.0])


def hat(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], dtype=float)


def euler_zyx_from_R(R):
    """R = Rz(psi) Ry(theta) Rx(phi)"""
    theta = -np.arcsin(np.clip(R[2, 0], -1, 1))
    phi = np.arctan2(R[2, 1], R[2, 2]); psi = np.arctan2(R[1, 0], R[0, 0])
    return np.array([phi, theta, psi])


def body_rate_map(Phi, dPhi):
    """omega = W(Phi) dPhi for ZYX Euler angles, and (d/dt W) dPhi"""
    phi, th, _ = Phi; dphi, dth, _ = dPhi
    W = np.array([[1, 0, -np.sin(th)], [0, np.cos(phi), np.sin(phi) * np.cos(th)], [0, -np.sin(phi), np.cos(phi) * np.cos(th)]])
    Wd = np.array([[0, 0, -np.cos(th) * dth],
                   [0, -np.sin(phi) * dphi, np.cos(phi) * np.cos(th) * dphi - np.sin(phi) * np.sin(th) * dth],
                   [0, -np.cos(phi) * dphi, -np.sin(phi) * np.cos(th) * dphi - np.cos(phi) * np.sin(th) * dth]])
    return W, Wd @ dPhi


class PathOutput:
    """Lie-derivative chains of three scalar functions of a point p(t): two transverse coordinates
    s1, s2 that vanish on the path and the path coordinate w. The expressions are sympy strings in
    px, py, pz. order is 4 for the center of mass and 2 for the end effector."""
    def __init__(self, exprs, order):
        px, py, pz = sp.symbols('px py pz', real=True)
        self.order = order
        pj = [sp.Matrix(sp.symbols(f'p{k}_0 p{k}_1 p{k}_2', real=True)) for k in range(order + 1)]
        pj[0] = sp.Matrix([px, py, pz])
        def Dt(e):
            return sum((sum(e.diff(pj[k][i]) * pj[k + 1][i] for i in range(3)) for k in range(order)), sp.Integer(0))
        outs = []
        for e in exprs:
            e = sp.sympify(e).subs({sp.Symbol('px'): px, sp.Symbol('py'): py, sp.Symbol('pz'): pz})
            chain = [e]
            for k in range(order):
                chain.append(Dt(chain[-1]))
            top = chain[-1]
            drift = top.subs({pj[order][i]: 0 for i in range(3)})
            grad = sp.Matrix([e.diff(pj[0][i]) for i in range(3)])
            outs.append((chain[:-1], drift, grad))
        args = [list(pj[k]) for k in range(order)]
        flat = []
        for chain, drift, grad in outs:
            flat += chain + [drift] + list(grad)
        self.n_per = order + 1 + 3
        self.f = sp.lambdify(sum(args, []), flat, 'math')
        self.exprs = exprs

    def __call__(self, pders):
        """pders: p, p', ..., p^(order-1). Returns per output (chain[0..order-1], drift, grad)."""
        vals = self.f(*np.concatenate(pders))
        res = []
        for i in range(3):
            v = vals[i * self.n_per:(i + 1) * self.n_per]
            res.append((np.array(v[:self.order]), v[self.order], np.array(v[self.order + 1:])))
        return res


def line_path(y0, z0):
    """straight line along x at (y0, z0)"""
    return [f"py - ({y0})", f"pz - ({z0})", "px"]


def circle_path(xc, yc, r, zexpr="1.0"):
    """horizontal circle with altitude profile zexpr(px, py); the path coordinate is the polar angle"""
    return [f"(px-({xc}))**2 + (py-({yc}))**2 - ({r})**2", f"pz - ({zexpr})", f"atan2(py-({yc}), px-({xc}))"]


def wavy_circle_path(xc, yc, r, z0, amp, k, phase=0.0, amp2=0.0, k2=0):
    """horizontal circle with altitude z0 + amp sin(k a + phase) + amp2 sin(k2 a), a the polar angle"""
    a = f"atan2(py-({yc}), px-({xc}))"
    zexpr = f"({z0}) + ({amp})*sin(({k})*{a} + ({phase})) + ({amp2})*sin(({k2})*{a})"
    return [f"(px-({xc}))**2 + (py-({yc}))**2 - ({r})**2", f"pz - ({zexpr})", a]


class AerialManipulator:
    """Symbolic rod model (am_model_gen.py). Extended state x = (q, dq, f, fdot) with q = (p_b, Phi, theta)."""
    def __init__(self, p):
        import am_model_gen as gm, am_extra_gen as ge
        self.gm, self.ge = gm, ge
        self.p = p
        self.m = gm.total_mass(p)[0]
        self.g = p['g']

    def split(self, x):
        return x[:9], x[9:18], x[18], x[19]

    def dynamics(self, x, u):
        gm = self.gm
        q, dq, f, fd = self.split(x)
        M = gm.mass_matrix(q, self.p); h = gm.coriolis(q, dq, self.p); G = gm.gravity(q, self.p)
        b_f, B_tau, B_j = gm.force_dirs(q)
        Q = b_f * f + B_tau @ u[1:4] + B_j @ u[4:7]
        ddq = np.linalg.solve(M, Q - h - G)
        return np.concatenate([dq, ddq, [fd, u[0]]])

    def chains(self, x):
        """Affine-in-u data of the output chains:
           p_c, v_c, a_c, j_c, s0, S    center of mass and derivatives, snap = s0 + S u
           p_e, v_e, e0, Eu             end effector, acceleration = e0 + Eu u
           Phi, dPhi, Phidd0, PhiB      Euler angles, acceleration = Phidd0 + PhiB u
           th, dth, thdd0, thB          joint angles, acceleration = thdd0 + thB u
           f, fd, R, omega, detJ, gradJ, hessJ (det J_rho and its derivatives in theta)"""
        gm, ge = self.gm, self.ge
        q, dq, f, fd = self.split(x)
        p = self.p; m = self.m; g = self.g
        M = gm.mass_matrix(q, p); h = gm.coriolis(q, dq, p); G = gm.gravity(q, p)
        b_f, B_tau, B_j = gm.force_dirs(q)
        Minv = np.linalg.inv(M)
        a0 = Minv @ (b_f * f - h - G)
        Bu = np.hstack([np.zeros((9, 1)), Minv @ np.hstack([B_tau, B_j])])
        R = gm.rotation(q)
        W, Wdot_dPhi = gm.body_rate_map(q, dq)
        omega = W @ dq[3:6]
        wdot0 = W @ a0[3:6] + Wdot_dPhi
        Kw = W @ Bu[3:6, :]
        p_c, J_c, bias_c = gm.com_kin(q, dq, p)
        v_c = J_c @ dq
        Re3 = R @ E3
        a_c = f * Re3 / m - g * E3
        j_c = (fd * Re3 + f * R @ np.cross(omega, E3)) / m
        s0 = (2 * fd * R @ np.cross(omega, E3) + f * R @ np.cross(omega, np.cross(omega, E3)) + f * R @ np.cross(wdot0, E3)) / m
        S = np.zeros((3, 7)); S[:, 0] = Re3 / m
        S += (f / m) * (R @ (-hat(E3))) @ Kw
        p_e, J_e, bias_e = gm.ee_kin(q, dq, p)
        v_e = J_e @ dq; e0 = J_e @ a0 + bias_e; Eu = J_e @ Bu
        rho, J_rho, d, grad, hess = ge.arm_singularity(q[6:9], p)
        return dict(q=q, dq=dq, f=f, fd=fd, R=R, omega=omega,
                    p_c=p_c, v_c=v_c, a_c=a_c, j_c=j_c, s0=s0, S=S,
                    p_e=p_e, v_e=v_e, e0=e0, Eu=Eu,
                    Phi=q[3:6], dPhi=dq[3:6], Phidd0=a0[3:6], PhiB=Bu[3:6, :],
                    th=q[6:9], dth=dq[6:9], thdd0=a0[6:9], thB=Bu[6:9, :],
                    detJ=d, gradJ=grad, hessJ=hess, a0=a0, Bu=Bu)


class DualPathTFL:
    """mode 'nominal': u = D^{-1} v. mode 'relaxed': the QP with the end-effector speed row relaxed,
    solved in closed form. mode 'filter': the nominal input passed through a safety filter with the
    same rows (identity weight)."""
    def __init__(self, model, path_c, path_e, vdes_c, vdes_e, psi_des,
                 gains=None, joint_limits=None, eps_att=0.1, f_min=2.0, sing_eps=1e-4,
                 cbf_poles=None, mode='relaxed',
                 use_barriers=('joint', 'att', 'thrust', 'sing', 'limits'), input_limits=None):
        self.model = model
        self.pc = PathOutput(path_c, 4)
        self.pe = PathOutput(path_e, 2)
        self.vdes_c, self.vdes_e = vdes_c, vdes_e
        if isinstance(psi_des, str):
            self.psi_ref = PathOutput([psi_des, "0", "0"], 4)
            self.psi_des = None
        else:
            self.psi_ref = None; self.psi_des = float(psi_des)
        g = gains or {}
        lam4 = g.get('lam4', 3.0)
        self.k4 = np.array([lam4 ** 4, 4 * lam4 ** 3, 6 * lam4 ** 2, 4 * lam4])
        lam_eta = g.get('lam_eta', 2.0)
        self.k_eta = np.array([lam_eta ** 3, 3 * lam_eta ** 2, 3 * lam_eta])
        lam2 = g.get('lam2', 6.0)
        self.k2 = np.array([lam2 ** 2, 2 * lam2])
        self.k_be = g.get('k_be', 4.0)
        lam_psi = g.get('lam_psi', 4.0)
        self.k_psi = np.array([lam_psi ** 2, 2 * lam_psi])
        self.joint_limits = joint_limits
        self.eps_att, self.f_min, self.sing_eps = eps_att, f_min, sing_eps
        self.mode = mode
        self.use_barriers = use_barriers
        self.E = np.zeros((7, 1)); self.E[5, 0] = 1.0
        self.input_limits = input_limits or dict(f_max=40.0, tau_max=(3.0, 3.0, 1.0), tauj_max=(3.0, 3.0, 3.0))
        cp = cbf_poles or {}
        self.pole_joint = cp.get('joint', 10.0); self.pole_att = cp.get('att', 10.0)
        self.pole_f = cp.get('thrust', 10.0); self.pole_sing = cp.get('sing', 10.0)
        self.last = {}

    def tfl_data(self, x):
        """D, b0, v with D u = v - b0 the linearizing equations, the chain values and the model data"""
        c = self.model.chains(x)
        D = np.zeros((7, 7)); b0 = np.zeros(7); v = np.zeros(7)
        xi = {}
        outs = self.pc([c['p_c'], c['v_c'], c['a_c'], c['j_c']])
        for i, (chain, drift, grad) in enumerate(outs):
            D[i, :] = grad @ c['S']
            b0[i] = drift + grad @ c['s0']
            if i < 2:
                v[i] = -self.k4 @ chain; xi[f'alpha_c{i+1}'] = chain
            else:
                v[i] = -self.k_eta @ np.array([chain[1] - self.vdes_c, chain[2], chain[3]]); xi['beta_c'] = chain
        outs = self.pe([c['p_e'], c['v_e']])
        for i, (chain, drift, grad) in enumerate(outs):
            D[3 + i, :] = grad @ c['Eu']
            b0[3 + i] = drift + grad @ c['e0']
            if i < 2:
                v[3 + i] = -self.k2 @ chain; xi[f'alpha_e{i+1}'] = chain
            else:
                v[3 + i] = -self.k_be * (chain[1] - self.vdes_e); xi['beta_e'] = chain
        if self.psi_ref is None:
            pd, pdd, pddd = self.psi_des, 0.0, 0.0
        else:
            ch_psi = self.psi_ref([c['p_c'], c['v_c'], c['a_c'], c['j_c']])[0][0]
            pd, pdd, pddd = ch_psi[0], ch_psi[1], ch_psi[2]
        e_psi = (c['Phi'][2] - pd + np.pi) % (2 * np.pi) - np.pi
        D[6, :] = c['PhiB'][2, :]; b0[6] = c['Phidd0'][2] - pddd
        v[6] = -self.k_psi @ np.array([e_psi, c['dPhi'][2] - pdd])
        xi['psi'] = np.array([e_psi, c['dPhi'][2] - pdd])
        return D, b0, v, xi, c

    def cbf_rows(self, c):
        """ECBF rows and actuator box rows, A u >= b"""
        rows = []; names = []
        def ecbf2(h, hd, hdd0, hdd_u, pole, name):
            k1, k2 = pole ** 2, 2 * pole
            rows.append((hdd_u, -(hdd0 + k1 * h + k2 * hd))); names.append(name)
        th, dth, thdd0, thB = c['th'], c['dth'], c['thdd0'], c['thB']
        if 'joint' in self.use_barriers and self.joint_limits is not None:
            for j, (lo, hi) in enumerate(self.joint_limits):
                ecbf2(th[j] - lo, dth[j], thdd0[j], thB[j, :], self.pole_joint, f'th{j+1}-')
                ecbf2(hi - th[j], -dth[j], -thdd0[j], -thB[j, :], self.pole_joint, f'th{j+1}+')
        if 'att' in self.use_barriers:
            a_eps = np.pi / 2 - self.eps_att
            Phi, dPhi, Phidd0, PhiB = c['Phi'], c['dPhi'], c['Phidd0'], c['PhiB']
            for j, nm in ((0, 'phi'), (1, 'theta')):
                ecbf2(a_eps - Phi[j], -dPhi[j], -Phidd0[j], -PhiB[j, :], self.pole_att, nm + '+')
                ecbf2(a_eps + Phi[j], dPhi[j], Phidd0[j], PhiB[j, :], self.pole_att, nm + '-')
        if 'thrust' in self.use_barriers:
            e0 = np.zeros(7); e0[0] = 1.0
            ecbf2(c['f'] - self.f_min, c['fd'], 0.0, e0, self.pole_f, 'thrust')
        if 'limits' in self.use_barriers and self.input_limits is not None:
            lim = self.input_limits
            e0 = np.zeros(7); e0[0] = 1.0
            ecbf2(lim['f_max'] - c['f'], -c['fd'], 0.0, -e0, self.pole_f, 'thrust+')
            for i, tm in enumerate(lim['tau_max']):
                ei = np.zeros(7); ei[1 + i] = 1.0
                rows.append((ei, -tm)); names.append(f'tau{i+1}-'); rows.append((-ei, -tm)); names.append(f'tau{i+1}+')
            for i, tm in enumerate(lim['tauj_max']):
                ei = np.zeros(7); ei[4 + i] = 1.0
                rows.append((ei, -tm)); names.append(f'tauj{i+1}-'); rows.append((-ei, -tm)); names.append(f'tauj{i+1}+')
        if 'sing' in self.use_barriers:
            grad, hess = c['gradJ'], c['hessJ']
            ecbf2(c['detJ'] - self.sing_eps, grad @ dth, grad @ thdd0 + dth @ hess @ dth, grad @ thB, self.pole_sing, 'detJ')
        A = np.array([r[0] for r in rows]); b = np.array([r[1] for r in rows])
        return A, b, names

    @staticmethod
    def solve_qp_1d(Q, qv, a, b, tol=1e-12):
        """min 1/2 Q d^2 + q d subject to a_i d >= b_i for scalar d. Rows with a_i > 0 bound d from
        below, rows with a_i < 0 from above, and rows with a_i = 0 hold iff b_i <= 0.
        Returns (d, feasible, active_rows, lo, hi, lo_row, hi_row, zero_margin)."""
        lo, hi, ilo, ihi, zm = -np.inf, np.inf, -1, -1, np.inf
        for i, (ai, bi) in enumerate(zip(a, b)):
            if abs(ai) <= tol:
                zm = min(zm, -bi)
            elif ai > 0:
                if bi / ai > lo: lo, ilo = bi / ai, i
            else:
                if bi / ai < hi: hi, ihi = bi / ai, i
        feas = (lo <= hi + 1e-12) and (zm >= -1e-9)
        if not feas: return None, False, (), lo, hi, ilo, ihi, zm
        d = min(max(-qv / Q, lo), hi)
        act = ()
        if ilo >= 0 and abs(d - lo) <= 1e-12 * (1 + abs(lo)): act = (ilo,)
        elif ihi >= 0 and abs(d - hi) <= 1e-12 * (1 + abs(hi)): act = (ihi,)
        return d, True, act, lo, hi, ilo, ihi, zm

    def __call__(self, x):
        D, b0, v, xi, c = self.tfl_data(x)
        u_tfl = np.linalg.solve(D, v - b0)
        info = dict(xi=xi, det_D=np.linalg.det(D), feasible=True, delta=np.zeros(1), active=(),
                    detJ=c['detJ'], u_tfl=u_tfl, cbf_margin=np.nan, chains=c)
        if self.mode == 'nominal':
            info['u'] = u_tfl; self.last = info; return u_tfl
        A, b, names = self.cbf_rows(c)
        if self.mode == 'relaxed':
            Rd = np.linalg.solve(D, self.E)
            Ab = A @ Rd; bb = b - A @ u_tfl
            info.update(int_lo=np.nan, int_hi=np.nan, int_width=np.nan, zero_margin=np.nan, cheb_r=np.nan)
            d1, feas, act, lo, hi, ilo, ihi, zm = self.solve_qp_1d(1.0, 0.0, Ab[:, 0], bb)
            info.update(int_lo=lo, int_hi=hi, int_width=hi - lo, zero_margin=zm,
                        bound_rows=(names[ilo] if ilo >= 0 else None, names[ihi] if ihi >= 0 else None))
            if feas:
                delta = np.array([d1])
                u = u_tfl + Rd @ delta
            else:
                # fallback without guarantee: nominal TFL with both speed references set to zero
                info['feasible'] = False; delta = np.zeros(1)
                vb = v.copy()
                vb[2] = -self.k_eta @ np.array([xi['beta_c'][1], xi['beta_c'][2], xi['beta_c'][3]])
                vb[5] = -self.k_be * xi['beta_e'][1]
                u = np.linalg.solve(D, vb - b0)
            info['delta'] = delta; info['active'] = tuple(names[i] for i in act); info['n_active'] = len(act)
        elif self.mode == 'filter':
            u, feas = self._solve_filter(u_tfl, A, b)
            if not feas: info['feasible'] = False; u = u_tfl
        info['cbf_margin'] = float(np.min(A @ u - b))
        info['u'] = u
        self.last = info
        return u

    def _solve_filter(self, u_tfl, A, b):
        import cvxpy as cp
        u = cp.Variable(7)
        prob = cp.Problem(cp.Minimize(0.5 * cp.sum_squares(u - u_tfl)), [A @ u >= b])
        prob.solve(solver=cp.CLARABEL)
        if u.value is None: return u_tfl, False
        return np.array(u.value), True
