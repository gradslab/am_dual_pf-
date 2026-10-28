"""Drake MultibodyPlant wrapper with the chains() interface of the controller.
State x = (q, v, f, fdot) with q = (quaternion wxyz, position, 3 joint angles) and v = (omega in W, velocity in W, joint rates)."""
import os, sys, numpy as np
from pydrake.all import MultibodyPlant, Parser, JacobianWrtVariable
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', 'model'))
from am_controller import hat, euler_zyx_from_R, body_rate_map, E3
from make_realistic_urdf import make as make_urdf


class DrakeAerialManipulator:
    def __init__(self, urdf=None, plant=None):
        if plant is None:
            self.plant = MultibodyPlant(time_step=0.0)
            Parser(self.plant).AddModels(urdf or make_urdf())
            self.plant.Finalize()
        else:
            self.plant = plant
        p = self.plant
        self.ctx = p.CreateDefaultContext()
        self.ctx2 = p.CreateDefaultContext()
        self.body = p.GetBodyByName('x2'); self.ee = p.GetFrameByName('ee'); self.W = p.world_frame()
        self.joints = [p.GetJointByName(n) for n in ('joint1', 'joint2', 'joint3')]
        self.jv = [j.velocity_start() for j in self.joints]; self.jq = [j.position_start() for j in self.joints]
        self.nv = p.num_velocities(); self.nq = p.num_positions()
        self.mass = p.CalcTotalMass(self.ctx); self.g = 9.81
        self.p = None

    def set_state(self, x, ctx=None):
        ctx = self.ctx if ctx is None else ctx
        self.plant.SetPositions(ctx, x[:self.nq]); self.plant.SetVelocities(ctx, x[self.nq:self.nq + self.nv])

    def get_state(self, f, fd, ctx=None):
        ctx = self.ctx if ctx is None else ctx
        return np.concatenate([self.plant.GetPositions(ctx), self.plant.GetVelocities(ctx), [f, fd]])

    def generalized_force(self, R, f, tau, tau_j):
        """generalized force: spatial force about the body origin in W, then the joint torques"""
        qf = np.zeros(self.nv)
        qf[0:3] = R @ tau; qf[3:6] = f * R @ E3
        for k, i in enumerate(self.jv): qf[i] += tau_j[k]
        return qf

    def _detJ(self, th):
        p, c2 = self.plant, self.ctx2
        p.SetPositions(c2, p.GetPositions(self.ctx)); p.SetVelocities(c2, np.zeros(self.nv))
        for k, j in enumerate(self.joints): j.set_angle(c2, th[k])
        R = p.EvalBodyPoseInWorld(c2, self.body).rotation().matrix()
        Je = p.CalcJacobianTranslationalVelocity(c2, JacobianWrtVariable.kV, self.ee, np.zeros(3), self.W, self.W)
        Jc = p.CalcJacobianCenterOfMassTranslationalVelocity(c2, JacobianWrtVariable.kV, self.W, self.W)
        return np.linalg.det(R.T @ (Je[:, self.jv] - Jc[:, self.jv]))

    def arm_singularity(self, th, h=1e-4):
        d0 = self._detJ(th); grad = np.zeros(3); hess = np.zeros((3, 3)); fp = np.zeros(3); fm = np.zeros(3)
        for i in range(3):
            e = np.zeros(3); e[i] = h
            fp[i] = self._detJ(th + e); fm[i] = self._detJ(th - e)
            grad[i] = (fp[i] - fm[i]) / (2 * h); hess[i, i] = (fp[i] - 2 * d0 + fm[i]) / h ** 2
        for i in range(3):
            for j in range(i + 1, 3):
                ei = np.zeros(3); ei[i] = h; ej = np.zeros(3); ej[j] = h
                hess[i, j] = hess[j, i] = (self._detJ(th + ei + ej) - fp[i] - fp[j] + 2 * d0 - fm[i] - fm[j] + self._detJ(th - ei - ej)) / (2 * h ** 2)
        return d0, grad, hess

    def chains(self, x):
        p, ctx = self.plant, self.ctx
        self.set_state(x)
        f, fd = x[self.nq + self.nv], x[self.nq + self.nv + 1]
        X = p.EvalBodyPoseInWorld(ctx, self.body); R = X.rotation().matrix()
        v = p.GetVelocities(ctx)
        omega = R.T @ v[0:3]
        Phi = euler_zyx_from_R(R)
        W, _ = body_rate_map(Phi, np.zeros(3)); dPhi = np.linalg.solve(W, omega); _, Wdot_dPhi = body_rate_map(Phi, dPhi)
        M = p.CalcMassMatrix(ctx); Cv = p.CalcBiasTerm(ctx); tg = p.CalcGravityGeneralizedForces(ctx)
        Minv = np.linalg.inv(M)
        a0 = Minv @ (tg - Cv + self.generalized_force(R, f, np.zeros(3), np.zeros(3)))
        Bu = np.zeros((self.nv, 7))
        for k in range(6):
            tau = np.zeros(3); tj = np.zeros(3)
            if k < 3: tau[k] = 1.0
            else: tj[k - 3] = 1.0
            Bu[:, 1 + k] = Minv @ self.generalized_force(R, 0.0, tau, tj)
        wdot0 = R.T @ a0[0:3]; Kw = R.T @ Bu[0:3, :]
        Winv = np.linalg.inv(W)
        Phidd0 = Winv @ (wdot0 - Wdot_dPhi); PhiB = Winv @ Kw
        p_c = p.CalcCenterOfMassPositionInWorld(ctx)
        J_c = p.CalcJacobianCenterOfMassTranslationalVelocity(ctx, JacobianWrtVariable.kV, self.W, self.W); v_c = J_c @ v
        Re3 = R @ E3
        a_c = f * Re3 / self.mass - self.g * E3
        j_c = (fd * Re3 + f * R @ np.cross(omega, E3)) / self.mass
        s0 = (2 * fd * R @ np.cross(omega, E3) + f * R @ np.cross(omega, np.cross(omega, E3)) + f * R @ np.cross(wdot0, E3)) / self.mass
        S = np.zeros((3, 7)); S[:, 0] = Re3 / self.mass; S += (f / self.mass) * (R @ (-hat(E3))) @ Kw
        p_e = p.CalcPointsPositions(ctx, self.ee, np.zeros(3), self.W).ravel()
        J_e = p.CalcJacobianTranslationalVelocity(ctx, JacobianWrtVariable.kV, self.ee, np.zeros(3), self.W, self.W)
        bias_e = p.CalcBiasTranslationalAcceleration(ctx, JacobianWrtVariable.kV, self.ee, np.zeros(3), self.W, self.W).ravel()
        v_e = J_e @ v; e0 = J_e @ a0 + bias_e; Eu = J_e @ Bu
        th = np.array([j.get_angle(ctx) for j in self.joints]); dth = v[self.jv]
        thdd0 = a0[self.jv]; thB = Bu[self.jv, :]
        detJ, gradJ, hessJ = self.arm_singularity(th)
        return dict(q=None, dq=None, f=f, fd=fd, R=R, omega=omega, p_c=p_c, v_c=v_c, a_c=a_c, j_c=j_c, s0=s0, S=S,
                    p_e=p_e, v_e=v_e, e0=e0, Eu=Eu, Phi=Phi, dPhi=dPhi, Phidd0=Phidd0, PhiB=PhiB,
                    th=th, dth=dth, thdd0=thdd0, thB=thB, detJ=detJ, gradJ=gradJ, hessJ=hessJ, a0=a0, Bu=Bu, p_b=X.translation().copy())

