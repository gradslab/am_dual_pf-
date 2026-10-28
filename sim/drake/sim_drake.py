"""Closed-loop runs in Drake. The plant is a continuous MultibodyPlant integrated with RK3 at 4 kHz.
The controller runs at a fixed rate (400 Hz by default) with a zero-order hold on the body-frame
command and integrates the thrust chain (u_t, du_t/dt) itself.

Runs: D3 nominal controller, equal speeds (Fig. 2 of the paper)
      D4 end-effector speed reference raised at t_sw, one run per mode (Fig. 3)
      D5 model mismatch: the simulated plant masses and inertias are scaled, the controller keeps the model

    python3 drake/sim_drake.py D3
    python3 drake/sim_drake.py D4 --modes relaxed,filter,nominal
    python3 drake/sim_drake.py D4 --modes relaxed --hz 1000 --tag _1khz
    python3 drake/sim_drake.py D5 --arm-scale 1.02 --tag _a102
"""
import os, sys, time, json, argparse, numpy as np
from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph, Parser, LeafSystem, Simulator,
                         MeshcatVisualizer, StartMeshcat, RotationMatrix, RigidTransform, ResetIntegratorFromFlags,
                         Rgba, Quaternion, Sphere, SpatialInertia, JacobianWrtVariable)
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', 'model')); sys.path.insert(0, here)
from am_controller import DualPathTFL, wavy_circle_path, E3
from drake_model import DrakeAerialManipulator
from make_realistic_urdf import make as make_urdf
RES = os.path.join(here, '..', 'results'); os.makedirs(RES, exist_ok=True)

JOINT_LIMITS = [(-1.5, 1.5), (-1.3, 1.3), (-1.3, 1.2)]
GAINS = dict(lam4=3.0, lam_eta=2.0, lam2=1.5, k_be=1.0, lam_psi=3.0)
EPS_J = 1.5e-3

# the two paths of the paper: wavy circles of radius 1.2 m
R_PATH, Z_C0, Z_E0 = 1.2, 1.6, 1.42
PC_PATH = wavy_circle_path(0, 0, R_PATH, Z_C0, 0.10, 2)
PE_PATH = wavy_circle_path(0, 0, R_PATH, Z_E0, 0.06, 2, phase=0.8, amp2=0.03, k2=8)
PSI_REF = "atan2(py, px) + pi/2"
def zc_of(a): return Z_C0 + 0.10 * np.sin(2 * a)
def ze_of(a): return Z_E0 + 0.06 * np.sin(2 * a + 0.8) + 0.03 * np.sin(8 * a)
LEAD0 = 0.22 / R_PATH
V_DES, V_E_SWITCH = 0.25, 0.42


class ZOHController(LeafSystem):
    """discrete controller at period dt: reads the plant state, solves the TFL-QP, holds the command"""
    def __init__(self, model, ctrl, dt, f0, events=()):
        super().__init__()
        self.model, self.ctrl, self.dt = model, ctrl, dt
        self.events = sorted(events, key=lambda e: e[0]); self.ev_i = 0
        nq, nv = model.nq, model.nv
        self.DeclareVectorInputPort('x', nq + nv)
        # discrete state: f_k, fdot_k, t_k and the held input u (7)
        self.istate = self.DeclareDiscreteState(np.concatenate([[f0, 0.0, 0.0], np.zeros(7)]))
        self.DeclareVectorOutputPort('cmd', 10, self.CalcOutput)
        self.DeclarePeriodicDiscreteUpdateEvent(dt, 0.0, self.Update)
        self.log = []

    def Update(self, context, discrete_state):
        xs = discrete_state.get_vector(self.istate).get_value().copy()
        f, fd = xs[0], xs[1]
        while self.ev_i < len(self.events) and context.get_time() >= self.events[self.ev_i][0] - 1e-9:
            self.events[self.ev_i][1](self.ctrl); self.ev_i += 1
        xp = self.get_input_port(0).Eval(context)
        x = np.concatenate([xp, [f, fd]])
        t0 = time.time(); u = self.ctrl(x); L = self.ctrl.last; solve_ms = 1e3 * (time.time() - t0)
        c = L['chains']; xi = L['xi']
        self.log.append(dict(t=context.get_time(), p_c=c['p_c'], p_e=c['p_e'], p_b=c['p_b'],
                             alpha_c=[xi['alpha_c1'][0], xi['alpha_c2'][0]], alpha_e=[xi['alpha_e1'][0], xi['alpha_e2'][0]],
                             eta_c=xi['beta_c'][:2], eta_e=xi['beta_e'][:2], psi=xi['psi'][0], th=c['th'], Phi=c['Phi'], f=f, u=u,
                             detJ=L['detJ'], detD=L['det_D'], delta=L['delta'], feasible=L['feasible'], cbf_margin=L['cbf_margin'],
                             active=L['active'], solve_ms=solve_ms, cheb_r=L.get('cheb_r', np.nan), n_active=L.get('n_active', 0),
                             int_lo=L.get('int_lo', np.nan), int_hi=L.get('int_hi', np.nan), int_width=L.get('int_width', np.nan), zero_margin=L.get('zero_margin', np.nan)))
        h = self.dt
        xs[0], xs[1], xs[2] = f + fd * h + 0.5 * u[0] * h * h, fd + u[0] * h, context.get_time() + h
        xs[3:] = u
        discrete_state.get_mutable_vector(self.istate).SetFromVector(xs)

    def CalcOutput(self, context, output):
        xs = context.get_discrete_state(self.istate).get_value()
        h = self.dt; u = xs[3:]
        f_k, fd_k, t_k = xs[0] - xs[1] * h + 0.5 * u[0] * h * h, xs[1] - u[0] * h, xs[2] - h
        output.SetFromVector(np.concatenate([[f_k, fd_k, t_k], u]))


class BodyFrameActuator(LeafSystem):
    """continuous map from the held body-frame command to the generalized force at the current attitude"""
    def __init__(self, model):
        super().__init__()
        self.model = model
        self.DeclareVectorInputPort('x', model.nq + model.nv)
        self.DeclareVectorInputPort('cmd', 10)
        self.DeclareVectorOutputPort('qf', model.nv, self.CalcOutput)

    def CalcOutput(self, context, output):
        x = self.get_input_port(0).Eval(context); cmd = self.get_input_port(1).Eval(context)
        f_k, fd_k, t_k, u = cmd[0], cmd[1], cmd[2], cmd[3:]
        s = max(context.get_time() - t_k, 0.0)
        f = f_k + fd_k * s + 0.5 * u[0] * s * s
        q = x[:self.model.nq]
        qn = q[:4] / np.linalg.norm(q[:4])
        R = RotationMatrix(Quaternion(qn[0], qn[1], qn[2], qn[3])).matrix()
        output.SetFromVector(self.model.generalized_force(R, f, u[1:4], u[4:7]))


def build(record=False, paths_draw=None):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    Parser(plant).AddModels(make_urdf()); plant.Finalize()
    meshcat = None; vis = None
    if record:
        meshcat = StartMeshcat()
        vis = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        if paths_draw is not None:
            for name, pts, rgba in paths_draw:
                for i, pt in enumerate(pts):
                    meshcat.SetObject(f"paths/{name}/{i}", Sphere(0.006), Rgba(*rgba))
                    meshcat.SetTransform(f"paths/{name}/{i}", RigidTransform(pt))
    return builder, plant, scene_graph, meshcat, vis


def run(ctrl_factory, x0_fn, T, name, dt, record=False, paths_draw=None, verbose=True, events=(), plant_scale=None):
    """plant_scale: {body_name: factor} applied to the mass and inertia of the simulated plant only"""
    builder, plant, scene_graph, meshcat, vis = build(record, paths_draw)
    model = DrakeAerialManipulator(plant=plant)
    ctrl = ctrl_factory(model)
    x0 = x0_fn(model)
    zoh = builder.AddSystem(ZOHController(model, ctrl, dt, x0[model.nq + model.nv], events))
    act = builder.AddSystem(BodyFrameActuator(model))
    builder.Connect(plant.get_state_output_port(), zoh.get_input_port(0))
    builder.Connect(plant.get_state_output_port(), act.get_input_port(0))
    builder.Connect(zoh.get_output_port(0), act.get_input_port(1))
    builder.Connect(act.get_output_port(0), plant.get_applied_generalized_force_input_port())
    diagram = builder.Build()
    sim = Simulator(diagram)
    ResetIntegratorFromFlags(sim, "runge_kutta3", 2.5e-4)
    sim.get_mutable_integrator().set_fixed_step_mode(True)
    ctx = sim.get_mutable_context(); pctx = plant.GetMyMutableContextFromRoot(ctx)
    if plant_scale:
        for bn, fac in plant_scale.items():
            body = plant.GetBodyByName(bn)
            M = body.CalcSpatialInertiaInBodyFrame(pctx)
            body.SetSpatialInertiaInBodyFrame(pctx, SpatialInertia(fac * M.get_mass(), M.get_com(), M.get_unit_inertia()))
        print(f"  plant mass {plant.CalcTotalMass(pctx):.4f} kg, controller model mass {model.mass:.4f} kg")
    plant.SetPositions(pctx, x0[:model.nq]); plant.SetVelocities(pctx, x0[model.nq:model.nq + model.nv])
    if record: vis.StartRecording()
    t0 = time.time(); nrep = int(T / 2.0)
    for k in range(nrep):
        try:
            sim.AdvanceTo((k + 1) * 2.0)
        except RuntimeError as ex:
            print(f"  ** simulation diverged near t={zoh.log[-1]['t']:.3f}s ({str(ex)[:60]}), stopping"); break
        L = zoh.log[-1]
        if verbose:
            print(f"  t={L['t']:5.2f}s |a_c|={np.linalg.norm(L['alpha_c']):.1e} |a_e|={np.linalg.norm(L['alpha_e']):.1e} "
                  f"eta_c={L['eta_c'][1]:.3f} eta_e={L['eta_e'][1]:.3f} th={np.round(L['th'],2)} detJ={L['detJ']:.2e} feas={L['feasible']} act={L['active']} ({time.time()-t0:.0f}s)")
    if record:
        vis.StopRecording(); vis.PublishRecording()
        open(os.path.join(RES, name + '_meshcat.html'), 'w').write(meshcat.StaticHtml())
    log = {k: np.array([e[k] for e in zoh.log], dtype=float) for k in zoh.log[0] if k != 'active'}
    np.savez(os.path.join(RES, name + '.npz'), **log)
    json.dump([list(e['active']) for e in zoh.log], open(os.path.join(RES, name + '_active.json'), 'w'))
    return log


def initial_state(model, p_c, p_e, psi, v=np.zeros(3), th0=(0.0, 0.3, -0.3)):
    """level attitude with yaw psi, center of mass at p_c, end effector at p_e (joint angles by Newton
    iteration from th0), velocity v of the body origin, hover thrust"""
    p, ctx = model.plant, model.ctx
    R = np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
    th = np.array(th0, float)
    for _ in range(100):
        p.SetFreeBodyPose(ctx, model.body, RigidTransform(RotationMatrix(R), np.zeros(3)))
        for k, j in enumerate(model.joints): j.set_angle(ctx, th[k])
        rho = R.T @ (p.CalcPointsPositions(ctx, model.ee, np.zeros(3), model.W).ravel() - p.CalcCenterOfMassPositionInWorld(ctx))
        r = rho - R.T @ (p_e - p_c)
        if np.linalg.norm(r) < 1e-12: break
        J = R.T @ (p.CalcJacobianTranslationalVelocity(ctx, JacobianWrtVariable.kV, model.ee, np.zeros(3), model.W, model.W)[:, model.jv]
                   - p.CalcJacobianCenterOfMassTranslationalVelocity(ctx, JacobianWrtVariable.kV, model.W, model.W)[:, model.jv])
        th = th - np.linalg.solve(J, r)
    pos = p_c - p.CalcCenterOfMassPositionInWorld(ctx)
    p.SetFreeBodyPose(ctx, model.body, RigidTransform(RotationMatrix(R), pos))
    vfull = np.zeros(model.nv); vfull[3:6] = v
    p.SetVelocities(ctx, vfull)
    return model.get_state(model.mass * model.g, 0.0)


def far_start(a0=0.0, dr=0.4, dz=-0.25):
    """start dr outside and dz below the vehicle path at polar angle a0; the end effector keeps its
    on-path offset from the center of mass plus a small perturbation"""
    r = R_PATH
    p_c_path = np.array([r * np.cos(a0), r * np.sin(a0), zc_of(a0)])
    p_e_path = np.array([r * np.cos(a0 + LEAD0), r * np.sin(a0 + LEAD0), ze_of(a0 + LEAD0)])
    e_r = np.array([np.cos(a0), np.sin(a0), 0.0])
    p_c0 = p_c_path + dr * e_r + np.array([0, 0, dz])
    p_e0 = p_c0 + (p_e_path - p_c_path) + 0.05 * e_r + np.array([0, 0, -0.04])
    return p_c0, p_e0


def paths_draw():
    aa = np.linspace(-np.pi, np.pi, 400)
    return [('com', np.c_[R_PATH * np.cos(aa), R_PATH * np.sin(aa), zc_of(aa)], (0.1, 0.3, 1.0, 1.0)),
            ('ee', np.c_[R_PATH * np.cos(aa), R_PATH * np.sin(aa), ze_of(aa)], (1.0, 0.1, 0.1, 1.0))]


def controller(model, mode):
    return DualPathTFL(model, PC_PATH, PE_PATH, V_DES, V_DES, PSI_REF, joint_limits=JOINT_LIMITS, mode=mode,
                       gains=GAINS, sing_eps=EPS_J)


def x0(model):
    p_c0, p_e0 = far_start()
    return initial_state(model, p_c0, p_e0, np.pi / 2 + 0.35, v=V_DES * R_PATH * np.array([0, 1, 0]))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument('scenario', choices=['D3', 'D4', 'D5'])
    ap.add_argument('--modes', default='relaxed', help='D4: comma-separated list of relaxed, filter, nominal')
    ap.add_argument('--hz', type=float, default=400.0, help='controller rate')
    ap.add_argument('--T', type=float, default=None, help='horizon in s (28 for D3, 40 otherwise)')
    ap.add_argument('--t-sw', type=float, default=15.0, help='time of the end-effector speed switch')
    ap.add_argument('--arm-scale', type=float, default=1.0, help='D5: factor on the arm masses and inertias of the plant')
    ap.add_argument('--veh-scale', type=float, default=1.0, help='D5: factor on the airframe mass and inertia of the plant')
    ap.add_argument('--tag', default='', help='suffix of the log name')
    ap.add_argument('--record', action='store_true', help='record a MeshCat animation')
    a = ap.parse_args()
    dt = 1.0 / a.hz
    switch = [(a.t_sw, lambda c: setattr(c, 'vdes_e', V_E_SWITCH))]
    if a.scenario == 'D3':
        run(lambda m: controller(m, 'nominal'), x0, a.T or 28.0, 'D3_nominal' + a.tag, dt, record=a.record, paths_draw=paths_draw())
    elif a.scenario == 'D4':
        for mode in a.modes.split(','):
            run(lambda m, mode=mode: controller(m, mode), x0, a.T or 40.0, f'D4_{mode}' + a.tag, dt, record=a.record,
                paths_draw=paths_draw(), events=switch)
    else:
        scale = {bn: a.arm_scale for bn in ('arm_base', 'link2', 'link3', 'link4', 'link5')}; scale['x2'] = a.veh_scale
        run(lambda m: controller(m, 'relaxed'), x0, a.T or 40.0, 'D5_mismatch' + a.tag, dt, record=a.record,
            paths_draw=paths_draw(), events=switch, plant_scale=scale)
    print('done')
