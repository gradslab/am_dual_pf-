"""Offline rendering of a logged run with the VTK render engine of Drake.

    python3 drake/render_drake.py D4_relaxed --fps 30 --cam side
    python3 drake/render_drake.py D3_nominal --cam close --frame 6000

The log carries p_b, Phi and the joint angles at every controller step, which fixes the configuration.
The reference paths are drawn dotted and the traces of p_c and p_e solid, with depth occlusion from
the depth image of the sensor.
"""
import os, sys, argparse, numpy as np
from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph, Parser, RigidTransform, RotationMatrix, RollPitchYaw,
                         MakeRenderEngineVtk, RenderEngineVtkParams, RgbdSensor, DepthRenderCamera, ColorRenderCamera,
                         RenderCameraCore, CameraInfo, ClippingRange, DepthRange, LightParameter, Box, Rgba, GeometryInstance,
                         MakePhongIllustrationProperties, PerceptionProperties)
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here)
from make_realistic_urdf import make as make_urdf
RES = os.path.join(here, '..', 'results')
ANIM = os.path.join(RES, 'anim'); os.makedirs(ANIM, exist_ok=True)
W, H = 1280, 720


def camera_pose(cam, t, T, lookat=np.array([0.0, 0.0, 1.4]), p_c=None):
    """world pose of the camera (+z forward, +x right, +y down)"""
    if cam == 'close' and p_c is not None:
        lookat = p_c + np.array([0.0, 0.0, -0.12]); az, el, dist = np.radians(150.0), np.radians(-14.0), 1.9
    elif cam == 'side':      # near the start of the runs, outside the circle, so the convergence is visible
        lookat = np.array([0.4, 0.2, 1.15]); az, el, dist = np.radians(25.0), np.radians(-21.0), 3.9
    elif cam == 'orbit':
        lookat = np.array([0.0, 0.0, 1.2]); az, el, dist = np.radians(25.0 + 360.0 * t / T), np.radians(-22.0), 4.1
    else:
        az, el, dist = np.radians(135.0), np.radians(-18.0), 4.2
    eye = lookat + dist * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), -np.sin(el)])
    fwd = (lookat - eye); fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, np.array([0, 0, 1.0])); right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = np.c_[right, down, fwd]
    return RigidTransform(RotationMatrix(R), eye)


def build_scene():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    Parser(plant).AddModels(make_urdf())
    src = scene_graph.RegisterSource('env')
    ground = GeometryInstance(RigidTransform([0, 0, -0.025]), Box(12, 12, 0.05), 'ground')
    ground.set_illustration_properties(MakePhongIllustrationProperties(np.array([0.42, 0.45, 0.47, 1.0])))
    tex = os.path.join(RES, 'checker_ground.png')
    if not os.path.exists(tex):
        from PIL import Image
        n, cell = 48, 16
        a = np.zeros((n * cell, n * cell, 3), np.uint8)
        for i in range(n):
            for j in range(n):
                c = (168, 176, 182) if (i + j) % 2 == 0 else (140, 150, 156)
                a[i * cell:(i + 1) * cell, j * cell:(j + 1) * cell] = c
        Image.fromarray(a).save(tex)
    pp = PerceptionProperties(); pp.AddProperty('phong', 'diffuse', Rgba(1.0, 1.0, 1.0, 1.0)); pp.AddProperty('phong', 'diffuse_map', tex)
    ground.set_perception_properties(pp)
    scene_graph.RegisterAnchoredGeometry(src, ground)
    plant.Finalize()
    params = RenderEngineVtkParams()
    params.default_clear_color = [0.80, 0.88, 0.97]
    key = LightParameter(); key.type = 'directional'; key.direction = [0.45, 0.35, -1.0]; key.intensity = 1.0; key.frame = 'world'
    fill = LightParameter(); fill.type = 'directional'; fill.direction = [-0.5, -0.3, -1.0]; fill.intensity = 0.45; fill.frame = 'world'
    params.lights = [key, fill]
    params.cast_shadows = True; params.shadow_map_size = 4096
    scene_graph.AddRenderer('vtk', MakeRenderEngineVtk(params))
    intr = CameraInfo(W, H, np.radians(36.0))
    core = RenderCameraCore('vtk', intr, ClippingRange(0.05, 50.0), RigidTransform())
    color = ColorRenderCamera(core, False); depth = DepthRenderCamera(core, DepthRange(0.1, 40.0))
    sensor = builder.AddSystem(RgbdSensor(scene_graph.world_frame_id(), RigidTransform(), color, depth))
    builder.Connect(scene_graph.get_query_output_port(), sensor.query_object_input_port())
    diagram = builder.Build()
    return diagram, plant, scene_graph, sensor, intr


def set_config(plant, pctx, p_b, Phi, th):
    R = RollPitchYaw(Phi[0], Phi[1], Phi[2]).ToRotationMatrix()
    plant.SetFreeBodyPose(pctx, plant.GetBodyByName('x2'), RigidTransform(R, p_b))
    for k, n in enumerate(('joint1', 'joint2', 'joint3')): plant.GetJointByName(n).set_angle(pctx, th[k])


def project(X_WC, intr, pts):
    """pixel coordinates and camera depth of world points"""
    P = X_WC.inverse().multiply(pts.T).T
    z = P[:, 2]
    u = intr.focal_x() * P[:, 0] / z + intr.center_x(); v = intr.focal_y() * P[:, 1] / z + intr.center_y()
    return u, v, z


def draw_overlay(img, depth, X_WC, intr, paths, traces):
    from PIL import Image, ImageDraw
    im = Image.fromarray(img[:, :, :3]); dr = ImageDraw.Draw(im)
    def visible(u, v, z):
        ui, vi = int(round(u)), int(round(v))
        if z <= 0 or ui < 0 or vi < 0 or ui >= W or vi >= H: return False
        d = depth[vi, ui]
        return (not np.isfinite(d)) or d <= 0 or z < d + 0.02
    for pts, rgb in paths:                 # dotted reference paths
        u, v, z = project(X_WC, intr, pts)
        for ui, vi, zi in zip(u, v, z):
            if visible(ui, vi, zi):
                r = max(1.8, 7.0 / zi); dr.ellipse([ui - r, vi - r, ui + r, vi + r], fill=rgb)
    for pts, rgb in traces:                # solid traces
        if len(pts) < 2: continue
        u, v, z = project(X_WC, intr, pts)
        for i in range(len(pts) - 1):
            if visible(u[i], v[i], z[i]) and visible(u[i + 1], v[i + 1], z[i + 1]):
                dr.line([u[i], v[i], u[i + 1], v[i + 1]], fill=rgb, width=max(3, int(12.0 / z[i])))
    return np.asarray(im)


def paths_for(kind='circle'):
    r = 1.2; z_c0, z_e0 = 1.6, 1.42
    aa = np.linspace(-np.pi, np.pi, 300)
    zc = z_c0 + 0.10 * np.sin(2 * aa); ze = z_e0 + 0.06 * np.sin(2 * aa + 0.8) + 0.03 * np.sin(8 * aa)
    return [(np.c_[r * np.cos(aa), r * np.sin(aa), zc], (30, 80, 255)), (np.c_[r * np.cos(aa), r * np.sin(aa), ze], (255, 40, 40))]


def render(logname, fps=30, cam='orbit', one_frame=None):
    L = np.load(os.path.join(RES, logname + '.npz'))
    t = L['t']; T = float(t[-1])
    diagram, plant, scene_graph, sensor, intr = build_scene()
    ctx = diagram.CreateDefaultContext(); pctx = plant.GetMyMutableContextFromRoot(ctx); sctx = sensor.GetMyMutableContextFromRoot(ctx)
    paths = paths_for()
    frames = []; trace_c = []; trace_e = []
    ks = [one_frame] if one_frame is not None else list(np.searchsorted(t, np.arange(0, T, 1.0 / fps)))
    for n, k in enumerate(ks):
        set_config(plant, pctx, L['p_b'][k], L['Phi'][k], L['th'][k])
        trace_c.append(L['p_c'][k]); trace_e.append(L['p_e'][k])
        X_WC = camera_pose(cam, t[k], T, p_c=L['p_c'][k])
        if hasattr(sensor, 'SetX_PB'): sensor.SetX_PB(sctx, X_WC)
        elif n == 0: print('WARNING: RgbdSensor.SetX_PB unavailable, camera fixed')
        img = sensor.color_image_output_port().Eval(sctx).data.copy()
        dep = sensor.depth_image_32F_output_port().Eval(sctx).data.copy()[:, :, 0]
        frame = draw_overlay(img, dep, X_WC, intr, paths, [(np.array(trace_c), (30, 80, 255)), (np.array(trace_e), (255, 40, 40))])
        frames.append(frame)
        if n % 100 == 0: print(f'  frame {n}/{len(ks)}')
    if one_frame is not None:
        from PIL import Image
        k = ks[0]; X_WC = camera_pose(cam, t[k], T, p_c=L['p_c'][k])
        for lab, pt in (('p_c', L['p_c'][k]), ('p_e', L['p_e'][k])):
            u, v, z = project(X_WC, intr, pt[None, :]); print(f'{lab} pixel ({u[0]:.0f}, {v[0]:.0f}) depth {z[0]:.2f}')
        for lab, pts in (('gamma_c', paths[0][0]), ('gamma_e', paths[1][0])):
            u, v, z = project(X_WC, intr, pts); ok = (u > 40) & (u < W - 40) & (v > 40) & (v < H - 40) & (z > 0)
            print(f'{lab} visible pixel range u [{u[ok].min():.0f},{u[ok].max():.0f}] v [{v[ok].min():.0f},{v[ok].max():.0f}]; sample:', [(int(a), int(b)) for a, b in zip(u[ok][::40], v[ok][::40])])
        out = os.path.join(ANIM, f'{logname}_{cam}_frame{one_frame}.png'); Image.fromarray(frames[0]).save(out); print('wrote', out); return
    import imageio
    out = os.path.join(ANIM, f'{logname}_{cam}_drake.mp4')
    imageio.mimsave(out, frames, fps=fps, codec='libx264', quality=8, macro_block_size=None); print('wrote', out)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('log', help='log name in results/, for example D4_relaxed')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--cam', default='orbit', choices=['orbit', 'side', 'close', 'iso'])
    ap.add_argument('--frame', type=int, default=None, help='render this controller step to a PNG instead of a video')
    a = ap.parse_args()
    render(a.log, a.fps, a.cam, a.frame)
