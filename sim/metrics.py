"""Every number quoted in Section VI of the paper, computed from the logs in results/.

    python3 metrics.py                      prints them
    import metrics; metrics.all_metrics()   returns them as a dictionary (used by the project page)

Distances d_c and d_e are measured to the points of the paths at the same polar angle, which bounds
the distances to the paths from above.
"""
import os, json, numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, 'results')
R_PATH, Z_C0, Z_E0 = 1.2, 1.6, 1.42
EPS_J = 1.5e-3
JOINT_LIMITS = [(-1.5, 1.5), (-1.3, 1.3), (-1.3, 1.2)]
T_SW = 15.0
POLE_THRUST = 10.0
F_MIN, F_MAX = 2.0, 40.0


def zc_of(a): return Z_C0 + 0.10 * np.sin(2 * a)
def ze_of(a): return Z_E0 + 0.06 * np.sin(2 * a + 0.8) + 0.03 * np.sin(8 * a)


def load(name):
    d = dict(np.load(os.path.join(RES, name + '.npz'), allow_pickle=True))
    try:
        d['active'] = json.load(open(os.path.join(RES, name + '_active.json')))
    except FileNotFoundError:
        d['active'] = None
    return d


def path_errors(L):
    """d_c, d_e in m: distance of p_c and p_e to the path point at the same polar angle"""
    pc, pe = L['p_c'], L['p_e']
    ac = np.arctan2(pc[:, 1], pc[:, 0]); ae = np.arctan2(pe[:, 1], pe[:, 0])
    erc = np.hypot(pc[:, 0], pc[:, 1]) - R_PATH; ezc = pc[:, 2] - zc_of(ac)
    ere = np.hypot(pe[:, 0], pe[:, 1]) - R_PATH; eze = pe[:, 2] - ze_of(ae)
    return np.hypot(erc, ezc), np.hypot(ere, eze)


def _first(t, mask):
    k = np.where(mask)[0]
    return float(t[k[0]]) if len(k) else None


def nominal_metrics(name='D3_nominal'):
    L = load(name); t = L['t']; dc, de = path_errors(L)
    bad = np.where(np.maximum(dc, de) > 1e-3)[0]
    t_mm = float(t[bad[-1]]) if len(bad) else 0.0
    after = t > t_mm
    return dict(T=float(t[-1]), d_c0=float(dc[0]), d_e0=float(de[0]), t_below_1mm=t_mm,
                max_dc_after_mm=1e3 * float(dc[after].max()), max_de_after_mm=1e3 * float(de[after].max()),
                heading_err_after_10s=float(np.abs(L['psi'][t > 10]).max()),
                eta_c_after_6s=[float(L['eta_c'][t > 6, 1].min()), float(L['eta_c'][t > 6, 1].max())],
                eta_e_after_6s=[float(L['eta_e'][t > 6, 1].min()), float(L['eta_e'][t > 6, 1].max())],
                max_tilt_deg=float(np.degrees(np.abs(L['Phi'][:, :2]).max())),
                thrust=[float(L['f'].min()), float(L['f'].max())],
                com_offset_cm=[1e2 * float(np.linalg.norm(L['p_c'] - L['p_b'], axis=1).min()),
                               1e2 * float(np.linalg.norm(L['p_c'] - L['p_b'], axis=1).max())])


def proposed_metrics(name='D4_relaxed', t_sw=T_SW):
    L = load(name); t = L['t']; dc, de = path_errors(L); act = L['active']
    dt = float(np.median(np.diff(t)))
    first = [(ti, a) for ti, a in zip(t, act) if a]
    t_act = float(first[0][0]); row_act = first[0][1][0]
    after_act = t >= t_act; after_sw = t > t_sw; late = t > 20.0
    names = [a[0] if a else '' for a in act]
    n_after = int(after_act.sum())
    share = {r: float(sum(1 for n, m in zip(names, after_act) if m and n == r) / n_after) for r in ('detJ', 'th2+')}
    any_active = float(sum(1 for a, m in zip(act, after_act) if m and a) / n_after)
    delta = L['delta'][:, 0]
    # thrust rows of the reduced program at the nominal input (zero coefficient on the slack):
    # -b_i = u_d + k1 (f - f_min) + k2 fdot and -u_d + k1 (f_max - f) - k2 fdot, with fdot by central differences
    k1, k2 = POLE_THRUST ** 2, 2 * POLE_THRUST
    fd = np.gradient(L['f'], t); ud = L['u'][:, 0]
    thr_lo = ud + k1 * (L['f'] - F_MIN) + k2 * fd; thr_hi = -ud + k1 * (F_MAX - L['f']) - k2 * fd
    # oscillation of the end-effector speed after 20 s
    y = L['eta_e'][late, 1] - L['eta_e'][late, 1].mean()
    y = y * np.hanning(len(y)); n = 64 * len(y)
    freqs = np.fft.rfftfreq(n, dt); spec = np.abs(np.fft.rfft(y, n)); spec[freqs < 0.1] = 0
    f_osc = float(freqs[np.argmax(spec)])
    f_res = 1.0 / (t[-1] - 20.0)
    tj = np.abs(L['u'][:, 4:7]).max()
    out = dict(T=float(t[-1]), rate_hz=1.0 / dt, t_activation=t_act, activation_row=row_act,
               max_slack=float(np.abs(delta).max()), max_slack_before_switch=float(np.abs(delta[t < t_sw]).max()),
               steps_active_after_activation=any_active, share_detJ=share['detJ'], share_th2=share['th2+'],
               barrier_residual=float(max(0.0, EPS_J - L['detJ'].min())), max_detJ_after_20s=float(L['detJ'][late].max()),
               peak_de_after_switch_mm=1e3 * float(de[after_sw].max()), peak_dc_after_switch_mm=1e3 * float(dc[after_sw].max()),
               de_after_20s_mm=1e3 * float(de[late].max()), dc_after_20s_mm=1e3 * float(dc[late].max()),
               t_de_below_half_mm=float(t[np.where(after_sw & (de > 5e-4))[0][-1]]),
               eta_e_mean_after_20s=float(L['eta_e'][late, 1].mean()),
               eta_e_range_after_20s=[float(L['eta_e'][late, 1].min()), float(L['eta_e'][late, 1].max())],
               eta_e_osc_hz=f_osc, eta_e_osc_resolution_hz=f_res, scallop_hz=8 * 0.25 / (2 * np.pi),
               eta_c_range_after_switch=[float(L['eta_c'][after_sw, 1].min()), float(L['eta_c'][after_sw, 1].max())],
               heading_err_after_10s=float(np.abs(L['psi'][t > 10]).max()),
               thrust_after_switch=[float(L['f'][after_sw].min()), float(L['f'][after_sw].max())],
               thrust_whole_run=[float(L['f'].min()), float(L['f'].max())],
               max_moment=float(np.abs(L['u'][:, 1:4]).max()), max_joint_torque=float(tj),
               max_tilt_deg=float(np.degrees(np.abs(L['Phi'][:, :2]).max())),
               feasible_all=bool(L['feasible'].all()),
               int_width_before_switch=float(np.nanmin(L['int_width'][t < t_sw])),
               int_width_after_switch=float(np.nanmin(L['int_width'][after_sw])),
               t_int_width_min=float(t[np.nanargmin(L['int_width'])]),
               attitude_row_margin=float(np.nanmin(L['zero_margin'])),
               thrust_row_margin=float(min(thr_lo.min(), thr_hi.min())),
               solve_ms_mean=float(L['solve_ms'].mean()), solve_ms_median=float(np.median(L['solve_ms'])),
               com_offset_cm=[1e2 * float(np.linalg.norm(L['p_c'] - L['p_b'], axis=1).min()),
                              1e2 * float(np.linalg.norm(L['p_c'] - L['p_b'], axis=1).max())],
               joint_violation=float(max(max(lo - L['th'][:, j].min(), L['th'][:, j].max() - hi, 0.0) for j, (lo, hi) in enumerate(JOINT_LIMITS))))
    out['int_width_over_slack'] = out['int_width_after_switch'] / out['max_slack']
    return out


def cascade_metrics(name='D4_filter', t_act=None, t_sw=T_SW):
    L = load(name); t = L['t']; dc, de = path_errors(L)
    if t_act is None:
        t_act = proposed_metrics()['t_activation']
    after = t > t_sw
    t_e5 = _first(t, after & (de > 0.05)); t_c5 = _first(t, after & (dc > 0.05))
    k_rev = int(np.argmin(L['eta_c'][:, 1]))
    return dict(T=float(t[-1]), t_activation_ref=t_act, t_ee_5cm_after_activation=t_e5 - t_act, t_vehicle_5cm_after_activation=t_c5 - t_act,
                min_vehicle_speed=float(L['eta_c'][:, 1].min()), t_min_vehicle_speed=float(t[k_rev]),
                max_thrust=float(L['f'].max()), max_moment=[float(v) for v in np.abs(L['u'][:, 1:4]).max(axis=0)],
                max_joint_torque=float(np.abs(L['u'][:, 4:7]).max()),
                min_dc_after_leaving_mm=1e3 * float(dc[t > t_c5].min()), min_de_after_leaving_mm=1e3 * float(de[t > t_e5].min()),
                final_dc_mm=1e3 * float(dc[-1]), final_de_mm=1e3 * float(de[-1]),
                joint_violation=float(max(max(lo - L['th'][:, j].min(), L['th'][:, j].max() - hi, 0.0) for j, (lo, hi) in enumerate(JOINT_LIMITS))),
                feasible_all=bool(L['feasible'].all()), t_first_infeasible=_first(t, ~L['feasible'].astype(bool)))


def nominal_switch_metrics(name='D4_nominal'):
    L = load(name); t = L['t']
    kz = np.where(L['detJ'] <= 0)[0]
    return dict(T=float(t[-1]), t_detJ_zero=float(t[kz[0]]) if len(kz) else None,
                th3_after_crossing=float(L['th'][kz[0], 2]) if len(kz) else None)


def coincidence_before_switch(t_sw=T_SW):
    """largest difference of the applied input between the three controllers before the switch"""
    P, F, N = load('D4_relaxed'), load('D4_filter'), load('D4_nominal')
    m = P['t'] < t_sw; n = int(m.sum())
    return dict(cascade_vs_proposed=float(np.abs(F['u'][:n] - P['u'][:n]).max()),
                nominal_vs_proposed=float(np.abs(N['u'][:n] - P['u'][:n]).max()),
                active_steps_before_switch=int(sum(1 for a, k in zip(P['active'], m) if k and a)))


def mismatch_metrics(name, t_sw=T_SW):
    L = load(name); t = L['t']; dc, de = path_errors(L)
    late = t > 20.0; after = t > t_sw
    feas = L['feasible'].astype(bool)
    out = dict(T=float(t[-1]), diverged=bool(t[-1] < 39.0), feasible_all=bool(feas.all()),
               t_first_infeasible=_first(t, ~feas), infeasible_share_after_switch=float((~feas[after]).mean()) if after.any() else None,
               barrier_residual=float(max(0.0, EPS_J - L['detJ'].min())), min_detJ=float(L['detJ'].min()),
               joint_violation=float(max(max(lo - L['th'][:, j].min(), L['th'][:, j].max() - hi, 0.0) for j, (lo, hi) in enumerate(JOINT_LIMITS))),
               peak_de_after_switch_cm=1e2 * float(de[after].max()) if after.any() else None,
               peak_dc_after_switch_cm=1e2 * float(dc[after].max()) if after.any() else None,
               max_speed_err_after_10s=float(np.abs(L['eta_c'][t > 10, 1] - 0.25).max()) if (t > 10).any() else None,
               max_tilt_deg=float(np.degrees(np.abs(L['Phi'][:, :2]).max())))
    if late.any():
        m10 = t > 10.0
        out.update(dc_after_20s_cm=[1e2 * float(dc[late].min()), 1e2 * float(dc[late].max())],
                   de_after_20s_cm=[1e2 * float(de[late].min()), 1e2 * float(de[late].max())],
                   dc_after_10s_cm=[1e2 * float(dc[m10].min()), 1e2 * float(dc[m10].max())],
                   de_after_10s_cm=[1e2 * float(de[m10].min()), 1e2 * float(de[m10].max())],
                   de_10_to_15s_cm=[1e2 * float(de[m10 & ~after].min()), 1e2 * float(de[m10 & ~after].max())])
    return out


def all_metrics():
    prop = proposed_metrics('D4_relaxed')
    m = dict(nominal=nominal_metrics('D3_nominal'), proposed=prop, proposed_1khz=proposed_metrics('D4_relaxed_1khz'),
             cascade=cascade_metrics('D4_filter', prop['t_activation']), nominal_switch=nominal_switch_metrics('D4_nominal'),
             before_switch=coincidence_before_switch(),
             mismatch={k: mismatch_metrics('D5_mismatch_' + k) for k in ('a102', 'v105', 'a105', 'v110')
                       if os.path.exists(os.path.join(RES, f'D5_mismatch_{k}.npz'))})
    m['one_khz'] = dict(activation_time_difference=abs(m['proposed_1khz']['t_activation'] - prop['t_activation']),
                        residual_ratio_de=prop['peak_de_after_switch_mm'] / m['proposed_1khz']['peak_de_after_switch_mm'],
                        residual_ratio_dc=prop['peak_dc_after_switch_mm'] / m['proposed_1khz']['peak_dc_after_switch_mm'],
                        residual_ratio_barrier=prop['barrier_residual'] / m['proposed_1khz']['barrier_residual'])
    return m


def _print(d, indent=0):
    for k, v in d.items():
        if isinstance(v, dict):
            print(' ' * indent + f'{k}:'); _print(v, indent + 2)
        elif isinstance(v, float):
            print(' ' * indent + f'{k:40s} {v:.6g}')
        elif isinstance(v, list):
            print(' ' * indent + f'{k:40s} [' + ', '.join(f'{x:.6g}' for x in v) + ']')
        else:
            print(' ' * indent + f'{k:40s} {v}')


if __name__ == '__main__':
    _print(all_metrics())
