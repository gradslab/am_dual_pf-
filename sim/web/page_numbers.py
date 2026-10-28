"""Collect every number the project page quotes into results/web/numbers.json.

    python3 web/page_numbers.py

The page is rendered from this file, so a rerun that changes a result changes the page.
"""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__)); SIM = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, SIM); sys.path.insert(0, os.path.join(SIM, 'drake')); sys.path.insert(0, os.path.join(SIM, 'model'))
import metrics as M
OUT = os.path.join(SIM, 'results', 'web'); os.makedirs(OUT, exist_ok=True)


def parameters():
    import sim_drake as SD
    from am_controller import DualPathTFL
    poles = dict(lam4=SD.GAINS['lam4'], lam2=SD.GAINS['lam2'], lam_eta=SD.GAINS['lam_eta'], k_be=SD.GAINS['k_be'], lam_psi=SD.GAINS['lam_psi'])
    c = DualPathTFL.__init__.__defaults__
    return dict(joint_limits=SD.JOINT_LIMITS, eps_J=SD.EPS_J, poles=poles, cbf_pole=10.0, eps_att=0.1, f_min=2.0, f_max=40.0,
                tau_max=[3.0, 3.0, 1.0], tauj_max=[3.0, 3.0, 3.0], v_des=SD.V_DES, v_e_switch=SD.V_E_SWITCH, t_switch=15.0,
                r_path=SD.R_PATH, z_c0=SD.Z_C0, z_e0=SD.Z_E0, lead0=SD.LEAD0, ctrl_hz=400, plant_hz=4000, start=dict(dr=0.4, dz=-0.25, psi_err=0.35),
                gains=dict(k_xi=[poles['lam4'] ** 4, 4 * poles['lam4'] ** 3, 6 * poles['lam4'] ** 2, 4 * poles['lam4']],
                           k_chi=[poles['lam2'] ** 2, 2 * poles['lam2']],
                           k_eta=[poles['lam_eta'] ** 3, 3 * poles['lam_eta'] ** 2, 3 * poles['lam_eta']],
                           k_varsigma=poles['k_be'], k_mu=[poles['lam_psi'] ** 2, 2 * poles['lam_psi']],
                           k_cbf=[100.0, 20.0]))


def main():
    N = dict(metrics=M.all_metrics(), params=parameters())
    N['study'] = json.load(open(os.path.join(SIM, 'results', 'studies', 'posture_study.json')))
    sizes = os.path.join(OUT, 'anim_sizes.json')
    N['anim_mb'] = json.load(open(sizes)) if os.path.exists(sizes) else {}
    json.dump(N, open(os.path.join(OUT, 'numbers.json'), 'w'), indent=1)
    print('wrote', os.path.join(OUT, 'numbers.json'))


if __name__ == '__main__':
    main()
