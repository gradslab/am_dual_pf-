"""Figures of the project page, drawn from the logs and the study outputs in results/.
Written as PNG to results/figs/ and copied to docs/figs/ by build_page.py.

    python3 web/figs_web.py
"""
import os, sys, json, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams

HERE = os.path.dirname(os.path.abspath(__file__)); SIM = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, SIM)
from metrics import load, path_errors, EPS_J, JOINT_LIMITS, T_SW, F_MIN, F_MAX
RES = os.path.join(SIM, 'results'); FIG = os.path.join(RES, 'figs'); STU = os.path.join(RES, 'studies')
os.makedirs(FIG, exist_ok=True)

rcParams.update({'font.size': 11, 'font.family': 'sans-serif', 'mathtext.fontset': 'dejavusans', 'axes.labelsize': 11,
                 'legend.fontsize': 10, 'xtick.labelsize': 10, 'ytick.labelsize': 10, 'lines.linewidth': 1.4,
                 'axes.grid': True, 'grid.alpha': 0.3, 'figure.dpi': 160, 'savefig.dpi': 160, 'axes.titlesize': 11.5})
C_PROP, C_FILT, C_NOM, C_K = '#1f4e9c', '#d9480f', '#2a9d8f', '#333333'
C_1K = '#8e44ad'


def save(fig, name):
    out = os.path.join(FIG, name + '.png')
    fig.savefig(out, bbox_inches='tight', pad_inches=0.05, facecolor='white'); plt.close(fig); print('wrote', out)


def refs(n=400):
    a = np.linspace(-np.pi, np.pi, n)
    zc = 1.6 + 0.10 * np.sin(2 * a); ze = 1.42 + 0.06 * np.sin(2 * a + 0.8) + 0.03 * np.sin(8 * a)
    return np.c_[1.2 * np.cos(a), 1.2 * np.sin(a), zc], np.c_[1.2 * np.cos(a), 1.2 * np.sin(a), ze]


def fig_nominal(name='D3_nominal'):
    L = load(name); t = L['t']; dc, de = path_errors(L); pc_ref, pe_ref = refs()
    fig = plt.figure(figsize=(10.5, 4.2))
    ax = fig.add_axes([0.0, -0.02, 0.5, 1.06], projection='3d')
    ax.plot(*pc_ref.T, '-', color=C_PROP, lw=5, alpha=0.22, label=r'$\gamma_c$')
    ax.plot(*pe_ref.T, '-', color=C_FILT, lw=5, alpha=0.22, label=r'$\gamma_e$')
    ax.plot(*L['p_c'].T, color=C_PROP, lw=1.3, label=r'$p_c$'); ax.plot(*L['p_e'].T, color=C_FILT, lw=1.3, label=r'$p_e$')
    ax.scatter(*L['p_c'][0], color=C_PROP, s=30, zorder=5); ax.scatter(*L['p_e'][0], color=C_FILT, s=30, zorder=5)
    ax.set_xlabel('x [m]', labelpad=-4); ax.set_ylabel('y [m]', labelpad=-4); ax.set_zlabel('z [m]', labelpad=-6)
    ax.tick_params(pad=-2); ax.view_init(elev=26, azim=-52)
    ax.set_xticks([-1, 0, 1]); ax.set_yticks([-1, 0, 1]); ax.set_zticks([1.4, 1.6])
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.0), ncol=4, frameon=False, handlelength=1.2)
    ax.set_box_aspect((1, 1, 0.45)); ax.grid(False)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane): pane.fill = False
    a1 = fig.add_axes([0.6, 0.57, 0.38, 0.38]); a2 = fig.add_axes([0.6, 0.1, 0.38, 0.38])
    a1.semilogy(t, dc + 1e-13, color=C_PROP, label=r'$d_c$'); a1.semilogy(t, de + 1e-13, color=C_FILT, label=r'$d_e$')
    a1.set_ylabel('distance to path [m]'); a1.set_ylim(1e-7, 1); a1.set_yticks([1e-6, 1e-4, 1e-2, 1]); a1.set_xticklabels([])
    a1.legend(frameon=False, loc='upper right')
    a2.plot(t, L['eta_c'][:, 1], color=C_PROP, label=r'$\eta_2$ (vehicle speed)'); a2.plot(t, L['eta_e'][:, 1], color=C_FILT, label=r'$\varsigma_2$ (end-effector speed)')
    a2.plot(t, L['psi'], color=C_K, lw=1.1, label=r'$\mu_1-\psi_{\rm des}$ (heading error)')
    a2.set_ylabel('[rad/s], [rad]'); a2.set_xlabel('t [s]'); a2.set_ylim(-0.03, 0.62); a2.legend(frameon=False, loc='upper right')
    save(fig, 'W_nominal')


def fig_relaxed(names=('D4_relaxed', 'D4_filter', 'D4_nominal'), T=40.0):
    labels = ['proposed', 'TFL + safety filter', 'nominal TFL']; styles = ['-', '--', ':']; cols = [C_PROP, C_FILT, C_NOM]; clip = [False, False, True]
    fig, ax = plt.subplots(3, 2, figsize=(10.5, 7.2), sharex=True); ax = ax.ravel(); marks = []
    for n, lab, s, c, cl in zip(names, labels, styles, cols, clip):
        L = load(n); t = L['t']; dc, de = path_errors(L)
        bad = np.where((np.degrees(np.abs(L['Phi'][:, :2]).max(axis=1)) > 60) | (np.maximum(dc, de) > 2.0))[0] if cl else []
        k = bad[0] if len(bad) else len(t)
        t, dc, de = t[:k], dc[:k], de[:k]
        if k < len(L['t']):
            marks.append((c, t[-1], (dc[-1] + 1e-13, de[-1] + 1e-13, L['eta_c'][k-1, 1], L['eta_e'][k-1, 1], L['th'][k-1, 1], 1e3 * L['detJ'][k-1])))
        ax[0].semilogy(t, dc + 1e-13, s, color=c, label=lab, lw=1.6); ax[1].semilogy(t, de + 1e-13, s, color=c, lw=1.6)
        ax[2].plot(t, L['eta_c'][:k, 1], s, color=c, lw=1.6); ax[3].plot(t, L['eta_e'][:k, 1], s, color=c, lw=1.6)
        ax[4].plot(t, L['th'][:k, 1], s, color=c, lw=1.6); ax[5].plot(t, 1e3 * L['detJ'][:k], s, color=c, lw=1.6)
    ax[0].set_ylabel(r'$d_c$ [m]'); ax[1].set_ylabel(r'$d_e$ [m]'); ax[2].set_ylabel(r'$\eta_2$ [rad/s]'); ax[3].set_ylabel(r'$\varsigma_2$ [rad/s]')
    ax[4].set_ylabel(r'$\vartheta_2$ [rad]'); ax[5].set_ylabel(r'$\det J_\rho$ [$10^{-3}$ m$^3$]')
    for a in (ax[4],): a.axhline(1.3, color='k', lw=0.8, ls='-.'); a.axhline(-1.3, color='k', lw=0.8, ls='-.')
    ax[5].axhline(1e3 * EPS_J, color='k', lw=0.8, ls='-.'); ax[5].axhline(0, color='k', lw=0.5)
    ax[2].axhline(0.25, color='gray', lw=0.8, ls=':'); ax[3].plot([0, T_SW, T_SW, T], [0.25, 0.25, 0.42, 0.42], color='gray', lw=0.8, ls=':')
    for a in ax[:2]: a.set_ylim(1e-7, 50); a.set_yticks([1e-6, 1e-3, 1])
    ax[2].set_ylim(0.15, 0.30); ax[3].set_ylim(-0.3, 0.6); ax[4].set_ylim(-1.55, 1.75); ax[5].set_ylim(-1.5, 7)
    for a in ax: a.axvline(T_SW, color='0.55', lw=1.0, alpha=0.8)
    for c, tm, ys in marks:
        for a, y in zip(ax, ys):
            lo, hi = a.get_ylim(); a.plot([tm], [min(max(y, lo), hi)], 'x', color=c, ms=7, mew=1.5)
    ax[4].set_xlabel('t [s]'); ax[5].set_xlabel('t [s]'); ax[4].set_xlim(0, T)
    h, l = ax[0].get_legend_handles_labels()
    fig.legend(h, l, loc='upper center', ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.0))
    fig.tight_layout(rect=(0, 0, 1, 0.96)); save(fig, 'W_relaxed')


def fig_cascade(names=('D4_relaxed', 'D4_filter')):
    P, F = load(names[0]), load(names[1])
    fig, ax = plt.subplots(1, 3, figsize=(10.5, 3.3))
    for L, lab, s, c in ((P, 'proposed', '-', C_PROP), (F, 'TFL + safety filter', '--', C_FILT)):
        t = L['t']
        ax[0].plot(t, L['eta_c'][:, 1], s, color=c, label=lab)
        ax[1].plot(t, L['f'], s, color=c)
        ax[2].plot(t, np.abs(L['u'][:, 1:4]).max(axis=1), s, color=c)
    ax[0].set_ylabel(r'vehicle speed $\eta_2$ [rad/s]'); ax[1].set_ylabel(r'thrust $u_t$ [N]'); ax[2].set_ylabel(r'largest $|\tau_i|$ [Nm]')
    ax[1].axhline(F_MAX, color='k', lw=0.8, ls='-.'); ax[1].axhline(F_MIN, color='k', lw=0.8, ls='-.')
    ax[2].axhline(3.0, color='k', lw=0.8, ls='-.')
    for a in ax: a.set_xlabel('t [s]'); a.axvline(T_SW, color='0.55', lw=1.0, alpha=0.8); a.set_xlim(0, 40)
    ax[0].legend(frameon=False, loc='lower left'); fig.tight_layout(); save(fig, 'W_cascade')


def fig_interval(name='D4_relaxed'):
    L = load(name); t = L['t']; lo, hi, d = L['int_lo'], L['int_hi'], L['delta'][:, 0]; act = L['active']
    fig, ax = plt.subplots(1, 2, figsize=(10.5, 3.6))
    for a, (t0, t1, y0, y1) in zip(ax, ((0, 40, -12, 12), (14, 40, -6, 2.5))):
        a.fill_between(t, np.clip(lo, y0, y1), np.clip(hi, y0, y1), color='0.88', label=r'feasible interval $[\delta_{\min},\delta_{\max}]$')
        a.plot(t, lo, color=C_PROP, lw=1.2, label=r'largest lower bound $\delta_{\min}$'); a.plot(t, hi, color=C_FILT, lw=1.2, label=r'smallest upper bound $\delta_{\max}$')
        a.plot(t, d, color='k', lw=1.4, label=r'applied slack $\delta^\star$')
        a.axvline(T_SW, color='0.55', lw=1.0, alpha=0.8); a.set_xlim(t0, t1); a.set_ylim(y0, y1); a.set_xlabel('t [s]'); a.set_ylabel(r'[rad/s$^2$]')
    rows = np.array([x[0] if x else '' for x in act])
    for nm, c, lab in (('detJ', C_FILT, r'bound set by the row of $\det J_\rho$'), ('th2+', C_NOM, r'bound set by the upper limit of $\vartheta_2$')):
        k = rows == nm; ax[1].plot(t[k], hi[k], '.', color=c, ms=3, label=lab)
    h, l = ax[1].get_legend_handles_labels()
    fig.legend(h, l, loc='upper center', ncol=3, frameon=False, fontsize=9.5, bbox_to_anchor=(0.5, 1.02))
    ax[0].set_title('whole run'); ax[1].set_title('after the switch')
    fig.tight_layout(rect=(0, 0, 1, 0.9)); save(fig, 'W_interval')


def fig_window():
    W = np.load(os.path.join(STU, 'lead_window.npz')); L = load('D4_relaxed'); t = L['t']
    acs, leads, win = W['acs'], W['leads'], W['win']
    ac = np.arctan2(L['p_c'][:, 1], L['p_c'][:, 0]); ae = np.arctan2(L['p_e'][:, 1], L['p_e'][:, 0]); lead = (ae - ac + np.pi) % (2 * np.pi) - np.pi
    fig, ax = plt.subplots(figsize=(10.5, 4.2))
    A, Ld = np.meshgrid(np.r_[acs, np.pi], leads, indexing='ij'); Wn = np.vstack([win, win[:1]])
    ax.contourf(A, Ld, Wn, levels=[0.5, 1.5], colors=['#cfe4d4'])
    ax.plot(np.r_[acs, np.pi], np.r_[W['lo_b'], W['lo_b'][0]], color=C_PROP, lw=1.6, label='lower end (joint limit)')
    ax.plot(np.r_[acs, np.pi], np.r_[W['hi_b'], W['hi_b'][0]], color=C_FILT, lw=1.6, label=r'upper end ($\det J_\rho = \epsilon_J$ or joint limit)')
    def segs(mask, label, **kw):
        idx = np.where(mask)[0]; brk = np.where(np.abs(np.diff(ac[idx])) > np.pi)[0]
        for i, seg in enumerate(np.split(idx, brk + 1)):
            ax.plot(ac[seg], lead[seg], label=label if i == 0 else None, **kw)
    segs(t < T_SW, 'run, t < 15 s', color='k', lw=1.5); segs(t >= T_SW, 'run, t > 15 s', color='k', lw=1.5, ls='--')
    ax.set_xlim(-np.pi, np.pi); ax.set_ylim(0.0, 0.45); ax.set_xlabel(r'vehicle path coordinate $a_c$ [rad]'); ax.set_ylabel(r'lead $a_e - a_c$ of the end effector [rad]')
    ax.legend(frameon=False, loc='upper left', ncol=2); fig.tight_layout(); save(fig, 'W_window')


def fig_sampling():
    S = np.load(os.path.join(STU, 'sampling.npz'))
    fig, ax = plt.subplots(1, 3, figsize=(10.5, 3.5), sharey=True)
    for a, s2 in zip(ax, (0.25, -0.5, 1.5)):
        k = S['s2'] == s2; inO = S['adm'] >= 0; strict = S['strict'] > 0
        a.plot(S['bc'][k & ~inO], S['lead'][k & ~inO], '.', color='0.75', ms=5, label='outside the admissible set')
        a.plot(S['bc'][k & inO & strict], S['lead'][k & inO & strict], 'o', color=C_PROP, ms=3.5, label='strictly feasible')
        a.plot(S['bc'][k & inO & ~strict], S['lead'][k & inO & ~strict], 'x', color=C_FILT, ms=6, mew=1.5, label='not strictly feasible')
        a.set_title(f'end-effector speed {s2:g} rad/s'); a.set_xlabel(r'$a_c$ [rad]')
    ax[0].set_ylabel('lead [rad]')
    h, l = ax[0].get_legend_handles_labels()
    fig.legend(h, l, loc='lower center', ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.04))
    fig.tight_layout(rect=(0, 0.05, 1, 1)); save(fig, 'W_sampling')


def fig_rate(names=('D4_relaxed', 'D4_relaxed_1khz')):
    fig, ax = plt.subplots(2, 2, figsize=(10.5, 6.2)); ax = ax.ravel()
    for n, lab, c, lw in zip(names, ('400 Hz', '1 kHz'), (C_PROP, C_1K), (1.8, 1.2)):
        L = load(n); t = L['t']; dc, de = path_errors(L)
        ax[0].semilogy(t, de + 1e-13, color=c, lw=lw, label=lab); ax[1].semilogy(t, dc + 1e-13, color=c, lw=lw)
        ax[2].plot(t, L['delta'][:, 0], color=c, lw=lw); ax[3].plot(t, 1e6 * (EPS_J - L['detJ']), color=c, lw=lw)
    ax[0].set_ylabel(r'$d_e$ [m]'); ax[1].set_ylabel(r'$d_c$ [m]'); ax[2].set_ylabel(r'slack $\delta^\star$ [rad/s$^2$]'); ax[3].set_ylabel(r'$\epsilon_J - \det J_\rho$ [$10^{-6}$ m$^3$]')
    for a in ax[:2]: a.set_ylim(1e-7, 1e-1); a.set_xlim(10, 40)
    ax[2].set_xlim(15, 25); ax[3].set_xlim(15, 25); ax[3].set_ylim(-60, 5); ax[3].axhline(0, color='k', lw=0.8, ls='-.')
    for a in ax: a.set_xlabel('t [s]'); a.axvline(T_SW, color='0.55', lw=1.0, alpha=0.8)
    ax[0].legend(frameon=False); fig.tight_layout(); save(fig, 'W_rate')


def fig_actuator(name='D4_relaxed'):
    L = load(name); t = L['t']
    fig, ax = plt.subplots(1, 3, figsize=(10.5, 3.3))
    ax[0].plot(t, L['f'], color=C_PROP); ax[0].set_ylabel(r'thrust $u_t$ [N]')
    for i, lab in enumerate((r'$\tau_1$', r'$\tau_2$', r'$\tau_3$')): ax[1].plot(t, L['u'][:, 1 + i], lw=1.1, label=lab)
    for i, lab in enumerate((r'$\tau_{\vartheta,1}$', r'$\tau_{\vartheta,2}$', r'$\tau_{\vartheta,3}$')): ax[2].plot(t, L['u'][:, 4 + i], lw=1.1, label=lab)
    ax[1].set_ylabel('body moments [Nm]'); ax[2].set_ylabel('joint torques [Nm]')
    ax[1].legend(frameon=False, ncol=3, loc='upper left', fontsize=9); ax[2].legend(frameon=False, ncol=3, loc='upper left', fontsize=9)
    ax[1].set_ylim(-1.3, 1.6); ax[2].set_ylim(-1.1, 1.3)
    for a in ax: a.set_xlabel('t [s]'); a.axvline(T_SW, color='0.55', lw=1.0, alpha=0.8); a.set_xlim(0, 40)
    fig.tight_layout(); save(fig, 'W_actuator')


def fig_mismatch():
    runs = (('D4_relaxed', 'exact model', C_PROP, '-'), ('D5_mismatch_a102', 'arm +2 %', C_FILT, '-'), ('D5_mismatch_v105', 'airframe +5 %', C_NOM, '--'))
    fig, ax = plt.subplots(2, 2, figsize=(10.5, 6.4)); ax = ax.ravel()
    for n, lab, c, s in runs:
        L = load(n); t = L['t']; dc, de = path_errors(L)
        ax[0].plot(t, 1e2 * dc, s, color=c, label=lab); ax[1].plot(t, 1e2 * de, s, color=c)
        ax[2].plot(t, L['th'][:, 1], s, color=c, lw=1.3, label=lab)
    for n, lab, c in (('D5_mismatch_a105', 'arm +5 %', C_FILT), ('D5_mismatch_v110', 'airframe +10 %', C_NOM)):
        L = load(n); t = L['t']; dc, de = path_errors(L)
        ok = np.isfinite(dc) & np.isfinite(de) & (np.maximum(dc, de) < 2.0); k = np.where(~ok)[0]; k = k[0] if len(k) else len(t)
        ax[3].semilogy(t[:k], dc[:k] + 1e-13, color=c, label=lab + r', $d_c$'); ax[3].semilogy(t[:k], de[:k] + 1e-13, color=c, alpha=0.5, label=lab + r', $d_e$')
        ax[3].plot([t[k - 1]], [max(dc[k - 1], 1e-13)], 'x', color=c, ms=8, mew=1.6)
    ax[0].set_ylabel(r'$d_c$ [cm]'); ax[1].set_ylabel(r'$d_e$ [cm]'); ax[0].set_ylim(0, 25); ax[1].set_ylim(0, 25)
    ax[2].set_ylabel(r'shoulder angle $\vartheta_2$ [rad]'); ax[2].axhline(1.3, color='k', lw=0.8, ls='-.'); ax[2].axhline(-1.3, color='k', lw=0.8, ls='-.'); ax[2].set_ylim(-1.5, 2.6)
    ax[3].set_ylabel('distance to path [m]'); ax[3].set_ylim(3e-2, 2); ax[3].set_xlim(0, 5)
    for a in ax[:3]: a.axvline(T_SW, color='0.55', lw=1.0, alpha=0.8); a.set_xlim(0, 40)
    for a in ax: a.set_xlabel('t [s]')
    ax[0].legend(frameon=False); ax[2].legend(frameon=False, ncol=3, fontsize=9.5, loc='upper left'); ax[3].legend(frameon=False, fontsize=9, loc='lower left')
    ax[0].set_title('center of mass'); ax[1].set_title('end effector'); ax[2].set_title('shoulder angle and its limits'); ax[3].set_title('the two diverging cases (cross: end of the run)')
    fig.tight_layout(); save(fig, 'W_mismatch')


if __name__ == '__main__':
    fig_nominal(); fig_relaxed(); fig_cascade(); fig_interval(); fig_window(); fig_sampling(); fig_rate(); fig_actuator(); fig_mismatch()
