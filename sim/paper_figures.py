"""The three figures of the paper, from the logs in results/. Written to results/figs/ as PDF and PNG.

    python3 paper_figures.py            all three
    python3 paper_figures.py nominal    one of concept, nominal, relaxed
"""
import os, sys, json, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams.update({'font.size': 8, 'font.family': 'serif', 'mathtext.fontset': 'cm', 'axes.labelsize': 8, 'legend.fontsize': 7.5,
                 'xtick.labelsize': 7.5, 'ytick.labelsize': 7.5, 'lines.linewidth': 1.0, 'axes.grid': True, 'grid.alpha': 0.3})
here = os.path.dirname(os.path.abspath(__file__)); RES = os.path.join(here, 'results')
FIG = os.path.join(RES, 'figs'); os.makedirs(FIG, exist_ok=True)
sys.path.insert(0, here)
from metrics import load, path_errors
C_PROP, C_FILT, C_NOM = '#1f4e9c', '#d9480f', '#2a9d8f'


def circle_refs(r=1.2, z_c0=1.6, z_e0=1.42, n=400):
    a = np.linspace(-np.pi, np.pi, n)
    zc = z_c0 + 0.10 * np.sin(2 * a); ze = z_e0 + 0.06 * np.sin(2 * a + 0.8) + 0.03 * np.sin(8 * a)
    return np.c_[r * np.cos(a), r * np.sin(a), zc], np.c_[r * np.cos(a), r * np.sin(a), ze]


def fig_nominal(name='D3_nominal', out='fig_nominal.pdf'):
    L = load(name); t = L['t']
    pc_ref, pe_ref = circle_refs()
    dc, de = path_errors(L)
    fig = plt.figure(figsize=(3.45, 1.95))
    ax = fig.add_axes([-0.04, -0.04, 0.50, 1.08], projection="3d")
    ax.plot(*pc_ref.T, '-', color=C_PROP, lw=3.2, alpha=0.25, label=r'$\gamma_c$')
    ax.plot(*pe_ref.T, '-', color=C_FILT, lw=3.2, alpha=0.25, label=r'$\gamma_e$')
    ax.plot(*L['p_c'].T, color=C_PROP, lw=0.9, label=r'$p_c$')
    ax.plot(*L['p_e'].T, color=C_FILT, lw=0.9, label=r'$p_e$')
    ax.scatter(*L['p_c'][0], color=C_PROP, s=16, zorder=5); ax.scatter(*L['p_e'][0], color=C_FILT, s=16, zorder=5)
    ax.set_xlabel('x [m]', labelpad=-6, fontsize=8); ax.set_ylabel('y [m]', labelpad=-6, fontsize=8); ax.set_zlabel('z [m]', labelpad=-8, fontsize=8)
    ax.tick_params(pad=-3, labelsize=7.5); ax.view_init(elev=26, azim=-52)
    ax.set_xticks([-1, 0, 1]); ax.set_yticks([-1, 0, 1]); ax.set_zticks([1.4, 1.6])
    ax.legend(loc='upper center', bbox_to_anchor=(0.52, 1.02), ncol=4, columnspacing=0.5, handlelength=1.0, frameon=False, fontsize=8)
    ax.set_box_aspect((1, 1, 0.45)); ax.grid(False)
    ax.xaxis.pane.fill = False; ax.yaxis.pane.fill = False; ax.zaxis.pane.fill = False
    a1 = fig.add_axes([0.745, 0.58, 0.255, 0.39]); a2 = fig.add_axes([0.745, 0.12, 0.255, 0.39])
    a1.semilogy(t, dc + 1e-13, color=C_PROP, label=r'$d_c$'); a1.semilogy(t, de + 1e-13, color=C_FILT, label=r'$d_e$')
    a1.set_ylabel('distance [m]', labelpad=1); a1.set_ylim(1e-7, 1); a1.set_yticks([1e-6, 1e-4, 1e-2, 1])
    a1.legend(frameon=False, ncol=1, handlelength=1.0, loc='upper right', bbox_to_anchor=(1.02, 1.02), fontsize=7.5, labelspacing=0.2); a1.set_xticklabels([])
    a2.plot(t, L['eta_c'][:, 1], color=C_PROP, label=r'$\eta_2$'); a2.plot(t, L['eta_e'][:, 1], color=C_FILT, label=r'$\varsigma_2$')
    a2.plot(t, L['psi'], color='k', lw=0.8, label=r'$\mu_1\!-\!\psi_{\rm des}$')
    a2.set_ylabel('[rad/s], [rad]', labelpad=1); a2.set_xlabel('t [s]', labelpad=0); a2.set_ylim(-0.03, 0.62)
    a2.legend(frameon=False, ncol=1, handlelength=1.0, loc='upper right', bbox_to_anchor=(1.03, 1.04), fontsize=7.5, labelspacing=0.05)
    save(fig, out)


def save(fig, out, **kw):
    fig.savefig(os.path.join(FIG, out), bbox_inches='tight', pad_inches=kw.get('pad', 0.02), dpi=kw.get('dpi', None))
    fig.savefig(os.path.join(FIG, os.path.splitext(out)[0] + '.png'), bbox_inches='tight', pad_inches=kw.get('pad', 0.02), dpi=300)
    plt.close(fig); print('wrote', os.path.join(FIG, out))


def fig_relaxed(names=('D4_relaxed', 'D4_filter', 'D4_nominal'), out='fig_relaxed.pdf', eps_J=1.5e-3, th2max=1.3, t_sw=15.0, T=40.0):
    labels = ['proposed', 'cascade', 'nominal']; styles = ['-', '--', ':']; cols = [C_PROP, C_FILT, C_NOM]
    clip = [False, False, True]   # only the nominal run diverges, the cascade stays bounded and is drawn in full
    fig, ax = plt.subplots(3, 2, figsize=(3.45, 2.55), sharex=True)
    ax = ax.ravel(); marks = []
    for n, lab, s, c, cl in zip(names, labels, styles, cols, clip):
        try: L = load(n)
        except FileNotFoundError: continue
        t = L['t']; dc, de = path_errors(L)
        # the nominal run is cut where its state stops being meaningful (tilt beyond 60 deg or distance beyond 2 m)
        bad = np.where((np.degrees(np.abs(L['Phi'][:, :2]).max(axis=1)) > 60) | (np.maximum(dc, de) > 2.0))[0] if cl else []
        k = bad[0] if len(bad) else len(t)
        t, dc, de = t[:k], dc[:k], de[:k]
        if k < len(L['t']):   # the cut instant is marked by a cross
            marks.append((c, t[-1], (dc[-1] + 1e-13, de[-1] + 1e-13, L['eta_c'][k-1, 1], L['eta_e'][k-1, 1], L['th'][k-1, 1], 1e3 * L['detJ'][k-1])))
        ax[0].semilogy(t, dc + 1e-13, s, color=c, label=lab, lw=1.1)
        ax[1].semilogy(t, de + 1e-13, s, color=c, label=lab, lw=1.1)
        ax[2].plot(t, L['eta_c'][:k, 1], s, color=c, lw=1.1); ax[3].plot(t, L['eta_e'][:k, 1], s, color=c, lw=1.1)
        ax[4].plot(t, L['th'][:k, 1], s, color=c, lw=1.1); ax[5].plot(t, 1e3 * L['detJ'][:k], s, color=c, lw=1.1)
    ax[0].set_ylabel(r'$d_c$ [m]'); ax[1].set_ylabel(r'$d_e$ [m]')
    ax[2].set_ylabel(r'$\eta_2$ [rad/s]'); ax[3].set_ylabel(r'$\varsigma_2$ [rad/s]')
    ax[4].set_ylabel(r'$\vartheta_2$ [rad]'); ax[5].set_ylabel(r'$\det J_\rho$ [$10^{-3}$m$^3$]')
    ax[4].axhline(th2max, color='k', lw=0.6, ls='-.'); ax[4].axhline(-th2max, color='k', lw=0.6, ls='-.'); ax[5].axhline(1e3 * eps_J, color='k', lw=0.6, ls='-.'); ax[5].axhline(0, color='k', lw=0.4)
    ax[2].axhline(0.25, color='gray', lw=0.6, ls=':'); ax[3].plot([0, t_sw, t_sw, T], [0.25, 0.25, 0.42, 0.42], color='gray', lw=0.6, ls=':')
    for a in ax[:2]: a.set_ylim(1e-7, 50); a.set_yticks([1e-6, 1e-3, 1])
    ax[2].set_ylim(0.15, 0.30); ax[3].set_ylim(-0.3, 0.6); ax[4].set_ylim(-1.55, 1.75); ax[5].set_ylim(-1.5, 7)
    for a in ax: a.axvline(t_sw, color='0.55', lw=0.8, ls='-', alpha=0.8)
    for c, tm, ys in marks:
        for a, y in zip(ax, ys):
            lo, hi = a.get_ylim(); a.plot([tm], [min(max(y, lo), hi)], 'x', color=c, ms=3.5, mew=0.9)
    ax[4].set_xlabel('t [s]'); ax[5].set_xlabel('t [s]'); ax[4].set_xlim(0, T)
    handles, labels_ = ax[0].get_legend_handles_labels()
    fig.legend(handles, labels_, loc='upper center', ncol=3, frameon=False, handlelength=1.8, columnspacing=1.5, bbox_to_anchor=(0.55, 1.0), fontsize=8)
    fig.tight_layout(pad=0.3, h_pad=0.4, w_pad=0.6, rect=(0, 0, 1, 0.935))
    save(fig, out)


def fig_concept(render='D3_nominal_close_frame6000.png', out='concept.pdf'):
    """the render of step 6000 of the nominal run (render_drake.py D3_nominal --cam close --frame 6000), annotated"""
    import imageio
    img = imageio.imread(os.path.join(RES, 'anim', render))
    fig, ax = plt.subplots(figsize=(3.45, 1.95)); ax.imshow(img); ax.axis('off')
    kw = dict(fontsize=9, color='white', ha='center', va='center', bbox=dict(boxstyle='round,pad=0.15', fc='black', ec='none', alpha=0.55))
    ax.annotate(r'$\gamma_c$', xy=(310, 413), xytext=(200, 330), arrowprops=dict(arrowstyle='-', color='white', lw=0.8), **kw)
    ax.annotate(r'$\gamma_e$', xy=(318, 510), xytext=(420, 600), arrowprops=dict(arrowstyle='-', color='white', lw=0.8), **kw)
    ax.annotate(r'$p_c$', xy=(640, 291), xytext=(880, 230), arrowprops=dict(arrowstyle='-', color='white', lw=0.8), **kw)
    ax.annotate(r'$p_e$', xy=(683, 399), xytext=(900, 470), arrowprops=dict(arrowstyle='-', color='white', lw=0.8), **kw)
    save(fig, out, pad=0.0, dpi=300)


if __name__ == "__main__":
    which = sys.argv[1:] or ['concept', 'nominal', 'relaxed']
    if 'concept' in which: fig_concept()
    if 'nominal' in which: fig_nominal()
    if 'relaxed' in which: fig_relaxed()
