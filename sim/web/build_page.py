r"""Render docs/index.html from results/web/numbers.json and copy the figures to docs/figs.

Run page_numbers.py, figs_web.py and anim_web.py first. No result is written into the page by hand.
"""
import os, json, math, shutil

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.abspath(os.path.join(HERE, '..'))
SITE = os.path.abspath(os.path.join(SIM, '..', 'docs'))
N = json.load(open(os.path.join(SIM, 'results', 'web', 'numbers.json')))
REPO = "https://github.com/gradslab/am_dual_pf-"

FIGS = ['W_nominal', 'W_relaxed', 'W_interval', 'W_actuator', 'W_cascade', 'W_window', 'W_sampling', 'W_rate', 'W_mismatch', 'concept']


def f(x, spec=".2f", dash="n/a"):
    return dash if x is None else format(x, spec)


def lb(x, d=1, dash="n/a"):
    """lower bound rounded down, so a quoted minimum never exceeds the data"""
    return dash if x is None else format(math.floor(x * 10 ** d) / 10 ** d, f".{d}f")


def sig(x, n=2, dash="n/a"):
    """2.9e-05 becomes 2.9 x 10^-5, 0.0021 stays decimal"""
    if x is None:
        return dash
    if x == 0:
        return "0"
    e = math.floor(math.log10(abs(x)))
    if -3 <= e <= 4:
        return f"{x:,.{max(0, n - 1 - e)}f}".rstrip("0").rstrip(".")
    m = x / 10 ** e
    return f"{m:.1f}&times;10<sup>{e}</sup>"


def rng(v, spec=".3f", unit=""):
    return f"[{format(v[0], spec)}, {format(v[1], spec)}]{unit}"


def ratio_text(one):
    r = [one["residual_ratio_de"], one["residual_ratio_dc"], one["residual_ratio_barrier"]]
    lo, hi = f(min(r), ".1f"), f(max(r), ".1f")
    return lo if lo == hi else f"{lo} to {hi}"


SEC = [("gallery", "Animations"),
       ("nominal", "The nominal run"),
       ("relaxed", "Incompatible speed references"),
       ("comparison", "Against a cascade and the nominal controller"),
       ("feasibility", "Feasibility margins"),
       ("postures", "Postures compatible with both paths"),
       ("rate", "Controller rate"),
       ("mismatch", "Model mismatch"),
       ("parameters", "Parameters"),
       ("reproduce", "Reproducing this"),
       ("cite", "Citation")]


def rail():
    items = "\n".join(f'      <li><a href="#{i}">{t}</a></li>' for i, t in SEC)
    return f'''  <aside class="rail">
    <div class="rail-h">On this page</div>
    <ol>
{items}
    </ol>
  </aside>'''


def video(base, alt=""):
    return (f'<div class="plate"><video src="anim/{base}.mp4" poster="anim/{base}.jpg" '
            f'autoplay loop muted playsinline preload="metadata" aria-label="{alt}"></video></div>')


def card(pill, pill_cls, title, base, alt, text=""):
    t = f"<h3>{title}</h3>" if title else ""
    p = f"<p>{text}</p>" if text else ""
    return f'<div class="card media"><span class="pill {pill_cls}">{pill}</span>{t}{video(base, alt)}{p}</div>'


def figure(src, cap):
    return f'''      <figure class="figblock">
        <div class="plate"><img src="{src}" alt="" loading="lazy"></div>
        <figcaption class="cap">{cap}</figcaption>
      </figure>'''


def table(head, rows, note=None):
    th = "".join(f'<th class="num">{h}</th>' if i else f"<th>{h}</th>" for i, h in enumerate(head))
    body = ""
    for r in rows:
        if isinstance(r, str):
            body += f'<tr><td colspan="{len(head)}"><b>{r}</b></td></tr>'
        else:
            body += "<tr>" + "".join(f'<td class="num">{c}</td>' if i else f"<td>{c}</td>" for i, c in enumerate(r)) + "</tr>"
    out = f'<div class="tablewrap"><table><thead><tr>{th}</tr></thead><tbody>{body}</tbody></table></div>'
    if note:
        out += f'<p class="cap">{note}</p>'
    return out


def params_table(title, rows):
    body = "".join(f"<tr><td>{k}</td><td>{v}</td></tr>" for k, v in rows)
    return (f'<h3 style="margin:1.6rem 0 .7rem">{title}</h3>'
            f'<div class="tablewrap"><table><thead><tr><th>Parameter</th><th>Value</th></tr></thead><tbody>{body}</tbody></table></div>')


def build():
    M = N["metrics"]; P = N["params"]; S = N["study"]
    nom, pr, pk, ca, ns, bs = M["nominal"], M["proposed"], M["proposed_1khz"], M["cascade"], M["nominal_switch"], M["before_switch"]
    mm = M["mismatch"]; sg, sa, wi = S["singular"], S["sampling"], S["window"]
    one = M["one_khz"]
    g = P["gains"]; jl = P["joint_limits"]

    head = f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Dual Path Following for an Aerial Manipulator</title>
<meta name="description" content="Simulation study for dual path following of a quadrotor with a 3-DOF arm under state and input constraints via transverse feedback linearization.">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Newsreader:ital,opsz,wght@0,6..72,400;0,6..72,500;0,6..72,600;1,6..72,400&family=IBM+Plex+Sans:ital,wght@0,400;0,500;0,600;1,400&family=IBM+Plex+Mono:wght@400;500&display=swap" rel="stylesheet">
<link rel="stylesheet" href="assets/css/site.css">
<script>
window.MathJax = {{tex:{{inlineMath:[["\\\\(","\\\\)"]]}}, svg:{{fontCache:"global"}}}};
</script>
<script defer src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js"></script>
</head>
<body>

<header class="topbar">
  <span class="mark">Dual path following &middot; aerial manipulator</span>
  <span class="spacer"></span>
  <nav>
    <a href="#gallery">Animations</a>
    <a href="#relaxed">Incompatible speeds</a>
    <a href="#comparison">Comparison</a>
    <a href="#postures">Postures</a>
    <a href="#mismatch">Mismatch</a>
    <a href="#parameters">Parameters</a>
    <a href="#reproduce">Reproduce</a>
  </nav>
  <button id="theme" class="btn" type="button" aria-label="Switch colour theme">Theme</button>
</header>

<div class="shell">
{rail()}
  <main>

  <section class="hero">
    <div class="eyebrow">Supplementary simulation study</div>
    <h1>Dual Path Following for an Aerial Manipulator Under State and Input Constraints via Transverse Feedback Linearization</h1>
    <p class="byline">Hamza Tariq and Adeel Akhtar
      <span class="aff">GRaDS Lab, New Jersey Institute of Technology</span></p>
    <p class="lede">A quadrotor carries a 3-DOF arm. The center of mass of the combined system must follow one
      path and the end effector must follow a second path. The controller keeps both paths as hard constraints
      and gives up only the speed of the end effector along its path. This page carries the animations behind
      the figures of the paper, the complete controller settings, a run at 1&nbsp;kHz, a study of the postures
      compatible with both paths, a study of model mismatch, and everything needed to reproduce them.</p>
    <div class="buttons">
      <a class="btn primary" href="{REPO}">Code and data</a>
      <a class="btn" href="#comparison">Comparison</a>
      <a class="btn" href="#reproduce">Reproduce the figures</a>
    </div>
  </section>
'''

    gallery = f'''
  <section class="sec" id="gallery">
    <div class="sec-head">
      <span class="sec-tag">Animations</span>
      <h2>Incompatible speed references, resolved through the end-effector speed</h2>
      <p>Rendered in Drake from the logged run. A Skydio X2 carries an OpenMANIPULATOR-X arm mounted upside
      down below the airframe. The blue dots are the vehicle path of the center of mass and the red dots are
      the end-effector path. Both points start off their paths. At t&nbsp;=&nbsp;{f(P["t_switch"], ".0f")}&nbsp;s the
      end-effector speed reference is raised from {f(P["v_des"])} to {f(P["v_e_switch"])}&nbsp;rad/s while the vehicle
      speed reference is unchanged. The arm stretches until the singularity barrier becomes active at
      t&nbsp;=&nbsp;{f(pr["t_activation"])}&nbsp;s. From then on the end effector is slowed back to the vehicle speed and
      both points stay on their paths.</p>
    </div>
<div class="grid2">{card("side on", "neutral", "", "proposed_side", "proposed controller, side view")}{card("orbit", "neutral", "", "proposed_orbit", "proposed controller, orbiting camera")}</div>
    <div class="sec-head" style="margin-top:2.6rem">
      <h2>The nominal run</h2>
      <p>Equal speed references of {f(P["v_des"])}&nbsp;rad/s on both paths, from the same off-path start. No
      constraint becomes active, so the controller applies the transverse feedback linearizing input unchanged.</p>
    </div>
<div class="grid2">{card("orbit", "neutral", "", "nominal_orbit", "nominal run, orbiting camera")}{card("side on", "neutral", "", "nominal_side", "nominal run, side view")}</div>
    <div class="stats" style="margin-top:2.2rem">
      <div class="stat"><div class="n">{f(pr["peak_de_after_switch_mm"], ".1f")} mm</div>
        <div class="k">largest distance of the end effector to its path while the singularity row becomes active,
          against {f(ca["final_de_mm"] / 1e3, ".1f")}&nbsp;m for the safety-filtered cascade at the end of its run</div></div>
      <div class="stat"><div class="n">{f(pr["max_slack"])} rad/s<sup>2</sup></div>
        <div class="k">largest slack on the end-effector speed row, the only row that is ever relaxed</div></div>
      <div class="stat"><div class="n">{sig(pr["barrier_residual"], 1)} m<sup>3</sup></div>
        <div class="k">largest violation of the singularity margin \\(\\epsilon_J\\) over the run, the sampled-data residual of the barrier</div></div>
      <div class="stat"><div class="n">0</div>
        <div class="k">steps at which the reduced scalar program was infeasible, in every run with the exact model</div></div>
    </div>
  </section>
'''

    nominal = f'''
  <section class="sec" id="nominal">
    <div class="sec-head">
      <span class="sec-tag">Nominal run</span>
      <h2>The nominal run</h2>
      <p>The vehicle path is a horizontal circle of radius {f(P["r_path"], ".1f")}&nbsp;m with a wavy altitude
      \\(z = {f(P["z_c0"], ".1f")} + 0.1\\sin 2\\lambda\\), with \\(\\lambda\\) the polar angle. The end-effector path is the
      circle of the same radius below it with a finer scalloped altitude
      \\(z = {f(P["z_e0"])} + 0.06\\sin(2\\lambda + 0.8) + 0.03\\sin 8\\lambda\\). Speeds are rates of the polar angle, and
      {f(P["v_des"])}&nbsp;rad/s is {f(P["v_des"] * P["r_path"], ".1f")}&nbsp;m/s. The run starts {f(P["start"]["dr"], ".1f")}&nbsp;m outside and
      {f(-P["start"]["dz"])}&nbsp;m below the vehicle path with a heading error of {f(P["start"]["psi_err"])}&nbsp;rad. The
      distances \\(d_c\\) and \\(d_e\\) are measured to the points of the paths at the same polar angle, which bounds
      the distances to the paths from above.</p>
    </div>
{figure("figs/W_nominal.png", f"<b>Nominal dual path following from an off-path initial condition.</b> Left: center of mass (blue), end effector (red) and the two paths (wide light lines). The dots mark the start. Right: distances to the paths, the two along-path speeds and the heading error.")}
    <div class="prose">
      <p>From \\(d_c(0) = {f(nom["d_c0"])}\\)&nbsp;m and \\(d_e(0) = {f(nom["d_e0"])}\\)&nbsp;m both distances decay exponentially and fall
      below 1&nbsp;mm after {f(nom["t_below_1mm"], ".1f")}&nbsp;s. For the rest of the {f(nom["T"], ".0f")}&nbsp;s run, \\(d_c\\) stays below
      {f(nom["max_dc_after_mm"], ".3f")}&nbsp;mm and \\(d_e\\) below {f(nom["max_de_after_mm"], ".2f")}&nbsp;mm. These residuals come from the zero-order hold of
      the controller at {P["ctrl_hz"]}&nbsp;Hz, since in continuous time the hard rows keep the transverse coordinates at zero.
      The speeds settle in {rng(nom["eta_c_after_6s"])} and {rng(nom["eta_e_after_6s"])}&nbsp;rad/s after 6&nbsp;s,
      and the heading error stays below {sig(nom["heading_err_after_10s"], 1)}&nbsp;rad after 10&nbsp;s. The thrust stays within
      {rng(nom["thrust"], ".2f")}&nbsp;N and the tilt below {f(nom["max_tilt_deg"], ".1f")}&nbsp;degrees.</p>
    </div>
  </section>
'''

    relaxed = f'''
  <section class="sec" id="relaxed">
    <div class="sec-head">
      <span class="sec-tag">Incompatible speeds</span>
      <h2>Incompatible speed references</h2>
      <p>The same start and gains. At t&nbsp;=&nbsp;{f(P["t_switch"], ".0f")}&nbsp;s the end-effector speed reference is raised
      to {f(P["v_e_switch"])}&nbsp;rad/s. The arm cannot sustain a lead that grows without bound, so the nominal controller
      stretches it into a singular posture. The proposed controller keeps the four transverse rows, the vehicle speed row
      and the heading row as equalities and relaxes only the end-effector speed row by a scalar slack
      \\(\\delta\\). The reduced program is a scalar quadratic program over an interval, solved in closed form.</p>
    </div>
{figure("figs/W_relaxed.png", f"<b>Incompatible speed references.</b> Distances to the paths, along-path speeds with their references (gray dotted), second joint angle and arm singularity function with their limits and margin (black dash-dotted) for the proposed controller (solid), the cascade (dashed) and the nominal controller (dotted). The gray vertical line marks the switch at t = {f(P['t_switch'], '.0f')} s. The nominal run ends at the cross, where it diverges, and the speeds of the cascade leave the plotted range.")}
    <div class="prose">
      <p>Before the switch no row is active and \\(\\delta^\\star = 0\\), so the three controllers coincide. The applied
      inputs of the nominal controller and the proposed controller {"are identical to the last digit" if bs["nominal_vs_proposed"] == 0 else "differ by at most " + sig(bs["nominal_vs_proposed"], 1)}
      before the switch, and those of the cascade differ from them by at most {sig(bs["cascade_vs_proposed"], 1)}, which is the tolerance
      of its numerical solver.</p>
      <p>With the proposed controller the singularity row becomes active at t&nbsp;=&nbsp;{f(pr["t_activation"])}&nbsp;s and the end-effector
      speed is relaxed back to the vehicle speed. The largest slack is {f(pr["max_slack"])}&nbsp;rad/s<sup>2</sup>. From then on some
      row is active at every step, the singularity row at {f(100 * pr["share_detJ"], ".0f")}&nbsp;% of the steps and the upper limit row of
      \\(\\vartheta_2\\) at the rest, so the closed loop moves along the constraint boundary. The barrier holds up to a residual of
      {sig(pr["barrier_residual"], 1)}&nbsp;m<sup>3</sup> in \\(\\det J_\\rho\\), and after t&nbsp;=&nbsp;20&nbsp;s the value of \\(\\det J_\\rho\\) stays
      below {sig(pr["max_detJ_after_20s"], 3)}&nbsp;m<sup>3</sup>, close to \\(\\epsilon_J = {sig(P["eps_J"], 2)}\\)&nbsp;m<sup>3</sup>. The distance
      \\(d_e\\) peaks at {f(pr["peak_de_after_switch_mm"], ".1f")}&nbsp;mm during the activation and \\(d_c\\) at {f(pr["peak_dc_after_switch_mm"])}&nbsp;mm.
      After t&nbsp;=&nbsp;20&nbsp;s they stay below {f(pr["de_after_20s_mm"])}&nbsp;mm and {f(pr["dc_after_20s_mm"])}&nbsp;mm.</p>
      <p>After t&nbsp;=&nbsp;20&nbsp;s the end-effector speed has mean {f(pr["eta_e_mean_after_20s"], ".3f")}&nbsp;rad/s and ranges over
      {rng(pr["eta_e_range_after_20s"])}&nbsp;rad/s. It oscillates at about {f(pr["eta_e_osc_hz"])}&nbsp;Hz, close to the
      {f(pr["scallop_hz"])}&nbsp;Hz at which the eight scallops of the end-effector path pass, so the oscillation follows
      the path geometry and not the sampling. The vehicle speed ranges over {rng(pr["eta_c_range_after_switch"])}&nbsp;rad/s.
      Its dip is the sampled-data error of the fast arm motion. The heading error stays below
      {sig(pr["heading_err_after_10s"], 1)}&nbsp;rad.</p>
    </div>
{figure("figs/W_interval.png", f"<b>The feasible interval of the scalar program.</b> Rows with a positive coefficient on the slack bound it from below and rows with a negative coefficient bound it from above. The applied slack is the point of the interval closest to zero. After the activation the upper end is set by the row of \\(\\det J_\\rho\\) or by the upper limit of \\(\\vartheta_2\\), and the lower end by the fold rows of the arm. The interval is at least {lb(pr['int_width_before_switch'])} rad/s<sup>2</sup> wide before the switch and at least {lb(pr['int_width_after_switch'])} rad/s<sup>2</sup> wide afterwards.")}
{figure("figs/W_actuator.png", f"<b>Inputs of the proposed controller.</b> The thrust \\(u_t\\) stays within {rng(pr['thrust_after_switch'], '.2f')} N after the switch and within {rng(pr['thrust_whole_run'], '.2f')} N over the whole run. The largest body moment is {f(pr['max_moment'])} Nm and the largest joint torque is {f(pr['max_joint_torque'])} Nm, against the bounds of ({', '.join(f(v, '.0f') for v in P['tau_max'])}) Nm and {f(P['tauj_max'][0], '.0f')} Nm.")}
  </section>
'''

    rows_tab = [
        ("Singularity row first active [s]", f(pr["t_activation"]), f(pk["t_activation"])),
        ("Largest slack \\(|\\delta^\\star|\\) [rad/s<sup>2</sup>]", f(pr["max_slack"]), f(pk["max_slack"])),
        ("Steps with an active row after the activation", f"{100 * pr['steps_active_after_activation']:.0f} %", f"{100 * pk['steps_active_after_activation']:.0f} %"),
        ("Share of the singularity row among them", f"{100 * pr['share_detJ']:.0f} %", f"{100 * pk['share_detJ']:.0f} %"),
        ("Largest \\(\\epsilon_J - \\det J_\\rho\\) [m<sup>3</sup>]", sig(pr["barrier_residual"], 2), sig(pk["barrier_residual"], 2)),
        ("Peak \\(d_e\\) after the switch [mm]", f(pr["peak_de_after_switch_mm"]), f(pk["peak_de_after_switch_mm"])),
        ("Peak \\(d_c\\) after the switch [mm]", f(pr["peak_dc_after_switch_mm"]), f(pk["peak_dc_after_switch_mm"])),
        ("\\(d_e\\) after 20 s [mm]", "&lt; " + f(pr["de_after_20s_mm"]), "&lt; " + f(pk["de_after_20s_mm"])),
        ("\\(d_c\\) after 20 s [mm]", "&lt; " + f(pr["dc_after_20s_mm"]), "&lt; " + f(pk["dc_after_20s_mm"])),
        ("End-effector speed after 20 s, mean [rad/s]", f(pr["eta_e_mean_after_20s"], ".3f"), f(pk["eta_e_mean_after_20s"], ".3f")),
        ("End-effector speed after 20 s, range [rad/s]", rng(pr["eta_e_range_after_20s"]), rng(pk["eta_e_range_after_20s"])),
        ("Vehicle speed after the switch [rad/s]", rng(pr["eta_c_range_after_switch"]), rng(pk["eta_c_range_after_switch"])),
        ("Heading error after 10 s [rad]", "&lt; " + sig(pr["heading_err_after_10s"], 1), "&lt; " + sig(pk["heading_err_after_10s"], 1)),
        ("Thrust after the switch [N]", rng(pr["thrust_after_switch"], ".2f"), rng(pk["thrust_after_switch"], ".2f")),
        ("Largest body moment [Nm]", f(pr["max_moment"]), f(pk["max_moment"])),
        ("Largest joint torque [Nm]", f(pr["max_joint_torque"]), f(pk["max_joint_torque"])),
        ("Feasible interval width, before and after the switch [rad/s<sup>2</sup>]", f"&ge; {lb(pr['int_width_before_switch'])}, &ge; {lb(pr['int_width_after_switch'])}", f"&ge; {lb(pk['int_width_before_switch'])}, &ge; {lb(pk['int_width_after_switch'])}"),
        ("Steps with an infeasible program", "0" if pr["feasible_all"] else "some", "0" if pk["feasible_all"] else "some"),
    ]
    comparison = f'''
  <section class="sec" id="comparison">
    <div class="sec-head">
      <span class="sec-tag">Comparison</span>
      <h2>Against a cascade and the nominal controller</h2>
      <p>The cascade passes the nominal transverse feedback linearizing input through a safety filter with the same
      barrier and actuator rows. The filter minimizes the distance to the nominal input with an identity weight, so it may
      change every row of the linearizing equations, including the transverse rows. The nominal controller has no
      constraints at all. Gains, barrier poles, limits, initial condition and controller rate are identical across the three.</p>
    </div>
    <div class="grid3">
      {card("stays on both paths", "ok", "Proposed", "proposed_side", "proposed controller, side view", "The conflict is absorbed by the end-effector speed alone.")}
      {card("leaves both paths", "warn", "TFL + safety filter", "cascade_side", "safety-filtered cascade, side view", f"Nothing pins the transverse rows. The end effector leaves its path by 5 cm within {f(ca['t_ee_5cm_after_activation'])} s of the activation and the vehicle within {f(ca['t_vehicle_5cm_after_activation'])} s.")}
      {card("reaches the singular set", "bad", "Nominal TFL", "nominal_switch_side", "nominal controller, side view", f"The arm stretches until \\(\\det J_\\rho\\) crosses zero at t = {f(ns['t_detJ_zero'])} s. There the decoupling matrix is singular and the simulation diverges.")}
    </div>
{figure("figs/W_cascade.png", f"<b>What the cascade does to the vehicle.</b> Its vehicle speed reverses to {f(ca['min_vehicle_speed'], '.1f')} rad/s at t = {f(ca['t_min_vehicle_speed'], '.1f')} s, with the thrust at its upper bound of {f(P['f_max'], '.0f')} N and the body moments at their bounds. Neither point returns to within {f(ca['min_dc_after_leaving_mm'], '.1f')} mm of its path. The proposed controller is drawn for scale.")}
{table(["Proposed controller, incompatible-speed run", f"{P['ctrl_hz']} Hz", "1 kHz"], rows_tab, f"The 1 kHz run gives the same activation time within {f(one['activation_time_difference'])} s, the same largest slack, and residuals smaller by a factor of {ratio_text(one)}. The 1 kHz run is <a href='#rate'>below</a>.")}
    <div style="height:1.4rem"></div>
{table(["Safety-filtered cascade", "Value"], [
        ("End effector more than 5 cm off its path, after the activation", f"{f(ca['t_ee_5cm_after_activation'])} s"),
        ("Vehicle more than 5 cm off its path, after the activation", f"{f(ca['t_vehicle_5cm_after_activation'])} s"),
        ("Vehicle speed reversal", f"{f(ca['min_vehicle_speed'], '.1f')} rad/s at t = {f(ca['t_min_vehicle_speed'], '.1f')} s"),
        ("Largest thrust", f"{f(ca['max_thrust'], '.1f')} N (bound {f(P['f_max'], '.0f')} N)"),
        ("Largest body moments", f"({', '.join(f(v) for v in ca['max_moment'])}) Nm (bounds {', '.join(f(v, '.0f') for v in P['tau_max'])} Nm)"),
        ("Largest joint torque", f"{f(ca['max_joint_torque'])} Nm"),
        ("Closest return of the center of mass to its path after leaving", f"{f(ca['min_dc_after_leaving_mm'], '.1f')} mm"),
        ("Closest return of the end effector to its path after leaving", f"{f(ca['min_de_after_leaving_mm'], '.1f')} mm"),
        ("Joint limits exceeded by up to", f"{sig(ca['joint_violation'], 2)} rad"),
        "Nominal controller",
        ("\\(\\det J_\\rho\\) crosses zero at", f"{f(ns['t_detJ_zero'])} s"),
        ("\\(\\vartheta_3\\) at the first sample after the crossing", f"{f(ns['th3_after_crossing'])} rad (elbow singularity at {f(sg['th3_star'])} rad)"),
        ("Run ends at", f"{f(ns['T'])} s"),
    ])}
  </section>
'''

    feasibility = f'''
  <section class="sec" id="feasibility">
    <div class="sec-head">
      <span class="sec-tag">Feasibility</span>
      <h2>Feasibility margins</h2>
      <p>The reduced program is feasible exactly when every row with a zero coefficient on the slack holds at the
      nominal input and the largest lower bound does not exceed the smallest upper bound. One pass through the
      25 rows gives this test and the interval. The six vehicle rows, four attitude rows and two thrust rows, have a
      zero coefficient on the slack, because the slack acts through the arm only.</p>
    </div>
    <div class="stats">
      <div class="stat"><div class="n">&ge; {lb(pr["int_width_before_switch"], 0)} rad/s<sup>2</sup></div>
        <div class="k">width of the feasible interval before the switch</div></div>
      <div class="stat"><div class="n">&ge; {lb(pr["int_width_after_switch"])} rad/s<sup>2</sup></div>
        <div class="k">width after the switch, {f(pr["int_width_over_slack"], ".1f")} times the largest slack, at its narrowest at t = {f(pr["t_int_width_min"], ".1f")} s</div></div>
      <div class="stat"><div class="n">&ge; {f(pr["attitude_row_margin"], ".0f")} rad/s<sup>2</sup></div>
        <div class="k">margin of the four attitude rows at the nominal input, over the run</div></div>
      <div class="stat"><div class="n">&ge; {f(pr["thrust_row_margin"], ".0f")} N/s<sup>2</sup></div>
        <div class="k">margin of the two thrust rows at the nominal input, over the run</div></div>
    </div>
    <div class="prose">
      <p>The attitude margin is the smallest value of \\(-\\bar b_i\\) over the six vehicle rows logged at every step. The
      thrust margin is rebuilt from the logged thrust, its rate and the applied thrust input, since the slack leaves
      the thrust input unchanged. On the nominal motion of the posture study below, the same margins are
      {f(sa["vehicle_row_margin_attitude"], ".0f")}&nbsp;rad/s<sup>2</sup> and {f(sa["vehicle_row_margin_thrust"], ".0f")}&nbsp;N/s<sup>2</sup>,
      and the coefficient of the slack in the vehicle rows is at most {sig(sa["max_zero_coefficient_vehicle_rows"], 1)}.</p>
    </div>
  </section>
'''

    postures = f'''
  <section class="sec" id="postures">
    <div class="sec-head">
      <span class="sec-tag">Postures</span>
      <h2>Postures compatible with both paths</h2>
      <p>On the dual path-following manifold the posture of the arm is a function of the vehicle path coordinate
      \\(a_c\\) and of the lead \\(a_e - a_c\\) of the end effector along the circle. The arm reaches only a bounded set
      of end-effector positions relative to the center of mass, so only an interval of leads is admissible at each
      \\(a_c\\). Two studies map this interval and test strict feasibility on it.</p>
    </div>
{figure("figs/W_window.png", f"<b>Admissible lead window.</b> At {wi['positions']} positions on the vehicle path the lead is traced on a {f(wi['grid_step'], '.4f')} rad grid along the inverse-kinematics branch of the runs. A lead is admissible when the joints are inside their limits and \\(\\det J_\\rho \\ge \\epsilon_J\\). The window is {f(wi['width_range'][0])} to {f(wi['width_range'][1])} rad wide, {f(wi['arc_length_cm'][0], '.0f')} to {f(wi['arc_length_cm'][1], '.0f')} cm of arc. Its lower end, {f(wi['lower_end_range'][0])} to {f(wi['lower_end_range'][1])} rad, is set by a joint limit at every position. Its upper end, {f(wi['upper_end_range'][0])} to {f(wi['upper_end_range'][1])} rad, is set by \\(\\det J_\\rho = \\epsilon_J\\) at {wi['upper_end_reason'].get('det J below eps_J', 0)} positions and by the upper limit of \\(\\vartheta_2\\) at {wi['upper_end_reason'].get('joint limit', 0)}. After the switch the run rides the upper end: its lead minus the upper end lies in [{f(wi['run_lead_minus_upper_end_after_20s'][0], '.4f')}, {f(wi['run_lead_minus_upper_end_after_20s'][1], '.4f')}] rad after 20 s, within one grid step.")}
{figure("figs/W_sampling.png", f"<b>Feasibility sampling.</b> States on the manifold with every transverse coordinate, speed error and heading error equal to zero (residual at most {sig(sa['max_z_residual'], 1)}), at {sa['positions']} positions, {sa['leads']} leads from {f(sa['lead_range'][0])} to {f(sa['lead_range'][1])} rad and {len(sa['speeds'])} end-effector speeds, {sa['n_states']} states in all. Of the {sa['n_in_admissible_set']} states in the admissible set of every barrier, {sa['n_not_strictly_feasible']} are not strictly feasible: {sa['per_speed']['-0.5']['not_strictly_feasible']} at an end-effector speed of &minus;0.5 rad/s, {sa['per_speed']['1.0']['not_strictly_feasible']} at 1.0 rad/s and {sa['per_speed']['1.5']['not_strictly_feasible']} at 1.5 rad/s, and none at 0, 0.25, 0.42 and 0.6 rad/s. The speed reference does not enter this test, because it shifts the nominal input along the slack direction and moves the interval without changing its width.")}
{table(["Singular postures of the arm (from the multibody model)", "Value"], [
        ("Elbow angle \\(\\vartheta_3^*\\) with \\(\\det J_\\rho = 0\\) for every \\(\\vartheta_2\\)", f"{f(sg['th3_star'])} rad (spread over three values of \\(\\vartheta_2\\): {sig(sg['th3_star_spread'], 1)})"),
        ("Distance of the elbow limit {} rad to this singularity".format(f(jl[2][0], ".1f")), f"{f(sg['elbow_limit_to_singularity'])} rad"),
        ("Shoulder angle with \\(\\bar\\rho_x = 0\\) at \\(\\vartheta_3 = 0.1\\) rad", f"{f(sg['th2_rho_x_zero_at_th3_0p1'])} rad, inside the joint box"),
        ("Airframe mass and mass of the combined system", f"{f(sg['airframe_mass'], '.3f')} kg and {f(sg['total_mass'], '.3f')} kg"),
        ("Offset of the center of mass from the body origin, incompatible-speed run", f"{f(pr['com_offset_cm'][0], '.1f')} to {f(pr['com_offset_cm'][1], '.1f')} cm"),
    ])}
  </section>
'''

    rate = f'''
  <section class="sec" id="rate">
    <div class="sec-head">
      <span class="sec-tag">Rate</span>
      <h2>Controller rate</h2>
      <p>The controller runs at {P["ctrl_hz"]}&nbsp;Hz with a zero-order hold on the body-frame command, and the plant is
      integrated at {P["plant_hz"] // 1000}&nbsp;kHz with a third-order Runge&ndash;Kutta scheme. The runs are therefore sampled
      approximations of the continuous-time controller. The same incompatible-speed run at 1&nbsp;kHz shows what the
      sampling costs.</p>
    </div>
{figure("figs/W_rate.png", f"<b>400 Hz against 1 kHz.</b> Distances to the paths, the applied slack and the violation of the singularity margin. The activation time moves by {f(one['activation_time_difference'])} s, the largest slack {"is unchanged at " + f(pk['max_slack']) if f(pk['max_slack']) == f(pr['max_slack']) else "is " + f(pk['max_slack']) + " against " + f(pr['max_slack'])} rad/s<sup>2</sup>, and the peak distances and the barrier residual shrink by a factor of {ratio_text(one)}. The vehicle speed stays within {rng(pk['eta_c_range_after_switch'])} rad/s at 1 kHz.")}
  </section>
'''

    a102, v105, a105, v110 = mm["a102"], mm["v105"], mm["a105"], mm["v110"]
    mismatch = f'''
  <section class="sec" id="mismatch">
    <div class="sec-head">
      <span class="sec-tag">Mismatch</span>
      <h2>Model mismatch</h2>
      <p>The controller computes the drift of every row from the model, so a model error moves the closed loop off the
      paths. In these runs the masses and inertias of the simulated plant are scaled while the controller keeps the
      nominal model. The incompatible-speed scenario is unchanged.</p>
    </div>
{figure("figs/W_mismatch.png", "<b>Model mismatch.</b> Distances of the center of mass and of the end effector to their paths and the shoulder angle with its limits, for the exact model, the arm masses and inertias 2 % above their model values, and the airframe mass 5 % above its model value. Bottom right: the two cases that diverge, the arm 5 % and the airframe 10 % above their model values.")}
{table(["Plant scaled against the model", "Arm +2 %", "Airframe +5 %", "Arm +5 %", "Airframe +10 %"], [
        ("Run length [s]", f(a102["T"], ".1f"), f(v105["T"], ".1f"), f(a105["T"], ".1f") + " (diverges)", f(v110["T"], ".1f") + " (diverges)"),
        ("Program feasible at every step", "yes" if a102["feasible_all"] else "no", "yes" if v105["feasible_all"] else "no", "yes" if a105["feasible_all"] else "no", "yes" if v110["feasible_all"] else "no"),
        ("First infeasible step [s]", f(a102["t_first_infeasible"]), f(v105["t_first_infeasible"]), f(a105["t_first_infeasible"]), f(v110["t_first_infeasible"])),
        ("Share of infeasible steps after the switch", f"{100 * a102['infeasible_share_after_switch']:.0f} %", f"{100 * v105['infeasible_share_after_switch']:.0f} %", "n/a", "n/a"),
        ("Largest \\(\\epsilon_J - \\det J_\\rho\\) [m<sup>3</sup>]", sig(a102["barrier_residual"], 1), sig(v105["barrier_residual"], 1), sig(a105["barrier_residual"], 1), sig(v110["barrier_residual"], 1)),
        ("Joint limits exceeded by up to [rad]", sig(a102["joint_violation"], 2), f(v105["joint_violation"]), f(a105["joint_violation"], ".1f"), "diverges"),
        ("\\(d_c\\) after 20 s [cm]", rng(a102["dc_after_20s_cm"], ".1f"), rng(v105["dc_after_20s_cm"], ".1f"), "n/a", "n/a"),
        ("\\(d_e\\) after 20 s [cm]", rng(a102["de_after_20s_cm"], ".1f"), rng(v105["de_after_20s_cm"], ".2f"), "n/a", "n/a"),
        ("\\(d_e\\) between 10 and 15 s [cm]", rng(a102["de_10_to_15s_cm"], ".1f"), rng(v105["de_10_to_15s_cm"], ".2f"), "n/a", "n/a"),
        ("Peak \\(d_e\\) after the switch [cm]", f(a102["peak_de_after_switch_cm"], ".1f"), f(v105["peak_de_after_switch_cm"], ".1f"), "n/a", "n/a"),
        ("Largest vehicle speed error after 10 s [rad/s]", f(a102["max_speed_err_after_10s"]), f(v105["max_speed_err_after_10s"]), "n/a", "n/a"),
    ], f"With the arm 2 % heavier the run stays feasible and the barriers hold, but the center of mass settles {f(a102['dc_after_20s_cm'][0], '.0f')} cm off its path. The end effector settles {f(a102['de_10_to_15s_cm'][0], '.0f')} to {f(a102['de_10_to_15s_cm'][1], '.0f')} cm off its path before the switch and {f(a102['de_after_20s_cm'][0], '.0f')} to {f(a102['de_after_20s_cm'][1], '.0f')} cm after 20 s, with a peak of {f(a102['peak_de_after_switch_cm'], '.1f')} cm after the switch. With the airframe 5 % heavier the program becomes infeasible after the switch, the fallback takes over and the joint limits are exceeded by up to {f(v105['joint_violation'])} rad. The two larger errors make the run diverge within the initial transient, before the switch. Hardware will therefore need integral action or an estimate of the acceleration of the center of mass.")}
  </section>
'''

    PARAMS = [
        ("Plant and scenario", [
            ("Platform", "Skydio X2 airframe (MuJoCo Menagerie inertial data) with an OpenMANIPULATOR-X arm (ROBOTIS description file), wrist fixed, mounted upside down below the airframe"),
            ("Masses", f"airframe {f(sg['airframe_mass'], '.3f')} kg, combined system {f(sg['total_mass'], '.3f')} kg"),
            ("Engine", f"Drake MultibodyPlant, continuous, third-order Runge&ndash;Kutta at {P['plant_hz'] // 1000} kHz (fixed step 0.25 ms)"),
            ("Controller rate", f"{P['ctrl_hz']} Hz, zero-order hold on the body-frame command, thrust chain integrated exactly between updates"),
            ("Gravity", "9.81 m/s<sup>2</sup>"),
            ("Vehicle path", f"x<sup>2</sup> + y<sup>2</sup> = {f(P['r_path'], '.1f')}<sup>2</sup>, z = {f(P['z_c0'], '.1f')} + 0.1 sin 2&lambda;, path coordinate &lambda; = atan2(y, x)"),
            ("End-effector path", f"same circle, z = {f(P['z_e0'])} + 0.06 sin(2&lambda; + 0.8) + 0.03 sin 8&lambda;"),
            ("Heading reference", "&psi;<sub>des</sub> = &lambda; + &pi;/2, the tangent direction, with the error wrapped to (&minus;&pi;, &pi;]"),
            ("Speed references", f"v<sub>c</sub> = v<sub>e</sub> = {f(P['v_des'])} rad/s ({f(P['v_des'] * P['r_path'], '.1f')} m/s). Incompatible-speed run: v<sub>e</sub> = {f(P['v_e_switch'])} rad/s from t = {f(P['t_switch'], '.0f')} s"),
            ("Start", f"center of mass {f(P['start']['dr'], '.1f')} m outside and {f(-P['start']['dz'])} m below the vehicle path at &lambda; = 0, moving at {f(P['v_des'] * P['r_path'], '.1f')} m/s along the circle, heading error {f(P['start']['psi_err'])} rad, level attitude, hover thrust, end effector at its on-path lead of {f(P['lead0'], '.3f')} rad plus a 5 cm radial and 4 cm vertical perturbation"),
            ("Horizon", f"{f(nom['T'], '.0f')} s (nominal run), {f(pr['T'], '.0f')} s (incompatible-speed runs)"),
        ]),
        ("Proposed controller", [
            ("Transverse gains k<sub>&xi;</sub> = k<sub>&zeta;</sub>", f"({', '.join(f(v, '.0f') for v in g['k_xi'])}), quadruple pole at &minus;{f(P['poles']['lam4'], '.0f')}"),
            ("End-effector transverse gains k<sub>&chi;</sub> = k<sub>&sigma;</sub>", f"({', '.join(f(v, '.2f') for v in g['k_chi'])}), double pole at &minus;{f(P['poles']['lam2'], '.1f')}"),
            ("Vehicle speed gains k<sub>&eta;</sub>", f"({', '.join(f(v, '.0f') for v in g['k_eta'])}), triple pole at &minus;{f(P['poles']['lam_eta'], '.0f')}"),
            ("End-effector speed gain k<sub>&varsigma;</sub>", f"{f(g['k_varsigma'], '.0f')}, pole at &minus;{f(P['poles']['k_be'], '.0f')}"),
            ("Heading gains k<sub>&mu;</sub>", f"({', '.join(f(v, '.0f') for v in g['k_mu'])}), double pole at &minus;{f(P['poles']['lam_psi'], '.0f')}"),
            ("ECBF gains (k<sub>1</sub>, k<sub>2</sub>)", f"({f(g['k_cbf'][0], '.0f')}, {f(g['k_cbf'][1], '.0f')}), double pole at &minus;{f(P['cbf_pole'], '.0f')} for every barrier"),
            ("Joint limits [rad]", f"&vartheta;<sub>1</sub> &isin; [{f(jl[0][0], '.1f')}, {f(jl[0][1], '.1f')}], &vartheta;<sub>2</sub> &isin; [{f(jl[1][0], '.1f')}, {f(jl[1][1], '.1f')}], &vartheta;<sub>3</sub> &isin; [{f(jl[2][0], '.1f')}, {f(jl[2][1], '.1f')}] (the simulated plant does not enforce them)"),
            ("Attitude margin", f"|&phi;|, |&theta;| &le; &pi;/2 &minus; {f(P['eps_att'], '.1f')} rad"),
            ("Thrust bounds", f"{f(P['f_min'], '.0f')} N &le; u<sub>t</sub> &le; {f(P['f_max'], '.0f')} N"),
            ("Actuator bounds", f"|&tau;| &le; ({', '.join(f(v, '.0f') for v in P['tau_max'])}) Nm, |&tau;<sub>&vartheta;</sub>| &le; {f(P['tauj_max'][0], '.0f')} Nm, as box rows on the input"),
            ("Singularity margin", f"det J<sub>&rho;</sub> &ge; &epsilon;<sub>J</sub> = {sig(P['eps_J'], 2)} m<sup>3</sup>"),
            ("Rows", "25: six joint, four attitude, two thrust and one singularity ECBF rows, twelve actuator box rows"),
            ("Objective and solve", "&frac12;&delta;<sup>2</sup> over the scalar slack of the end-effector speed row, interval test and closed-form clipping, no numerical solver in the loop"),
            ("Fallback", "if the interval is empty, the nominal input with both speed references set to zero (never reached with the exact model)"),
            ("Model knowledge", "det J<sub>&rho;</sub>, inertia matrix, bias forces and Jacobians from the multibody model, exact"),
        ]),
        ("TFL with a safety filter (cascade)", [
            ("Nominal feedback", "the same transverse feedback linearizing input"),
            ("Filter", "min &frac12;&#8214;u &minus; u<sub>TFL</sub>&#8214;<sup>2</sup> subject to the same 25 rows, identity weight"),
            ("Solve", "CLARABEL through cvxpy"),
            ("Difference from the proposed controller", "every row of D(x)u = v(x) may change, so the transverse rows are not pinned"),
        ]),
        ("Nominal controller", [
            ("Feedback", "u = D(x)<sup>&minus;1</sup> v(x), no constraints"),
        ]),
    ]
    params = "".join(params_table(t, r) for t, r in PARAMS)
    parameters = f'''
  <section class="sec" id="parameters">
    <div class="sec-head">
      <span class="sec-tag">Parameters</span>
      <h2>Every parameter, for all three controllers</h2>
      <p>The comparison is only worth reading if the three controllers were given the same problem.
      These are the complete settings behind every figure on this page.</p>
    </div>
    {params}
  </section>
'''

    reproduce = f'''
  <section class="sec" id="reproduce">
    <div class="sec-head">
      <span class="sec-tag">Reproduce</span>
      <h2>Reproducing this</h2>
      <p>The simulation code is in the repository under <code>sim/</code>. The verification gate runs first. It checks
      the symbolic model against energy conservation, the center-of-mass identity and a Drake cross-check, and the
      closed form of the decoupling matrix determinant against direct evaluation at random states.</p>
    </div>
<pre><code><span class="c"># set up and verify</span>
git clone {REPO}.git
cd am_dual_pf/sim
pip install -r requirements.txt
python3 model/verify_model.py
python3 model/test_relative_degree.py

<span class="c"># the runs behind every number and figure</span>
python3 drake/sim_drake.py D3
python3 drake/sim_drake.py D4 --modes relaxed,filter,nominal
python3 drake/sim_drake.py D4 --modes relaxed --hz 1000 --tag _1khz
python3 drake/sim_drake.py D5 --arm-scale 1.02 --tag _a102
python3 drake/sim_drake.py D5 --veh-scale 1.05 --tag _v105
python3 drake/sim_drake.py D5 --arm-scale 1.05 --T 12 --tag _a105
python3 drake/sim_drake.py D5 --veh-scale 1.10 --T 12 --tag _v110

<span class="c"># the posture study, the numbers and the figures</span>
python3 studies/posture_study.py
python3 metrics.py
python3 paper_figures.py
python3 web/figs_web.py

<span class="c"># animations (Drake VTK renderer and ffmpeg), then the page</span>
for s in "D4_relaxed side" "D4_relaxed orbit" "D3_nominal orbit" "D3_nominal side" \\
         "D4_filter side" "D4_filter orbit" "D4_nominal side"; do
  set -- $s; python3 drake/render_drake.py $1 --cam $2
done
python3 web/anim_web.py
python3 web/page_numbers.py
python3 web/build_page.py</code></pre>
  </section>

  <section class="sec" id="cite">
    <div class="sec-head">
      <span class="sec-tag">Citation</span>
      <h2>Citation</h2>
    </div>
<pre><code>@unpublished{{tariq2026dual,
  author = {{Hamza Tariq and Adeel Akhtar}},
  title  = {{Dual Path Following for an Aerial Manipulator Under State and
            Input Constraints via Transverse Feedback Linearization}},
  note   = {{Under review}},
  year   = {{2026}}
}}</code></pre>
  </section>

  <footer>
    <div class="rulebar"></div>
    <p>GRaDS Lab, New Jersey Institute of Technology.
      Every figure and number on this page is generated by the scripts above.
      <a href="{REPO}">Repository</a>.</p>
  </footer>

  </main>
</div>
<script src="assets/js/site.js"></script>
</body>
</html>
'''
    return head + gallery + nominal + relaxed + comparison + feasibility + postures + rate + mismatch + parameters + reproduce


def write():
    doc = build()
    os.makedirs(os.path.join(SITE, 'figs'), exist_ok=True)
    manifest = {"figs": [], "anim": N.get("anim_mb", {})}
    for name in FIGS:
        src = os.path.join(SIM, 'results', 'figs', name + '.png')
        if os.path.exists(src):
            shutil.copy(src, os.path.join(SITE, 'figs', name + '.png'))
            manifest["figs"].append({"file": f"figs/{name}.png", "MB": round(os.path.getsize(src) / 1e6, 2)})
    out = os.path.join(SITE, 'index.html')
    with open(out, 'w') as fh:
        fh.write(doc)
    json.dump(manifest, open(os.path.join(SITE, 'manifest.json'), 'w'), indent=1)
    print(f"wrote {out}  ({len(doc):,} characters)")


if __name__ == "__main__":
    write()
