# WORKLOG

Repository for the ECC paper "Dual Path Following for an Aerial Manipulator Under State and Input
Constraints via Transverse Feedback Linearization" (Hamza Tariq, Adeel Akhtar, GRaDS Lab, NJIT).

Repo path: /Users/hamzamalik/Downloads/Aerial Manipulation TFL/am_dual_pf
Reference repo (imitated): /Users/hamzamalik/Downloads/Aerial Manipulation TFL/safe_multiquad_pf-main
Source material (read only): /Users/hamzamalik/Downloads/Aerial Manipulation TFL/AM_TFL_QP
Page URL: http://localhost:8765/  (python3 -m http.server 8765 --directory .../am_dual_pf/docs, started in background,
log in the scratchpad server.log; restart with the same command if it is not answering)
Scratch logs of the runs: /private/tmp/claude-501/-Users-hamzamalik-Downloads-Aerial-Manipulation-TFL/769c6ca8-a54c-4a22-b296-146b095749d3/scratchpad/runs/

## Status
- [x] git init, WORKLOG
- [x] Read reference repo (Readme, sim/README, reproduce.sh, build_page.py, index.html, site.css, site.js)
- [x] Read paper main.tex, all sim scripts, notes (math_verification, reviewer_report_round10, feasibility_study)
- [x] Copy and clean: sim/model (am_controller.py, params.py, derive_model_fast.py, derive_extra.py, am_model_gen.py,
      am_extra_gen.py, make_urdf.py, verify_model.py, test_relative_degree.py), sim/drake (make_realistic_urdf.py,
      drake_model.py, sim_drake.py, render_drake.py), assets (X2 mesh+texture+mtl+LICENSE+README, OMX meshes_obj+LICENSE)
- [x] Verification gate passes on the cleaned copy (verify_model.py ALL PASS, test_relative_degree.py ALL PASS)
- [x] Cleaned controller checked against the original on 40 random states in all three modes: identical outputs
- [x] D3_nominal regenerated with the cleaned scripts: identical to the original log to 3e-12 (20 s wall time, no MeshCat)
- [x] Server running on 8765
- [x] ALL RUNS REGENERATED with the cleaned scripts (about 6 min in total, no MeshCat): D3_nominal, D4_relaxed, D4_filter,
      D4_nominal, D4_relaxed_1khz, D5_mismatch_a102/v105/a105/v110. Every regenerated log equals the copied original
      (max difference 5.9e-7 on D4_relaxed, 0 on the others, identical active-row lists). The regenerated logs replaced
      the copies in sim/results (the copies are gone; the originals remain in AM_TFL_QP). Nothing is "copied" any more
      except the three mp4 renders in sim/results/anim, which are being re-rendered too (scratchpad/runs/render.log).
- [x] sim/metrics.py: every Section VI number from the logs (python3 metrics.py prints them). All agree with the paper,
      see Discrepancies for the only wrinkles.
- [x] sim/studies/posture_study.py (10 s): sampling 3248 states, z residual 2.7e-12, 1430 in the admissible set,
      142 not strictly feasible (92 at -0.5, 34 at 1.0, 16 at 1.5 rad/s, none at 0, 0.25, 0.42, 0.6), vehicle-row
      margins 146.3 rad/s^2 and 1675.9 N/s^2 on the nominal motion; window 144 positions, widths 0.1275 to 0.165 rad,
      lower end 0.0925 to 0.1475 (joint limit at all 144), upper end 0.2325 to 0.2975; th3* = -1.382 rad,
      rho_x = 0 at th2 = -1.266 rad (th3 = 0.1); masses 1.325 and 1.917 kg.
- [x] Renders done (about 50 s each): D4_relaxed side/orbit, D3_nominal orbit/side, D4_filter side/orbit, D4_nominal side
      -> sim/results/anim/<log>_<cam>_drake.mp4, plus D3_nominal_close_frame6000.png for the concept figure.
      The three mp4 copied from AM_TFL_QP were overwritten by the re-renders, so nothing on the page is copied.
- [x] sim/web/anim_web.py: docs/anim/*.mp4 (960 px, H.264, 0.4 to 5.6 MB), poster jpg, two README gifs
- [x] sim/paper_figures.py (concept, fig_nominal, fig_relaxed -> results/figs, PDF + PNG), sim/web/figs_web.py (nine W_*.png)
- [x] sim/web/page_numbers.py -> results/web/numbers.json; sim/web/build_page.py -> docs/index.html + manifest.json
- [x] FIRST PAGE BUILT at 14:08 and served at http://localhost:8765/ (the coordinator relayed that the user was watching the
      directory listing before that; from now on the page is rebuilt after every milestone so it always renders)
- [ ] README.md, sim/README.md, reproduce.sh, requirements.txt
- [ ] Quality loop on the page

## Decisions
- Results copied from AM_TFL_QP/sim/results into sim/results as the baseline (D3_nominal, D4_relaxed, D4_filter, D4_nominal,
  D4_relaxed_1khz, D5_mismatch_a102/v105/a105/v110 npz + _active.json; mp4 renders D3_nominal_orbit, D4_relaxed_side/orbit
  in sim/results/anim). Regeneration runs with tag _regen; once equal (or accepted), they replace the copies.
- MuJoCo removed entirely. make_realistic_urdf.py embeds the X2 inertial constants (mass 1.325 kg, CoM, principal inertia,
  principal-frame rpy) that the original read from the Menagerie MJCF with MuJoCo; the generated URDF is byte-identical
  to the original apart from the absolute mesh paths. x2.xml and scene.xml are therefore not copied.
- The generated URDF (absolute mesh paths) is gitignored and rebuilt by every run.
- am_controller.py: the two-slack enumeration (solve_qp_2d), the Chebyshev radius LP and the generic cvxpy solver were
  dead code for the paper's scalar QP and were removed. The log schema is unchanged (cheb_r stays NaN).
- sim_drake.py: far_start moved in from the MuJoCo scenario file; env-variable options replaced by argparse flags;
  MeshCat recording is off unless --record; the D1 sanity scenario (not in the paper) was dropped.
- render_drake.py: the visual test modes (X2VIS, LIGHTS, SHADING env vars) were dropped; argparse flags; output to results/anim.

## Discrepancies (paper number vs script output)
- No disagreement. Every paper number is reproduced by metrics.py and posture_study.py to the quoted digits. The paper
  quotes some values as rounded bounds, and the scripts give: interval width after the switch 2.676 (paper "at least 2.6"),
  attitude-row margin 141.63 (paper "at least 140"), thrust-row margin 1652 (paper "at least 1600"), cascade closest return
  7.42 mm (paper "never within 7 mm"), a102 end effector 8.1 to 10.8 cm after 10 s and 8.1 to 9.2 cm after 20 s (paper "8 to
  11 cm"), nominal run d_e 0.9986 mm after 5.77 s (paper "below 1 mm"), peak d_e 2.759 mm (paper 2.8), 1 kHz activation
  16.239 s (paper "within 0.02 s" of 16.2575: difference 0.0185 s). The page shows the script values.
- None of substance beyond that. Notes: (1) the end-effector speed oscillation "about 0.34 Hz": a plain FFT over the 20 s
  window (0.05 Hz bins) peaks at 0.30 Hz, a Hann-windowed zero-padded FFT peaks at 0.343 Hz; metrics.py reports the
  latter together with the 0.05 Hz resolution. (2) The thrust-row margin (>= 1600 N/s^2) is not in the log; metrics.py
  rebuilds it from the logged thrust, its numerical derivative and the applied u_d (1652 N/s^2), and the posture study
  gives 1676 N/s^2 on the nominal motion. (3) The window study's upper-end reasons differ from the authors' lost
  window.py (which never recorded the theta_2 limit); ours names the joint limit, det J, or branch end per position
  with a fine continuation. Paper only says det J = eps_J sets the upper end at grid resolution.

- [x] README.md, sim/README.md, sim/reproduce.sh, sim/requirements.txt written
- [x] Quality loop round 1 (14:10-14:20): page screenshotted with headless Chrome (dark theme, 1400 px, 18.4k px tall),
      every section viewed. Fixed: MathJax escape in the nominal-TFL card, "differ by at most 0" wording (now "identical to
      the last digit" when the regenerated logs coincide), cascade poster frame at 17 s (at 20 s the vehicle had left the
      field of view), legend overlaps in W_interval, W_sampling and W_mismatch. Link check: all 23 local references resolve.
      Naming sweep (tool, three-joint, thrust extension, em dash, semicolons in prose): clean. The light theme could not be
      forced in headless Chrome (site.js resets data-theme); the CSS is the reference's, untouched.
- [x] sim/gif_from_mp4.py dropped (anim_web.py makes the gifs with ffmpeg)

- [x] Quality loop round 2 (14:25): page text dumped and read in full, repetitive ratio wording fixed ("2.5 to 2.5"),
      mismatch sentence now gives the before-switch, after-20 s and peak values of d_e, link check clean, server 200.
- [x] Final commit 87e57ba. JOB COMPLETE as of 14:28. Server still running on 8765.

## Next steps (if resumed)
1. Nothing pending. After any change: cd sim && python3 web/page_numbers.py && python3 web/build_page.py.
2. Open items for a human: see the Open items section.

## Open items for a human
- The repository URL (https://github.com/gradslab/am_dual_pf-) and the project page URL (https://gradslab.github.io/am_dual_pf-/)
  are placeholders chosen to match the previous repo's naming. The paper's PROJECT-PAGE-URL-TODO must be filled with the real one.
- The bibtex key and note ("Submitted to the European Control Conference") are a guess; adjust once the paper is accepted.
- The cascade animations show the vehicle leaving the field of view after it leaves its path (it ends 6.8 m away). Intended, but worth a look.
- The window study names the theta_2 upper limit as the upper end at 22 of 144 positions, which the paper does not mention
  (the paper says det J = eps_J sets the upper end at grid resolution). The closed loop is consistent with both (th2+ active at 16 % of the steps).
- The am_primitive.urdf written by verify_model.py and the realistic URDF are gitignored (absolute mesh paths).
