# Simulation code

Everything behind the paper *"Dual Path Following for an Aerial Manipulator Under State and Input
Constraints via Transverse Feedback Linearization"* and the [project page](https://gradslab.github.io/am_dual_pf-/).

A quadrotor carries a 3-DOF arm. The center of mass of the combined system follows a vehicle path and
the end effector follows a second path. The controller writes the transverse feedback linearization as a
quadratic program whose transverse, vehicle speed and heading rows are hard equalities and whose
end-effector speed row carries one scalar slack. Joint limits, an attitude margin, thrust bounds and a
singularity margin enter as exponential control barrier function rows, and the moment and joint torque
bounds as box rows. The program reduces to a scalar problem over an interval, solved in closed form.
Two comparisons run on the same scenario: the nominal controller passed through a safety filter with the
same rows, and the nominal controller alone.

## Layout

```
model/      params.py, derive_model_fast.py, derive_extra.py   symbolic rod model (sympy) and the arm singularity
            am_model_gen.py, am_extra_gen.py                    generated code (derive_* write them)
            am_controller.py                                    nominal TFL, relaxed TFL-QP, safety-filtered cascade
            verify_model.py, test_relative_degree.py            verification gate
drake/      make_realistic_urdf.py   URDF of the Skydio X2 with the OpenMANIPULATOR-X
            drake_model.py           MultibodyPlant wrapper that supplies the output chains to the controller
            sim_drake.py             the closed-loop runs (D3 nominal, D4 incompatible speeds, D5 model mismatch)
            render_drake.py          animations with the VTK renderer of Drake
studies/    posture_study.py         postures compatible with both paths: feasibility sampling, lead window
metrics.py                           every number of Section VI of the paper, from the logs
paper_figures.py                     the three figures of the paper
web/        figs_web.py, anim_web.py, page_numbers.py, build_page.py   the project page in ../docs
results/    logs (npz, json), figures, animations, studies, numbers
```

## Reproduce everything

```sh
git clone https://github.com/gradslab/am_dual_pf-.git
cd am_dual_pf/sim
bash reproduce.sh          # install -> gate -> 9 runs -> study -> numbers -> figures -> page
bash reproduce.sh anim     # the same, with the seven animations
```

`reproduce.sh` runs the steps below. Use them individually if you prefer.

### 1. Install

Python 3.10 or newer on macOS or Linux. Drake installs from pip. The animations also need ffmpeg.

```sh
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

### 2. Verification gate

`verify_model.py` checks the symbolic model: energy conservation in free flight, the center-of-mass
identity, Koenig's theorem, the reduced rotational inertia matrix, and a Drake cross-check of the
energies and accelerations for the same state and forces. `test_relative_degree.py` checks the identities
behind the normal form: the attitude kinematics, the Jacobian bias terms, the path-output chain rule
against sympy, the closed form of the determinant of the decoupling matrix at random states, that the
body origin as vehicle output makes the decoupling matrix singular, and that no input appears before the
fourth derivative of the center of mass and the second derivative of the end effector. If a gate fails,
nothing downstream is worth running.

```sh
python3 model/verify_model.py
python3 model/test_relative_degree.py
```

The generated modules `am_model_gen.py` and `am_extra_gen.py` are tracked. To rebuild them from the
derivation, run `python3 model/derive_model_fast.py` (a few minutes) and `python3 model/derive_extra.py`.

### 3. The nine runs

The scenario (paths, speeds, start, gains, limits) lives in `drake/sim_drake.py`. The commands select the
scenario, the controller, the rate and the plant scaling. Each run takes between twenty seconds and a
few minutes. A run writes `results/<name>.npz` (the logged states, outputs, inputs, slack, interval and
margins at every controller step) and `results/<name>_active.json` (the active row at every step).

```sh
python3 drake/sim_drake.py D3                                      # nominal controller, equal speeds (Fig. 2)
python3 drake/sim_drake.py D4 --modes relaxed,filter,nominal       # incompatible speeds, three controllers (Fig. 3)
python3 drake/sim_drake.py D4 --modes relaxed --hz 1000 --tag _1khz
python3 drake/sim_drake.py D5 --arm-scale 1.02 --tag _a102         # model mismatch
python3 drake/sim_drake.py D5 --veh-scale 1.05 --tag _v105
python3 drake/sim_drake.py D5 --arm-scale 1.05 --T 12 --tag _a105
python3 drake/sim_drake.py D5 --veh-scale 1.10 --T 12 --tag _v110
```

### 4. The posture study, the numbers and the figures

```sh
python3 studies/posture_study.py      # results/studies/posture_study.json, sampling.npz, lead_window.npz
python3 metrics.py                    # prints every number quoted in Section VI
python3 drake/render_drake.py D3_nominal --cam close --frame 6000   # the frame behind Fig. 1
python3 paper_figures.py              # results/figs/concept, fig_nominal, fig_relaxed (PDF and PNG)
python3 web/figs_web.py               # results/figs/W_*.png, the figures of the project page
```

### 5. Animations and the page

The renderer needs the VTK render engine of Drake, which the pip wheel includes, and `anim_web.py`
needs ffmpeg.

```sh
for s in "D4_relaxed side" "D4_relaxed orbit" "D3_nominal orbit" "D3_nominal side" \
         "D4_filter side" "D4_filter orbit" "D4_nominal side"; do
  set -- $s; python3 drake/render_drake.py $1 --cam $2
done
python3 web/anim_web.py        # docs/anim/*.mp4 with poster frames, two gifs
python3 web/page_numbers.py    # results/web/numbers.json
python3 web/build_page.py      # docs/index.html and docs/figs
```

**Frames.** The inertial frame has its third axis up, as in the paper. The controller state is
`(q, v, u_t, du_t/dt)` with the Drake conventions for `q` and `v` (quaternion, position, joint angles;
angular velocity in the inertial frame, velocity of the body origin, joint rates).
