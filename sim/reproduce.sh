#!/usr/bin/env bash
# Ground-up reproduction: install -> verification gate -> the nine runs -> posture study -> numbers -> figures -> page.
# Run from sim/:  bash reproduce.sh      (add "anim" as the first argument to also render the animations)
set -euo pipefail
cd "$(dirname "$0")"

echo "== 1/6 install =="
if [ ! -d .venv ]; then python3 -m venv .venv; fi
source .venv/bin/activate
pip install --quiet -r requirements.txt

echo "== 2/6 verification gate =="
python3 model/verify_model.py
python3 model/test_relative_degree.py

echo "== 3/6 the nine runs =="
python3 drake/sim_drake.py D3
python3 drake/sim_drake.py D4 --modes relaxed,filter,nominal
python3 drake/sim_drake.py D4 --modes relaxed --hz 1000 --tag _1khz
python3 drake/sim_drake.py D5 --arm-scale 1.02 --tag _a102
python3 drake/sim_drake.py D5 --veh-scale 1.05 --tag _v105
python3 drake/sim_drake.py D5 --arm-scale 1.05 --T 12 --tag _a105
python3 drake/sim_drake.py D5 --veh-scale 1.10 --T 12 --tag _v110

echo "== 4/6 posture study and numbers =="
python3 studies/posture_study.py
python3 metrics.py

echo "== 5/6 figures =="
python3 drake/render_drake.py D3_nominal --cam close --frame 6000
python3 paper_figures.py
python3 web/figs_web.py

if [ "${1:-}" = "anim" ]; then
  echo "== animations =="
  for s in "D4_relaxed side" "D4_relaxed orbit" "D3_nominal orbit" "D3_nominal side" "D4_filter side" "D4_filter orbit" "D4_nominal side"; do
    set -- $s; python3 drake/render_drake.py $1 --cam $2
  done
  python3 web/anim_web.py
fi

echo "== 6/6 page =="
python3 web/page_numbers.py
python3 web/build_page.py

echo
echo "Done. Numbers: results/web/numbers.json   Figures: results/figs/   Page: ../docs/index.html"
