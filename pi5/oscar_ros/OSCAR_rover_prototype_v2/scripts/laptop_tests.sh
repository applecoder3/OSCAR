#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m compileall -q "$ROOT/oscar_preflight/oscar_preflight" "$ROOT/oscar_preflight/launch" "$ROOT/scripts"
PYTHONPATH="$ROOT/oscar_preflight" python3 -m unittest discover -s "$ROOT/tests" -v
PYTHONPATH="$ROOT/oscar_preflight" python3 -m unittest discover -s "$ROOT/oscar_preflight/test" -v
echo 'PASS: Python static compile and pure-code tests. ROS runtime build still required.'
