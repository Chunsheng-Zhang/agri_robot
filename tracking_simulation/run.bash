#!/usr/bin/env bash
set -euo pipefail
agri_sim_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
agri_project_dir="$(dirname -- "$agri_sim_dir")"
mkdir -p "$agri_sim_dir/.cache/tmp" "$agri_sim_dir/.cache/matplotlib"
export TMPDIR="$agri_sim_dir/.cache/tmp"
export MPLCONFIGDIR="$agri_sim_dir/.cache/matplotlib"
export PYTHONDONTWRITEBYTECODE=1
exec "$agri_project_dir/agri_backend/.venv/bin/python" -B "$agri_sim_dir/run_tracking.py" "$@"
