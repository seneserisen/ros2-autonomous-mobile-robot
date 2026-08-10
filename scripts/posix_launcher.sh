#!/usr/bin/env sh
set -u

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_ROOT=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
ACTION=${1:-}

if [ -z "$ACTION" ]; then
  echo "ERROR: No FaultNav workflow action was supplied." >&2
  exit 2
fi

if command -v python3 >/dev/null 2>&1; then
  PYTHON=python3
elif command -v python >/dev/null 2>&1; then
  PYTHON=python
else
  echo "ERROR: Python was not found." >&2
  echo "Install Python 3.10 or newer, then run sh setup.sh again." >&2
  exit 1
fi

cd "$PROJECT_ROOT" || exit 1
exec "$PYTHON" "$SCRIPT_DIR/faultnav_workflow.py" "$ACTION"
