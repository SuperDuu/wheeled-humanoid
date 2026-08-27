#!/usr/bin/env bash
# Quick runner for FOC Closed-Loop Diagnostics
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PYTHON="$DIR/software/foc_studio_oss/.venv/bin/python3"

if [ -f "$VENV_PYTHON" ]; then
    PYTHON_CMD="$VENV_PYTHON"
else
    PYTHON_CMD="python3"
fi

$PYTHON_CMD "$DIR/software/foc_studio_oss/foc_diagnose.py" "$@"
