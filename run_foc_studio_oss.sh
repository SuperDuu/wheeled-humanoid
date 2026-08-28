#!/bin/bash
# ==============================================================================
# Wheeled-Humanoid FOC Studio OSS Launcher
# ==============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$WORKSPACE_DIR/software/foc_studio_oss"

echo "======================================================================"
echo "Starting Wheeled-Humanoid FOC Studio OSS"
echo "App Directory: $APP_DIR"
echo "======================================================================"

cd "$APP_DIR"
exec ./run_studio.sh
