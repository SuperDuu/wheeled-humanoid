#!/bin/bash

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$WORKSPACE_DIR/software/foc_studio_oss/run_studio.sh"
