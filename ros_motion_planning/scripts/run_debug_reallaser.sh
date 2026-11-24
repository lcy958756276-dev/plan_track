#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_DEBUG_REAL_LASER=1 exec bash "$SCRIPT_DIR/run_debug.sh" "$@"
