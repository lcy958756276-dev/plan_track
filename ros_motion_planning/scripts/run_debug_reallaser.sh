#!/usr/bin/env bash
#sudo chmod 666 /dev/ttyCH343USB0 
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_DEBUG_REAL_LASER=1 exec bash "$SCRIPT_DIR/run_debug.sh" "$@"
