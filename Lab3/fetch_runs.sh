#!/bin/sh
# Copy the run logs (runs/run_NNN.txt and .csv) off the EV3 into ./runs here.
# Usage: ./fetch_runs.sh [host]     Default host ev3dev.local; password: maker
set -e
HOST="${1:-ev3dev.local}"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$HERE/runs"
scp "robot@$HOST:/home/robot/$(basename "$HERE")/runs/run_*" "$HERE/runs/"
echo "Run logs saved in $HERE/runs"
