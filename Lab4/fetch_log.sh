#!/bin/sh
# Copy the Lab 4 run log (run_log.txt, every run appended) off the EV3.
# Usage: ./fetch_log.sh [host]     Default host ev3dev.local; password: maker
set -e
HOST="${1:-ev3dev.local}"
HERE="$(cd "$(dirname "$0")" && pwd)"
PROJECT="$(basename "$(dirname "$HERE")")"
scp "robot@$HOST:/home/robot/$PROJECT/$(basename "$HERE")/run_log.txt" "$HERE/run_log.txt"
echo "Run log saved in $HERE/run_log.txt"
