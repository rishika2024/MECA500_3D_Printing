#!/usr/bin/env bash
# Runs the reachability node, then the gcode parser, then sends the
# resulting file to the print service -- stops at the first failure.
set -e

MODEL_FILE="$1"
OUT_FILE="$2"
REACH_CSV="$3"

LAYERS="${4:-0}"

GCODE_PARSER="$(ros2 pkg prefix gcode)/share/gcode/gcode_parser.py"

ros2 run meca500_demo reachability --ros-args -p out_file:="$REACH_CSV"
python3 "$GCODE_PARSER" "$MODEL_FILE" "$OUT_FILE" --reach-csv "$REACH_CSV" -l "$LAYERS"
ros2 service call /gcode_file_service meca500_demo/srv/GcodeFile "{file_path: '$OUT_FILE'}"
