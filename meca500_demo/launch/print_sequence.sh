#!/usr/bin/env bash
# Runs the reachability node (which ends by returning the arm home), then
# printer.py (heats + primes the Ender3 while the arm sits at home), then
# the gcode parser, then sends the resulting file to the print service --
# stops at the first failure.
set -e

MODEL_FILE="$1"
OUT_FILE="$2"
REACH_CSV="$3"

LAYERS="${4:-0}"
USE_EXTRUDER="${5:-true}"
MOCK_HARDWARE="${6:-true}"
REAL_PRINT="${7:-true}"

GCODE_PARSER="$(ros2 pkg prefix gcode)/share/gcode/gcode_parser.py"
START_PRINTER_SEQUENCE="$(ros2 pkg prefix meca500_demo)/share/meca500_demo/start_print_sequence.py"
END_PRINTER_SEQUENCE="$(ros2 pkg prefix meca500_demo)/share/meca500_demo/end_print_sequence.py"

ros2 run meca500_demo reachability --ros-args -p out_file:="$REACH_CSV" -p use_extruder:="$USE_EXTRUDER"
if [ "$USE_EXTRUDER" = "true" ] && [ "$REAL_PRINT" = "true" ] && [ "$MOCK_HARDWARE" = "false" ]; then
  # -u: unbuffered stdout. Without it, Python fully block-buffers its
  # print() output instead of line-buffering as soon as stdout isn't a
  # real terminal (which is the case here -- ros2 launch's output="screen"
  # pipes this script's output through itself), so nothing would show up
  # until the whole heat/prime sequence finished and the process exited.
  python3 -u "$START_PRINTER_SEQUENCE"
fi
python3 "$GCODE_PARSER" "$MODEL_FILE" "$OUT_FILE" --reach-csv "$REACH_CSV" -l "$LAYERS"
ros2 service call /gcode_file_service meca500_demo/srv/GcodeFile "{file_path: '$OUT_FILE'}"

if [ "$USE_EXTRUDER" = "true" ] && [ "$REAL_PRINT" = "true" ] && [ "$MOCK_HARDWARE" = "false" ]; then
  python3 -u "$END_PRINTER_SEQUENCE"
fi
