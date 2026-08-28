#!/usr/bin/env bash
# Sets the bed pose on /table_service, then runs the reachability node (which
# ends by returning the arm home), then start_print_sequence.py (heats the
# Ender3 while the arm sits at home), then the gcode parser, then sends the
# resulting file to the print service, then end_print_sequence.py -- stops at
# the first failure.
#
# Everything tunable lives in $CONFIG_DIR (machine_settings.yaml,
# bed_settings.yaml, print_tuning.yaml); the args below are just the per-print
# inputs plus a couple of launch-level overrides.
set -e

MODEL_FILE="$1"
OUT_FILE="$2"
REACH_CSV="$3"
CONFIG_DIR="$4"

LAYERS="${5:-0}"
USE_EXTRUDER="${6:-true}"
MOCK_HARDWARE="${7:-true}"
REAL_PRINT="${8:-true}"
DEFAULT_BED="${9:-}"          # empty -> use bed_settings.yaml's value

MACHINE_CFG="$CONFIG_DIR/machine_settings.yaml"
BED_CFG="$CONFIG_DIR/bed_settings.yaml"
TUNING_CFG="$CONFIG_DIR/print_tuning.yaml"

GCODE_PARSER="$(ros2 pkg prefix msr_gcode)/share/msr_gcode/gcode_parser.py"
START_PRINTER_SEQUENCE="$(ros2 pkg prefix msr_meca500_print_pipeline)/share/msr_meca500_print_pipeline/start_print_sequence.py"
END_PRINTER_SEQUENCE="$(ros2 pkg prefix msr_meca500_print_pipeline)/share/msr_meca500_print_pipeline/end_print_sequence.py"

# Bed pose first -- reachability and the gcode transform both read /table_marker.
BED_ARGS=(--ros-args --params-file "$MACHINE_CFG" --params-file "$BED_CFG")
[ -n "$DEFAULT_BED" ] && BED_ARGS+=(-p "default_bed:=$DEFAULT_BED")
ros2 run msr_meca500_print_pipeline bed_from_touches "${BED_ARGS[@]}"

ros2 run msr_meca500_print_pipeline reachability --ros-args \
  --params-file "$MACHINE_CFG" --params-file "$TUNING_CFG" \
  -p "out_file:=$REACH_CSV" -p "use_extruder:=$USE_EXTRUDER"

if [ "$USE_EXTRUDER" = "true" ] && [ "$REAL_PRINT" = "true" ] && [ "$MOCK_HARDWARE" = "false" ]; then
  # -u: unbuffered stdout. Without it, Python fully block-buffers its
  # print() output instead of line-buffering as soon as stdout isn't a
  # real terminal (which is the case here -- ros2 launch's output="screen"
  # pipes this script's output through itself), so nothing would show up
  # until the whole heat/prime sequence finished and the process exited.
  python3 -u "$START_PRINTER_SEQUENCE" --config "$MACHINE_CFG"
fi

python3 "$GCODE_PARSER" "$MODEL_FILE" "$OUT_FILE" --reach-csv "$REACH_CSV" -l "$LAYERS"
ros2 service call /gcode_file_service msr_meca500_print_pipeline/srv/GcodeFile "{file_path: '$OUT_FILE'}"

if [ "$USE_EXTRUDER" = "true" ] && [ "$REAL_PRINT" = "true" ] && [ "$MOCK_HARDWARE" = "false" ]; then
  python3 -u "$END_PRINTER_SEQUENCE" --config "$MACHINE_CFG"
fi
