#!/usr/bin/env python3
# Standalone print-setup step: connects to the Ender3, heats bed + hotend,
# and primes filament. Runs between reachability and trajectory so the arm
# sits wherever reachability left it (home) while this happens, instead of
# trajectory.cpp doing it mid-goal.
#
# All values come from machine_settings.yaml (--config, default: the installed
# copy next to this script).
import argparse
import os
import sys
import time

import serial
import yaml
from ament_index_python.packages import get_package_share_directory

DEFAULT_CONFIG = os.path.join(get_package_share_directory('msr_meca500_print_pipeline'), 'config', 'machine_settings.yaml')


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


def wait_for_ok(ser):
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
        if 'ok' in line.lower():
            break


def send_gcode(ser, cmd):
    ser.write((cmd.strip() + '\n').encode('utf-8'))
    print(f">> {cmd}")
    wait_for_ok(ser)


def main():
    ap = argparse.ArgumentParser(description="Heat + prime the Ender3 before a print.")
    ap.add_argument('--config', default=DEFAULT_CONFIG, help='path to machine_settings.yaml')
    args = ap.parse_args()
    cfg = load_config(args.config)

    port = cfg['printer_serial_port']
    baud = cfg['printer_baud']
    bed_temp = cfg['start_bed_temp_c']
    hotend_temp = cfg['start_hotend_temp_c']

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=5)
    except serial.SerialException as e:
        print(f"Print setup not connected (could not open {port}): {e}", file=sys.stderr)
        sys.exit(1)

    time.sleep(2)  # let the board finish resetting after the serial DTR toggle
    # Discard the boot banner so the first wait_for_ok() below can't match
    # stale text instead of the real reply to the first command sent.
    ser.reset_input_buffer()

    send_gcode(ser, f'M140 S{bed_temp}')      # bed start
    send_gcode(ser, f'M104 S{hotend_temp}')   # hotend start
    send_gcode(ser, f'M190 R{bed_temp}')      # wait for bed
    send_gcode(ser, f'M109 R{hotend_temp}')   # wait for hotend

    print("Ender ready!")
    ser.close()


if __name__ == '__main__':
    main()
