#!/usr/bin/env python3
# Standalone print-teardown step: sets the Ender3's bed + hotend to their
# cool-down targets after a print. All values come from machine_settings.yaml
# (--config, default: the installed copy next to this script).
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
    ap = argparse.ArgumentParser(description="Cool the Ender3 down after a print.")
    ap.add_argument('--config', default=DEFAULT_CONFIG, help='path to machine_settings.yaml')
    args = ap.parse_args()
    cfg = load_config(args.config)

    port = cfg['printer_serial_port']
    baud = cfg['printer_baud']
    bed_temp = cfg['end_bed_temp_c']
    hotend_temp = cfg['end_hotend_temp_c']

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=5)
    except serial.SerialException as e:
        print(f"Print teardown not connected (could not open {port}): {e}", file=sys.stderr)
        sys.exit(1)

    time.sleep(2)  # let the board finish resetting after the serial DTR toggle
    ser.reset_input_buffer()

    send_gcode(ser, f'M140 S{bed_temp}')      # bed target
    send_gcode(ser, f'M104 S{hotend_temp}')   # hotend target
    send_gcode(ser, f'M190 R{bed_temp}')      # wait for bed
    send_gcode(ser, f'M109 R{hotend_temp}')   # wait for hotend

    print("Ender cooled down.")
    ser.close()


if __name__ == '__main__':
    main()
