#!/usr/bin/env python3
# Standalone check: just reports wherever Marlin currently thinks X/Y/Z is,
# without homing -- G28 on this printer has been triggering "Homing Failed"
# / a full kill() halt, so this deliberately skips it and only queries
# M114 for the position already tracked in firmware.
import re
import sys
import time

import serial

PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
BAUD = 115200


def wait_for_ok(ser):
    lines = []
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
            lines.append(line)
        if 'ok' in line.lower():
            return '\n'.join(lines)


def send_gcode(ser, cmd):
    ser.write((cmd.strip() + '\n').encode('utf-8'))
    print(f">> {cmd}")
    return wait_for_ok(ser)


def main():
    try:
        ser = serial.Serial(PORT, baudrate=BAUD, timeout=5)
    except serial.SerialException as e:
        print(f"Could not open {PORT}: {e}", file=sys.stderr)
        sys.exit(1)

    time.sleep(2)  # let the board finish resetting after the serial DTR toggle
    ser.reset_input_buffer()

    reply = send_gcode(ser, 'M114')  # report current position (no G28 -- see note above)

    match = re.search(r'X:([-\d.]+)\s+Y:([-\d.]+)\s+Z:([-\d.]+)', reply)
    if match:
        x, y, z = match.groups()
        print(f"\nHome position: X={x} Y={y} Z={z}")
    else:
        print("\nCould not parse position from M114 reply above.", file=sys.stderr)

    ser.close()


if __name__ == '__main__':
    main()
