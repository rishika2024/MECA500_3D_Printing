#!/usr/bin/env python3

import serial
import sys
import time

PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'  # stable path -- raw
                                                            # /dev/ttyUSBN numbering
                                                            # isn't guaranteed constant
                                                            # across reboots/replugs
BAUD = 115200
BED_TEMP = 25
HOTEND_TEMP = 28
PRIME_E = 20
PRIME_F = 200


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
    try:
        ser = serial.Serial(PORT, baudrate=BAUD, timeout=5)
    except serial.SerialException as e:
        print(f"Print setup not connected (could not open {PORT}): {e}", file=sys.stderr)
        sys.exit(1)

    time.sleep(2)  # let the board finish resetting after the serial DTR toggle
    # Discard the boot banner so the first wait_for_ok() below can't match
    # stale text instead of the real reply to the first command sent.
    ser.reset_input_buffer()

    send_gcode(ser, f'M140 S{BED_TEMP}')     # bed start
    send_gcode(ser, f'M104 S{HOTEND_TEMP}')  # hotend start    
    send_gcode(ser, f'M190 R{BED_TEMP}')     # wait for bed
    send_gcode(ser, f'M109 R{HOTEND_TEMP}')  # wait for hotend
   

    print("Ender ready!")
    ser.close()


if __name__ == '__main__':
    main()
