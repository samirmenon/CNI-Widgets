#!/usr/bin/env python

import serial
import sys, os, time

def main(argv):
    if len(argv) > 1:
        device = argv[1]
    else:
        device = '/dev/ttyACM0'

    if not os.path.exists(device):
        sys.stderr.write("ERROR: Serial device %r not found!\n\n" % (device,))
        return 1

    ser = serial.Serial(device, 115200, timeout=1)
    time.sleep(0.05)
    # Send an out pulse
    ser.write('[t]\n');
    ser.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv))

