#!/usr/bin/env python

import serial
import sys
from datetime import datetime


if __name__ == "__main__":
    filename = sys.argv[1]
    timeRef = datetime.now()
    with open(filename, 'a') as f:
        f.write('%% Reference time: %s\n' % str(timeRef))

        ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        while True:
            n = ser.inWaiting()
            if n>0:
              s = ser.read(n)
              if s[0]=='p':
                ts = datetime.now() - timeRef
                totalSeconds = (ts.microseconds + (ts.seconds + ts.days * 24 * 3600) * 10**6) / float(10**6)
                # Print the time stamp as an offset from the reference time, in milliseconds.
                # f.write('%0.4f\n' % totalSeconds)
                f.write('%0.9f\n' % totalSeconds)
                #print('%0.9f' % totalSeconds)

    ser.close()

#import pylab
#pylab.plot(tic,resp)
#pylab.plot(tic,ppg)
