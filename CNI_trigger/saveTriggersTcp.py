#!/usr/bin/env python

import serial
import sys
from datetime import datetime
import socket

HOST = ''                 # Symbolic name meaning all available interfaces
PORT = 50007              # Arbitrary non-privileged port


if __name__ == "__main__":
    filename = sys.argv[1]
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(1)
    # Maybe loop here to allow this to keep running?
    conn, addr = s.accept()
    print 'Connected by', addr
    conn.SetBlocking(True)
    data = conn.recv(16)
    if not data: break
    if data[0] == 's':
        timeRef = datetime.now()
        with open(filename, 'a') as f:
            f.write('%% Reference time: %s\n' % str(timeRef))
            ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
            # Send the command to enable pulses
            ser.write('[p]\n');
            # Display the firmware greeting
            out = ledSer.readlines()
            for l in out: print(l),

            conn.SetBlocking(False)
            while True:
                n = ser.inWaiting()
                if n>0:
                    s = ser.read(n)
                if s[0]=='p':
                    ts = datetime.now() - timeRef
                    totalSeconds = (ts.microseconds + (ts.seconds + ts.days * 24 * 3600) * 10**6) / float(10**6)
                    # Print the time stamp as an offset from the reference time, in milliseconds.
                    f.write('%0.9f\n' % totalSeconds)
                    conn.send('p')
                data = conn.recv(16)
                if data and data[0]=='e':
                    break;

            ser.close()
            f.close()

    conn.close()


#import pylab
#pylab.plot(tic,resp)
#pylab.plot(tic,ppg)
