#!/usr/bin/env python
import serial, time, array, random

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
data = array.array('h',[0, 0, 0, 0, 0, 0])
for i in range(1,10000):
  data[0] = i
  data[4] = random.randint(-3, 10)
  # "Pulse" every second
  if i%200>=0 and i%200<20:
    data[4] = data[4]+10
  ser.write(data.tostring())   # write a data packet
  data.tostring()
  time.sleep(0.004)

ser.close()

