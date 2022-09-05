#!/usr/bin/env python3


from ahrs import PyMahonyAHRS 

ahrso = PyMahonyAHRS()
print(ahrso)
roll = pitch = yaw = 0.0
while True:
    gx = gy = gz = 0.0

    ax = -1000.0 
    ay = 0.0 
    az = 0.0

    mx = 538.0 
    my = -241.0
    mz = -273

    ahrso.Update(gx, gy, gz, ax, ay, az, mx, my, mz)
    ahrso.ComputeAngles(roll, pitch, yaw)
    print("Roll = %2.1f    Pitch = %2.1f   Yaw = %3.1f" % ( roll, pitch, yaw ))

