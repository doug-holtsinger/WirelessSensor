#!/usr/bin/env python3


from ahrs import PyMahonyAHRS 

ahrso = PyMahonyAHRS()
print(ahrso)
roll = pitch = yaw = 0.0
while True:
    gx = gy = gz = 0.0

    ax = -1000.0 
    #ax = 0.0 
    ay = 0.0 
    az = 0.0

    mx = 538.0 
    my = -241.0
    mz = -273

    ahrso.Update(gx, gy, gz, ax, ay, az, mx, my, mz)
    ahrso.ComputeAngles(roll, pitch, yaw)

    roll = ahrso.GetAngle(0)
    pitch = ahrso.GetAngle(1)
    yaw = ahrso.GetAngle(2)

    q0 = ahrso.GetQuaternion(0)
    q1 = ahrso.GetQuaternion(1)
    q2 = ahrso.GetQuaternion(2)
    q3 = ahrso.GetQuaternion(3)

    print("Roll = %2.1f    Pitch = %2.1f   Yaw = %3.1f  q0=%2.2f q1=%2.2f q2=%2.2f q3=%2.2f" % ( roll, pitch, yaw , q0, q1, q2 , q3))

