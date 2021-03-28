# WirelessSensor
Wireless Sensor Hobbyist Project using a Nordic nRF52840

Known Problem:  Can't get to 90 degree pitch due to 
singularity at North Pole.

Current Problem: Pitch and Roll is slow to respond, Yaw is
very slow to respond.

Yaw responds better with increased gyro sensitivity to 16
and decreased sampleFreqDef to 352, but seems to ring oscillate
overshoot after rotation.

Could we drop gyro readings that fall below a min, and limit
gyro readings that go above a max?

What about measuring the actual angular velocity and using that
to calibrate the gyro?

Can we calibrate the accelerometer?

