# distutils: language = c++

from ahrs cimport MahonyAHRS 

# Create a Cython extension type which holds a C++ instance
# as an attribute and create a bunch of forwarding methods
# Python extension type.
cdef class PyMahonyAHRS:
    cdef MahonyAHRS c_mahonyAHRS  # Hold a C++ instance which we're wrapping

    def __cinit__(self):
        self.c_mahonyAHRS = MahonyAHRS()

    def Update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        self.c_mahonyAHRS.Update(gx, gy, gz, ax, ay, az, mx, my, mz)

    def ComputeAngles(self, float& roll, float& pitch, float& yaw):
        self.c_mahonyAHRS.ComputeAngles(roll, pitch, yaw)

