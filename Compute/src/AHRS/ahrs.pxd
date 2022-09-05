cdef extern from "MahonyAHRS.cpp":
    pass

cdef extern from "MadgwickAHRS.cpp":
    pass

cdef extern from "AHRS.cpp":
    pass

cdef extern from "MahonyAHRS.h":
    cdef cppclass MahonyAHRS:
        MahonyAHRS() except +
        void Update( float, float, float, float, float, float, float, float, float)
        void ComputeAngles(float& roll, float& pitch, float& yaw)

cdef extern from "MadgwickAHRS.h":
    cdef cppclass MadgwickAHRS:
        MadgwickAHRS() except +
        void Update( float, float, float, float, float, float, float, float, float)
        void ComputeAngles(float& roll, float& pitch, float& yaw)

cdef extern from "AHRS.h":
    cdef cppclass AHRS:
        AHRS() except +
        void Update( float, float, float, float, float, float, float, float, float)
        void ComputeAngles(float& roll, float& pitch, float& yaw)

