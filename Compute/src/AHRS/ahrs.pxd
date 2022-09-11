
cdef extern from "MahonyAHRS.cpp":
    pass

cdef extern from "MadgwickAHRS.cpp":
    pass

cdef extern from "AHRS.cpp":
    pass

cdef extern from "AHRS.h":
    ctypedef enum EULER_ANGLE_SELECT_t:
        ROLL,
        PITCH,
        YAW

    ctypedef enum QUATERNION_SELECT_t:
        Q0,
        Q1,
        Q2,
        Q3

    cdef cppclass AHRS:
        AHRS() except +
        void Update( float, float, float, float, float, float, float, float, float)
        void ComputeAngles(float& roll, float& pitch, float& yaw)
        float GetAngle(EULER_ANGLE_SELECT_t angle_select)
        float GetQuaternion(QUATERNION_SELECT_t quaternion_select)

cdef extern from "MahonyAHRS.h":
    cdef cppclass MahonyAHRS:
        MahonyAHRS() except +
        void Update( float, float, float, float, float, float, float, float, float)
        void ComputeAngles(float& roll, float& pitch, float& yaw)
        float GetAngle(EULER_ANGLE_SELECT_t angle_select)
        float GetQuaternion(QUATERNION_SELECT_t quaternion_select)

cdef extern from "MadgwickAHRS.h":
    cdef cppclass MadgwickAHRS:
        MadgwickAHRS() except +
        void Update( float, float, float, float, float, float, float, float, float)
        void ComputeAngles(float& roll, float& pitch, float& yaw)
        float GetAngle(EULER_ANGLE_SELECT_t angle_select)
        float GetQuaternion(QUATERNION_SELECT_t quaternion_select)

