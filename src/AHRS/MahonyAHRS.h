//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float sampleFreq;		// Sample Frequency 
extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern volatile float q0X, q1X, q2X, q3X;	// quaternion of sensor frame relative to auxiliary frame
// extern float roll, pitch, yaw;
extern float axN, ayN, azN;
extern float gxN, gyN, gzN;
extern float mxN, myN, mzN;

//---------------------------------------------------------------------------------------------------
// Function declarations
extern void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
extern void MahonyAHRSComputeAngles(float& roll, float& pitch, float& yaw);


#endif
