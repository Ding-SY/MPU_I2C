//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#include "arm_math.h"
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float32_t beta;				// algorithm gain
extern volatile float32_t q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern volatile float32_t roll,pitch,yaw;
//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az, float32_t mx, float32_t my, float32_t mz);
void MadgwickAHRSupdateIMU(float32_t gx, float32_t gy, float32_t gz, float32_t ax, float32_t ay, float32_t az);
void getRPY();
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
