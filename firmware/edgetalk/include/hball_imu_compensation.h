#ifndef HBALL_IMU_COMPENSATION_H
#define HBALL_IMU_COMPENSATION_H

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_IMU_GRAVITY_MPS2 9.80665F

/*
 * Convert the IMU body-forward specific force into horizontal vehicle
 * forward acceleration.  On the current installation body-Y is forward;
 * the caller maps that axis before calling this function.
 */
float hball_imu_specific_force_to_vehicle_accel(
    float specific_force_forward_mps2,
    float body_pitch_rad
);

#ifdef __cplusplus
}
#endif

#endif
