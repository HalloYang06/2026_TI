#ifndef HBALL_DEPLOYMENT_CONFIG_H
#define HBALL_DEPLOYMENT_CONFIG_H

/*
 * Measured mechanism/vision coordinates. Position is negative toward hinge C.
 * The user confirmed an approximately 10 mm ball diameter. Replace the
 * nominal 5 mm radius with the caliper result before final qualification.
 */
#define HBALL_DEPLOYMENT_HINGE_TO_VISION_ZERO_M 0.155F
#define HBALL_DEPLOYMENT_PHYSICAL_HALF_LENGTH_M 0.112F
#define HBALL_DEPLOYMENT_BALL_RADIUS_M 0.005F

/*
 * Formal control stays in shadow until the RS00 encoder reading at a
 * mechanically level pipe is measured and this validity flag is changed.
 */
#define HBALL_LINKAGE_LEVEL_ENCODER_VALID 0U
#define HBALL_LINKAGE_LEVEL_ENCODER_RAD 0.0F

#endif
