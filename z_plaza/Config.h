#ifndef CONFIG_H
#define CONFIG_H

// ================================================================
// ===                      Motor Pins                           ===
// ================================================================
#define IN1 19
#define IN2 18
#define ENA 4
#define IN3 17
#define IN4 16
#define ENB 5

// ================================================================
// ===                    Ultrasonic Pins                         ===
// ================================================================
#define FRONT_TRIG 2
#define FRONT_ECHO 34
#define RIGHT_TRIG 23
#define RIGHT_ECHO 36
#define LEFT_TRIG  15
#define LEFT_ECHO  39

// ================================================================
// ===                 Straight-Line PID (MPU)                   ===
// ================================================================
#define KP_STRAIGHT    5.0
#define KI_STRAIGHT    0.0
#define KD_STRAIGHT    1.2
#define LIMIT_STRAIGHT 255

// ================================================================
// ===            Straight-Line PID (Wall / Ultrasonic)          ===
// ================================================================
#define KP_WALL        5.0
#define KI_WALL        0.0
#define KD_WALL        0.5
#define LIMIT_WALL     255
#define WALL_MODE_ACTIVATE_MS 350   // المود يتفعل بعد ثانية من المشي للأمام
#define WALL_CRITICAL_DIST_CM 5      // يشتغل لو واحد من الحساسات قرأ أقل من أو يساوي 3 سم

// ================================================================
// ===                    Turning PID                            ===
// ================================================================
#define KP_TURN              6.3
#define KI_TURN              0.0
#define KD_TURN              0.22
#define LIMIT_TURN           255
#define TURN_SET_POINT       84
#define TURN_SIGN            1
#define MINIMUM_TURN_SPEED   120
#define TURN_ANGLE_TOLERANCE 1.0

// ================================================================
// ===                   General Config                          ===
// ================================================================
#define MPU_ADDR          0x68
#define SERIAL_BAUD       115200
#define DEFAULT_CAR_SPEED 255
#define MOTOR_PWM_MAX     255

// Distances
#define MIN_STOP_DIST      15
#define MAX_STOP_DIST      15
#define FORWARD_RAMP_TIME_MS 1200

// FreeRTOS task periods
#define IMU_TASK_MS          2
#define CONTROL_TASK_MS      10
#define ULTRASONIC_TASK_MS   70
#define PAUSE_BEFORE_SCAN_MS 120
#define US_SIDE_SAMPLES      3

// Braking
#define BRAKE_DURATION_MS 0//150
#define BRAKE_POWER       180

// MPU calibration
// #define MPU_G_Z_ERROR -91.42

// ================================================================
// ===                    Arm Servo Pins                          ===
// ================================================================
#define SERVO1_PIN  26
#define SERVO2_PIN  33
#define SERVO3_PIN  14
#define SERVO4_PIN  27
#define SERVO5_PIN  32
#define SERVO6_PIN  25

// ================================================================
// ===                   Arm Servo Config                         ===
// ================================================================
#define SERVO_STEP_DEG       2    // default (fallback)
#define SERVO_MAX_STEP_DEG   6    // default (fallback)
#define SERVO_MIN_ANGLE      0
#define SERVO_MAX_ANGLE      180

#define SERVO1_INITIAL_ANGLE 0
#define SERVO2_INITIAL_ANGLE 78
#define SERVO3_INITIAL_ANGLE 180
#define SERVO4_INITIAL_ANGLE 165
#define SERVO5_INITIAL_ANGLE 60
#define SERVO6_INITIAL_ANGLE 180

// Per-servo discrete step (button presses)
#define SERVO2_STEP_DEG      1
#define SERVO3_STEP_DEG      2
#define SERVO4_STEP_DEG      2
#define SERVO5_STEP_DEG      2
#define SERVO6_STEP_DEG      2

// Per-servo analog max step (proportional sticks)
#define SERVO1_MAX_STEP_DEG  6
#define SERVO6_MAX_STEP_DEG  6

// Servo5 (Gripper) manual limits
#define SERVO5_OPEN_ANGLE    150   // D-pad Right max
#define SERVO5_CLOSE_ANGLE   30    // D-pad Down max

#define SERVO_DEADZONE       180
#define DRIVE_STEER_DEADZONE 20

// ================================================================
// ===          Semi-Auto Mode — Per-Servo Snap Angles            ===
// ================================================================
// Servo2 (Arm joint 2) — R1 / L1
#define SEMI_S2_MAX_UP    150
#define SEMI_S2_MAX_DOWN  30

// Servo3 (Arm joint 3) — Square / Circle
#define SEMI_S3_MAX_UP    160
#define SEMI_S3_MAX_DOWN  20

// Servo4 (Arm joint 4) — Cross / Triangle
#define SEMI_S4_MAX_UP    180
#define SEMI_S4_MAX_DOWN  130

// Servo5 (Gripper) — D-pad Right / D-pad Down
#define SEMI_S5_MAX_UP    150
#define SEMI_S5_MAX_DOWN  30

// Servo6 (Shovel) — Circle / Square  (used inside SUBMODE_SHOVEL)
#define SEMI_S6_MAX_UP    180
#define SEMI_S6_MAX_DOWN  0

// Snap motion speed (degrees advanced per control loop tick = every CONTROL_TASK_MS)
#define SEMI_S1_SNAP_STEP   8
#define SEMI_S2_SNAP_STEP   8
#define SEMI_S3_SNAP_STEP   8
#define SEMI_S4_SNAP_STEP   8
#define SEMI_S5_SNAP_STEP   8
#define SEMI_S6_SNAP_STEP   12

#endif