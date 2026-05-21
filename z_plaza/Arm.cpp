#include "Arm.h"

// ================================================================
// ===                   Servo Objects                            ===
// ================================================================
static Servo servo1;
static Servo servo2;
static Servo servo3;
static Servo servo4;
static Servo servo5;
static Servo servo6;

// ================================================================
// ===           Current Angles (positional servos)               ===
// ================================================================
static int servo1Angle = SERVO1_INITIAL_ANGLE;
static int servo2Angle = SERVO2_INITIAL_ANGLE;
static int servo3Angle = SERVO3_INITIAL_ANGLE;
static int servo4Angle = SERVO4_INITIAL_ANGLE;
static int servo5Angle = SERVO5_INITIAL_ANGLE;
static int servo6Angle = SERVO6_INITIAL_ANGLE;

// ================================================================
// ===                    Lock States                             ===
// ================================================================
static bool baseLocked   = false;   // R3: locks servo1 (base) only
static bool armLocked    = false;   // reserved / unused
static bool shovelLocked = false;

// ================================================================
// ===                     Init                                   ===
// ================================================================
void initArm() {
  servo1.attach(SERVO1_PIN); servo1.write(servo1Angle);
  servo2.attach(SERVO2_PIN); servo2.write(servo2Angle);
  servo3.attach(SERVO3_PIN); servo3.write(servo3Angle);
  servo4.attach(SERVO4_PIN); servo4.write(servo4Angle);
  servo5.attach(SERVO5_PIN); servo5.write(servo5Angle);
  servo6.attach(SERVO6_PIN); servo6.write(servo6Angle);
}

// ================================================================
// ===             Stop / Lock / Reset Helpers                    ===
// ================================================================
void stopArmContinuous() {
}

void toggleArmLock() {
  armLocked = !armLocked;
}

void toggleBaseLock() {
  baseLocked = !baseLocked;
}

void toggleShovelLock() {
  shovelLocked = !shovelLocked;
}

void resetArmLocks() {
  baseLocked   = false;
  armLocked    = false;
  shovelLocked = false;
}

// ================================================================
// ===     Non-Blocking Snap Helper (one step per call)           ===
// ================================================================
// Call every control loop tick. snapTarget = -1 means idle.
static void advanceSnapStep(Servo &srv, int &angle, int &snapTarget, int stepDeg) {
  if (snapTarget < 0) return;
  int step = (snapTarget > angle) ? stepDeg : -stepDeg;
  angle += step;
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  srv.write(angle);
  if ((step > 0 && angle >= snapTarget) || (step < 0 && angle <= snapTarget)) {
    angle = snapTarget;
    srv.write(angle);
    snapTarget = -1;   // done
  }
}

// ================================================================
// ===        Arm Control (Servos 1-5)                            ===
// ================================================================
void processArmControl(ControllerPtr ctl) {

  // ============================================================
  // Servo1 (Base) — R3 lock applies here ONLY
  // ============================================================
  if (!baseLocked) {
    int rxAxis = ctl->axisRX();
    if (abs(rxAxis) > SERVO_DEADZONE) {
      int step = map(abs(rxAxis), SERVO_DEADZONE, 512, 1, SERVO1_MAX_STEP_DEG);
      servo1Angle = constrain(
        servo1Angle + (rxAxis > 0 ? -step : step),
        SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    }
    servo1.write(servo1Angle);
  }

  // ============================================================
  // Servo2
  // ============================================================
  bool l1 = ctl->l1();
  bool r1 = ctl->r1();
  if (r1 && !l1) {
    servo2Angle = constrain(servo2Angle + SERVO2_STEP_DEG, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  } else if (l1 && !r1) {
    servo2Angle = constrain(servo2Angle - SERVO2_STEP_DEG, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  }
  servo2.write(servo2Angle);

  // ============================================================
  // Servo3
  // ============================================================
  uint16_t btns = ctl->buttons();
  bool squareBtn   = btns & 0x0004;
  bool circleBtn   = btns & 0x0002;

  if (squareBtn) {
    servo3Angle = constrain(servo3Angle + SERVO3_STEP_DEG, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  } else if (circleBtn) {
    servo3Angle = constrain(servo3Angle - SERVO3_STEP_DEG, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  }
  servo3.write(servo3Angle);

  // ============================================================
  // Servo4  —  limits: [SEMI_S4_MAX_DOWN .. SEMI_S4_MAX_UP]
  // ============================================================
  bool crossBtn    = btns & 0x0001;
  bool triangleBtn = btns & 0x0008;

  if (triangleBtn) {
    servo4Angle = constrain(servo4Angle + SERVO_STEP_DEG, SEMI_S4_MAX_DOWN, SEMI_S4_MAX_UP);
  } else if (crossBtn) {
    servo4Angle = constrain(servo4Angle - SERVO_STEP_DEG, SEMI_S4_MAX_DOWN, SEMI_S4_MAX_UP);
  }
  servo4.write(servo4Angle);

  // ============================================================
  // Servo5 (Gripper)  —  limits: [SERVO5_CLOSE_ANGLE .. SERVO5_OPEN_ANGLE]
  // ============================================================
  uint8_t dp = ctl->dpad();
  bool dpadRight = dp & 0x04;
  bool dpadDown  = dp & 0x02;

  if (dpadRight) {
    servo5Angle = constrain(servo5Angle + SERVO5_STEP_DEG, SERVO5_CLOSE_ANGLE, SERVO5_OPEN_ANGLE);
  } else if (dpadDown) {
    servo5Angle = constrain(servo5Angle - SERVO5_STEP_DEG, SERVO5_CLOSE_ANGLE, SERVO5_OPEN_ANGLE);
  }
  servo5.write(servo5Angle);
}

// ================================================================
// ===      Shovel Control (Servo 6)                              ===
// ================================================================
void processShovelControl(ControllerPtr ctl) {
  if (shovelLocked) return;

  static int  snapT6  = -1;
  static bool prevCi6 = false, prevSq6 = false;
  uint16_t btns  = ctl->buttons();
  bool circleBtn = btns & 0x0002;
  bool squareBtn = btns & 0x0004;

  if      (circleBtn && !prevCi6) { snapT6 = SEMI_S6_MAX_UP; }
  else if (squareBtn && !prevSq6) { snapT6 = SEMI_S6_MAX_DOWN; }
  prevCi6 = circleBtn;
  prevSq6 = squareBtn;

  // Advance snap one step per call (non-blocking)
  advanceSnapStep(servo6, servo6Angle, snapT6, SEMI_S6_SNAP_STEP);

  // Analog RY — cancels any active snap and gives direct control
  int ryAxis = ctl->axisRY();
  if (abs(ryAxis) > SERVO_DEADZONE) {
    snapT6 = -1;   // cancel snap when analog used
    int step = map(abs(ryAxis), SERVO_DEADZONE, 512, 1, SERVO6_MAX_STEP_DEG);
    servo6Angle = constrain(
      servo6Angle + (ryAxis < 0 ? step : -step),
      SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    servo6.write(servo6Angle);
  }
}

// ================================================================
// ===   Semi-Auto Control (Servos 1-5, one press = home)         ===
// ===   Button → snap servo to its INITIAL angle                 ===
// ================================================================
void processSemiAutoControl(ControllerPtr ctl) {
  if (armLocked) return;

  // Snap targets per servo (-1 = idle)
  static int snapT1=-1, snapT2=-1, snapT3=-1, snapT4=-1, snapT5=-1;

  uint16_t btns  = ctl->buttons();
  uint8_t  dp    = ctl->dpad();
  bool r1        = ctl->r1();
  bool squareBtn = btns & 0x0004;
  bool circleBtn = btns & 0x0002;   // servo1 (base) home
  bool crossBtn  = btns & 0x0001;
  bool dpRight   = dp  & 0x04;

  static bool pR1=false, pSq=false, pCi=false, pCr=false, pDR=false;

  // Rising edge → snap to INITIAL angle
  if (circleBtn && !pCi && !baseLocked) { snapT1 = SERVO1_INITIAL_ANGLE; }
  if (r1        && !pR1)                { snapT2 = SERVO2_INITIAL_ANGLE; }
  if (squareBtn && !pSq)                { snapT3 = SERVO3_INITIAL_ANGLE; }
  if (crossBtn  && !pCr)                { snapT4 = SERVO4_INITIAL_ANGLE; }
  if (dpRight   && !pDR)                { snapT5 = SERVO5_INITIAL_ANGLE; }

  pR1=r1; pSq=squareBtn; pCi=circleBtn; pCr=crossBtn; pDR=dpRight;

  // Advance each servo one step this tick (non-blocking)
  if (!baseLocked) advanceSnapStep(servo1, servo1Angle, snapT1, SEMI_S1_SNAP_STEP);
  advanceSnapStep(servo2, servo2Angle, snapT2, SEMI_S2_SNAP_STEP);
  advanceSnapStep(servo3, servo3Angle, snapT3, SEMI_S3_SNAP_STEP);
  advanceSnapStep(servo4, servo4Angle, snapT4, SEMI_S4_SNAP_STEP);
  advanceSnapStep(servo5, servo5Angle, snapT5, SEMI_S5_SNAP_STEP);
}
