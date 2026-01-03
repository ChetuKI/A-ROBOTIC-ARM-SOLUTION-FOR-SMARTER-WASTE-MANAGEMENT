// 6-DOF Pick & Place (PCA9685) — Full sketch (Serial command input only)
// Send one of: "metal\n"  "glass\n"  "paper\n"
// Requires: Adafruit PWM Servo Driver library

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ---------- CHANNEL DEFINITIONS ----------
const uint8_t CH_BASE        = 0;
const uint8_t CH_SHOULDER    = 1;
const uint8_t CH_ELBOW       = 2;
const uint8_t CH_WRIST_PITCH = 3;
const uint8_t CH_WRIST_ROLL  = 4;
const uint8_t CH_GRIPPER     = 5;

// ---------- SERVO PULSE CALIBRATION (adjust for your servos) ----------
int SERVOMIN = 150;  // ~500µs
int SERVOMAX = 600;  // ~2500µs

// ---------- MOTION / TIMING ----------
const uint16_t PWM_FREQ = 50;     // 50Hz for servos
int stepDelay = 6;                // ms between micro-steps (smoothness)
int gripDelay = 600;              // time to wait when closing/opening gripper (ms)
int holdDelay = 1600;             // unused for main loop but kept for tuning

// ---------- POSES (angles in degrees 0..180) ----------
// HOME / HOLD (starting position) - tweak as needed
int hold_base        = 0;
int hold_shoulder    = 20;
int hold_elbow       = 20;
int hold_wrist_pitch = 0;
int hold_wrist_roll  = 40;
int hold_gripper    = 0;   // slightly closed

// PICK pose (where the gripper picks object)
int pick_base        = 0;
int pick_shoulder    = 120;
int pick_elbow       = 70;
int pick_wrist_pitch = 0;
int pick_wrist_roll  = 40;
int pick_gripper_open  = 0;
int pick_gripper_close = 70;

// LIFT pose (lift object straight up)
int lift_base        = 0;   // typically same as pick_base or tuned
int lift_shoulder    = 60;
int lift_elbow       = 40;
int lift_wrist_pitch = 25;
int lift_wrist_roll  = 38;

// Default place pose variables (will be set by category)
int place_base        = 0;
int place_shoulder    = 110;
int place_elbow       = 0;
int place_wrist_pitch = 5;
int place_wrist_roll  = 38;
int place_gripper_open_local  = 20;   // same as pick
int place_gripper_close_local = 70;

// ---------- THREE PLACE POSES (tweak these to match your bins) ----------
/* Metal place pose */
const int metal_place_base        = 95;
const int metal_place_shoulder    = 80;
const int metal_place_elbow       = 30;
const int metal_place_wrist_pitch = 10;
const int metal_place_wrist_roll  = 40;

/* Glass place pose */
const int glass_place_base        = 70;
const int glass_place_shoulder    = 80;
const int glass_place_elbow       = 30;
const int glass_place_wrist_pitch = 10;
const int glass_place_wrist_roll  = 40;

/* Paper place pose */
const int paper_place_base        = 40;
const int paper_place_shoulder    = 80;
const int paper_place_elbow       = 30;
const int paper_place_wrist_pitch = 10;
const int paper_place_wrist_roll  = 40;

// ---------- CATEGORY ENUM ----------
enum Category { CAT_NONE = 0, CAT_METAL, CAT_GLASS, CAT_PAPER };
Category currentCategory = CAT_NONE;

// ---------- UTILITY: convert angle (0..180) to PCA9685 ticks (0..4095) ----------
uint16_t angleToTicks(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  long ticks = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)ticks;
}

// ---------- MOVE: interpolate all joints simultaneously (linear) ----------
void moveToPose(int baseA, int shoulderA, int elbowA, int wristPitchA, int wristRollA, int gripperA, int delayMs) {
  static int cur_base        = -1;
  static int cur_shoulder    = -1;
  static int cur_elbow       = -1;
  static int cur_wrist_pitch = -1;
  static int cur_wrist_roll  = -1;
  static int cur_gripper     = -1;

  if (cur_base == -1) {
    cur_base        = hold_base;
    cur_shoulder    = hold_shoulder;
    cur_elbow       = hold_elbow;
    cur_wrist_pitch = hold_wrist_pitch;
    cur_wrist_roll  = hold_wrist_roll;
    cur_gripper     = hold_gripper;
  }

  int db = abs(baseA - cur_base);
  int ds = abs(shoulderA - cur_shoulder);
  int de = abs(elbowA - cur_elbow);
  int dwp = abs(wristPitchA - cur_wrist_pitch);
  int dwr = abs(wristRollA - cur_wrist_roll);
  int dg = abs(gripperA - cur_gripper);

  int steps = max(max(max(db, ds), max(de, max(dwp, dwr))), dg);
  if (steps == 0) return;

  for (int s = 1; s <= steps; s++) {
    int nb = cur_base + ((baseA - cur_base) * s) / steps;
    int ns = cur_shoulder + ((shoulderA - cur_shoulder) * s) / steps;
    int ne = cur_elbow + ((elbowA - cur_elbow) * s) / steps;
    int nwp = cur_wrist_pitch + ((wristPitchA - cur_wrist_pitch) * s) / steps;
    int nwr = cur_wrist_roll + ((wristRollA - cur_wrist_roll) * s) / steps;
    int ng = cur_gripper + ((gripperA - cur_gripper) * s) / steps;

    pwm.setPWM(CH_BASE,        0, angleToTicks(nb));
    pwm.setPWM(CH_SHOULDER,    0, angleToTicks(ns));
    pwm.setPWM(CH_ELBOW,       0, angleToTicks(ne));
    pwm.setPWM(CH_WRIST_PITCH, 0, angleToTicks(nwp));
    pwm.setPWM(CH_WRIST_ROLL,  0, angleToTicks(nwr));
    pwm.setPWM(CH_GRIPPER,     0, angleToTicks(ng));

    delay(delayMs);
  }

  cur_base        = baseA;
  cur_shoulder    = shoulderA;
  cur_elbow       = elbowA;
  cur_wrist_pitch = wristPitchA;
  cur_wrist_roll  = wristRollA;
  cur_gripper     = gripperA;
}

// ---------- APPLY HOLD POSE IMMEDIATELY ----------
void applyHoldPose() {
  pwm.setPWM(CH_BASE,        0, angleToTicks(hold_base));
  pwm.setPWM(CH_SHOULDER,    0, angleToTicks(hold_shoulder));
  pwm.setPWM(CH_ELBOW,       0, angleToTicks(hold_elbow));
  pwm.setPWM(CH_WRIST_PITCH, 0, angleToTicks(hold_wrist_pitch));
  pwm.setPWM(CH_WRIST_ROLL,  0, angleToTicks(hold_wrist_roll));
  pwm.setPWM(CH_GRIPPER,     0, angleToTicks(hold_gripper));
}

// ---------- SET CURRENT PLACE POSE BASED ON CATEGORY ----------
void setPlacePoseForCategory(Category cat) {
  if (cat == CAT_METAL) {
    place_base = metal_place_base;
    place_shoulder = metal_place_shoulder;
    place_elbow = metal_place_elbow;
    place_wrist_pitch = metal_place_wrist_pitch;
    place_wrist_roll = metal_place_wrist_roll;
  } else if (cat == CAT_GLASS) {
    place_base = glass_place_base;
    place_shoulder = glass_place_shoulder;
    place_elbow = glass_place_elbow;
    place_wrist_pitch = glass_place_wrist_pitch;
    place_wrist_roll = glass_place_wrist_roll;
  } else if (cat == CAT_PAPER) {
    place_base = paper_place_base;
    place_shoulder = paper_place_shoulder;
    place_elbow = paper_place_elbow;
    place_wrist_pitch = paper_place_wrist_pitch;
    place_wrist_roll = paper_place_wrist_roll;
  }
}

// ---------- PICK & PLACE SEQUENCE (uses current place_* variables) ----------
void pickAndPlaceCycle() {
  // 1) Approach: lift posture above pick spot
  moveToPose(pick_base, lift_shoulder, lift_elbow, lift_wrist_pitch, lift_wrist_roll, pick_gripper_open, stepDelay);

  // 2) Lower to pick pose and orient wrist
  moveToPose(pick_base, pick_shoulder, pick_elbow, pick_wrist_pitch, pick_wrist_roll, pick_gripper_open, stepDelay);

  // 3) Ensure open gripper
  moveToPose(pick_base, pick_shoulder, pick_elbow, pick_wrist_pitch, pick_wrist_roll, pick_gripper_open, stepDelay);
  delay(150);

  // 4) Close gripper (grab)
  moveToPose(pick_base, pick_shoulder, pick_elbow, pick_wrist_pitch, pick_wrist_roll, pick_gripper_close, stepDelay);
  delay(gripDelay);

  // 5) Lift object
  moveToPose(lift_base, lift_shoulder, lift_elbow, lift_wrist_pitch, lift_wrist_roll, pick_gripper_close, stepDelay);

  // 6) Move to place (keeping lift pose)
  moveToPose(place_base, lift_shoulder, lift_elbow, lift_wrist_pitch, lift_wrist_roll, pick_gripper_close, stepDelay);

  // 7) Lower to place pose and orient wrist for placing
  moveToPose(place_base, place_shoulder, place_elbow, place_wrist_pitch, place_wrist_roll, pick_gripper_close, stepDelay);
  delay(150);

  // 8) Open gripper to release
  moveToPose(place_base, place_shoulder, place_elbow, place_wrist_pitch, place_wrist_roll, pick_gripper_open, stepDelay);
  delay(gripDelay);

  // 9) Retract back to lift pose
  moveToPose(place_base, lift_shoulder, lift_elbow, lift_wrist_pitch, lift_wrist_roll, pick_gripper_open, stepDelay);

  // 10) Return to hold/home pose
  moveToPose(hold_base, hold_shoulder, hold_elbow, hold_wrist_pitch, hold_wrist_roll, hold_gripper, stepDelay);
}

// ---------- SETUP & LOOP ----------
void setup() {
  Serial.begin(9600);
  delay(50); // let Serial initialize
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(200);

  // apply hold pose immediately so servos hold steady on power-up
  applyHoldPose();
  delay(400);

  Serial.println("PCA9685 pick/place ready. Send: metal | glass | paper");
}

void loop() {
  // Only act when serial input is available (serial-only mode)
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // read until newline
    input.trim(); // remove whitespace and \r
    
    // convert to lowercase
    for (uint16_t i = 0; i < input.length(); i++) input[i] = tolower(input[i]);

    if (input.length() == 0) {
      // ignore empty lines
    } else {
      Serial.print("Received: ");
      Serial.println(input);

      if (input == "metal") {
        currentCategory = CAT_METAL;
      } else if (input == "glass") {
        currentCategory = CAT_GLASS;
      } else if (input == "paper") {
        currentCategory = CAT_PAPER;
      } else {
        currentCategory = CAT_NONE;
      }

      if (currentCategory != CAT_NONE) {
        setPlacePoseForCategory(currentCategory);
        pickAndPlaceCycle();
        Serial.println("Cycle complete. Back to hold.");
      } else {
        Serial.println("Invalid command. Use: metal | glass | paper");
      }
    }
  }

  // small pause to be cooperative with other tasks
  delay(10);
}