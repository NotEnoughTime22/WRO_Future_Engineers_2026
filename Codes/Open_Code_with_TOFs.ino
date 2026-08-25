#include "Evo.h"

// ------------------------------------------------------------
// Hardware setup
// ------------------------------------------------------------
EVOX1 evo;

EvoIMU bno(I2C1);

EvoTOF tofRight(I2C2);
EvoTOF tofBack(I2C5);
EvoTOF tofLeft(I2C3);
EvoTOF tofFront(I2C4);

EvoMotor driveMotor(M1, EVOMotor300, true);

EvoServo steerServo(SERVO1, GeekServo360Grey);

// ------------------------------------------------------------
// Motion settings
// ------------------------------------------------------------
const int DRIVE_SPEED = 3000;

const float SERVO_RATIO = 2.33f;
const int SERVO_CENTER = 180;
const int MAX_WHEEL_ANGLE = 35;

const float KP_HEADING = 1.4f;
const float KD_HEADING = 0.0f;

// ------------------------------------------------------------
// Gap detection settings
// ------------------------------------------------------------
const int GAP_THRESHOLD_MM = 500;
const int GAP_CONFIRM_SAMPLES = 4;
const int GAP_DIRECTION_MARGIN_MM = 100;

const long POST_TURN_DISTANCE = 1000;
const unsigned long TURN_COOLDOWN_MS = 700;
const int TURN_COMPLETE_ERROR = 3;
const int FRONT_EMERGENCY_MM = 80;

// ------------------------------------------------------------
// IMU and heading state
// ------------------------------------------------------------
float targetHeading = 0.0f;
float lastValidHeading = 0.0f;

float lastParkErr = 0.0f;
unsigned long lastParkTime = 0;

// ------------------------------------------------------------
// Heading utilities
// ------------------------------------------------------------
float normalize(float a) {
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

float headingError(float current, float target) {
    float e = target - current;

    if (e > 180.0f) e -= 360.0f;
    if (e < -180.0f) e += 360.0f;

    return e;
}

float snapHeading45(float a) {
    a = normalize(a);
    int k = (int)((a + 22.5f) / 45.0f);
    return normalize((float)(k * 45));
}

// ------------------------------------------------------------
// IMU reading
// ------------------------------------------------------------
float readHeadingSafe() {
    float x, y, z;

    bno.getEuler(&x, &y, &z);

    if (!isnan(x) && !isinf(x) && x >= 0.0f && x < 360.0f) {
        lastValidHeading = normalize(x);
        return lastValidHeading;
    }

    delay(2);

    bno.getEuler(&x, &y, &z);

    if (!isnan(x) && !isinf(x) && x >= 0.0f && x < 360.0f) {
        lastValidHeading = normalize(x);
        return lastValidHeading;
    }

    return lastValidHeading;
}

float readHeadingSafePark() {
    float x, y, z;

    bno.getEuler(&x, &y, &z);

    if (!isnan(x) && !isinf(x) && x >= 0.0f && x < 360.0f) {
        lastValidHeading = normalize(x);
        return lastValidHeading;
    }

    return lastValidHeading;
}

// ------------------------------------------------------------
// Steering control
// ------------------------------------------------------------
void applySteering(float wheelAngle) {
    wheelAngle = constrain(wheelAngle, -MAX_WHEEL_ANGLE, MAX_WHEEL_ANGLE);

    float servoCmd = SERVO_CENTER + wheelAngle * SERVO_RATIO;
    servoCmd = constrain(servoCmd, 0, 360);

    steerServo.write((int)servoCmd);
}

void headingDriveStep() {
    float heading = readHeadingSafePark();
    float err = headingError(heading, targetHeading);

    unsigned long now = millis();
    float dt = (now - lastParkTime) * 0.001f;

    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    float dErr = (err - lastParkErr) / dt;

    float wheelAngle = -(KP_HEADING * err + KD_HEADING * dErr);

    applySteering(wheelAngle);

    lastParkErr = err;
    lastParkTime = now;
}

// ------------------------------------------------------------
// Display: raw ToF readings
// ------------------------------------------------------------
void showGapStatus(int leftDist, int rightDist,
                   int frontDist, int backDist,
                   float heading, const char* state) {
    evo.clearDisplay();

    evo.writeToDisplay("L:", 0, 0);
    evo.writeToDisplay(leftDist, 15, 0);

    evo.writeToDisplay("R:", 62, 0);
    evo.writeToDisplay(rightDist, 77, 0);

    evo.writeToDisplay("F:", 0, 16);
    evo.writeToDisplay(frontDist, 15, 16);

    evo.writeToDisplay("B:", 62, 16);
    evo.writeToDisplay(backDist, 77, 16);

    evo.writeToDisplay("H:", 0, 32);
    evo.writeToDisplay((int)heading, 15, 32);

    evo.writeToDisplay("T:", 62, 32);
    evo.writeToDisplay((int)targetHeading, 77, 32);

    evo.writeToDisplay("S:", 0, 48);
    evo.writeToDisplay(state, 15, 48);

    evo.drawDisplay();
}

// ------------------------------------------------------------
// Main navigation function
// ------------------------------------------------------------
void driveAndTurnOnGaps() {
    targetHeading = snapHeading45(readHeadingSafe());

    int leftGapCount = 0;
    int rightGapCount = 0;

    int previousLeftDist = 0;
    int previousRightDist = 0;

    unsigned long lastTurnTime = 0;
    unsigned long lastDisplayTime = 0;

    lastParkErr = 0.0f;
    lastParkTime = millis();

    driveMotor.run(DRIVE_SPEED);

    while (true) {
        // Raw, unfiltered ToF readings.
        int leftDist = tofLeft.getDistance();
        int rightDist = tofRight.getDistance();
        int frontDist = tofFront.getDistance();
        int backDist = tofBack.getDistance();

        float heading = readHeadingSafe();

        headingDriveStep();

        bool leftGapNow = leftDist > GAP_THRESHOLD_MM;
        bool rightGapNow = rightDist > GAP_THRESHOLD_MM;

        // A sudden jump outward can indicate the leading edge of a gap.
        bool leftOpening = previousLeftDist > 0 &&
                           leftDist > previousLeftDist + 150;

        bool rightOpening = previousRightDist > 0 &&
                            rightDist > previousRightDist + 150;

        if (leftGapNow || leftOpening) {
            leftGapCount++;
        } else {
            leftGapCount = 0;
        }

        if (rightGapNow || rightOpening) {
            rightGapCount++;
        } else {
            rightGapCount = 0;
        }

        bool leftGapConfirmed = leftGapCount >= GAP_CONFIRM_SAMPLES;
        bool rightGapConfirmed = rightGapCount >= GAP_CONFIRM_SAMPLES;

        bool cooldownDone = (millis() - lastTurnTime) > TURN_COOLDOWN_MS;

        // ----------------------------------------------------
        // Detect a gap and set a new target heading.
        // ----------------------------------------------------
        if (cooldownDone && (leftGapConfirmed || rightGapConfirmed)) {
            bool turnLeft = false;

            if (leftGapConfirmed && !rightGapConfirmed) {
                turnLeft = true;
            } else if (rightGapConfirmed && !leftGapConfirmed) {
                turnLeft = false;
            } else {
                turnLeft = leftDist > rightDist + GAP_DIRECTION_MARGIN_MM;
            }

            if (turnLeft) {
                targetHeading = snapHeading45(targetHeading - 90.0f);
            } else {
                targetHeading = snapHeading45(targetHeading + 90.0f);
            }

            leftGapCount = 0;
            rightGapCount = 0;

            lastTurnTime = millis();
            lastParkErr = 0.0f;
            lastParkTime = millis();

            // Turn continuously until the IMU reaches the new heading.
            while (true) {
                int turnFrontDist = tofFront.getDistance();

                float turnHeading = readHeadingSafe();
                float turnError = headingError(turnHeading, targetHeading);

                headingDriveStep();
                driveMotor.run(DRIVE_SPEED);

                if (abs(turnError) < TURN_COMPLETE_ERROR) {
                    break;
                }

                if (turnFrontDist > 0 && turnFrontDist < FRONT_EMERGENCY_MM) {
                    driveMotor.brake();
                    delay(100);
                    driveMotor.run(DRIVE_SPEED);
                }

                delay(10);
            }

            // Travel forward after turning to leave the gap/intersection.
            long clearStartAngle = driveMotor.getAngle();

            while (labs(driveMotor.getAngle() - clearStartAngle) <
                   POST_TURN_DISTANCE) {

                headingDriveStep();
                driveMotor.run(DRIVE_SPEED);

                delay(10);
            }

            leftGapCount = 0;
            rightGapCount = 0;
        }

        if (millis() - lastDisplayTime >= 150) {
            showGapStatus(leftDist, rightDist, frontDist, backDist,
                          heading, "DRIVE");

            lastDisplayTime = millis();
        }

        previousLeftDist = leftDist;
        previousRightDist = rightDist;

        delay(10);
    }
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
    evo.begin();

    tofRight.begin();
    tofBack.begin();
    tofLeft.begin();
    tofFront.begin();

    bno.begin();

    driveMotor.begin();
    steerServo.begin();

    steerServo.write(SERVO_CENTER);

    evo.clearDisplay();
    evo.writeLineToDisplay("Init IMU...", 0, true, true);
    evo.drawDisplay();

    evo.playTone(1000, 10, false);
    delay(1000);

    float x, y, z;
    bno.getEuler(&x, &y, &z);

    if (!isnan(x) && !isinf(x) && x >= 0.0f && x < 360.0f) {
        lastValidHeading = normalize(x);
    } else {
        lastValidHeading = 0.0f;
    }

    targetHeading = snapHeading45(lastValidHeading);

    driveMotor.resetAngle();

    evo.clearDisplay();
    evo.writeLineToDisplay("Raw ToF gap mode", 0, true, true);
    evo.writeLineToDisplay("Starting...", 1, true, false);
    evo.drawDisplay();

    delay(500);

    driveMotor.runAngle(4000, 500);

    driveAndTurnOnGaps();
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------
void loop() {
    // driveAndTurnOnGaps() runs forever from setup().
}
