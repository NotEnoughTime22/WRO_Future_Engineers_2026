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
// MaixCAM detection settings
//
// Change class IDs to match your YOLO model labels.
// ------------------------------------------------------------
const int GREEN_ID = 1;
const int RED_ID = 2;

const int MIN_COLOR_AREA = 100;
const float MIN_COLOR_CONFIDENCE = 0.50f;

// MaixCAM image-space limit. Lower image areas represent nearby
// obstacles if camera y increases downward.
const int MAX_COLOR_Y_CENTER = 160;

const float COLOR_AVOID_ANGLE = 45.0f;

// Minimum encoder travel while avoiding the coloured object.
const long COLOR_MIN_OUTWARD_DISTANCE = 500;

// The object must be missing for this long before crossing back.
const unsigned long COLOR_LOST_TIMEOUT_MS = 350;

// Prevent repeat detection of the same object immediately afterward.
const unsigned long COLOR_RETRIGGER_COOLDOWN_MS = 1000;

// ------------------------------------------------------------
// MaixCAM packet sent by:
// struct.pack("<hhHHHf", x, y, w, h, classID, confidence)
//
// Packed ensures the structure has exactly 14 bytes.
// ------------------------------------------------------------
struct __attribute__((packed)) MaixObject {
    int16_t x;
    int16_t y;
    uint16_t width;
    uint16_t height;
    uint16_t classID;
    float confidence;
};

const int MAIX_PACKET_BYTES = sizeof(MaixObject);

// ------------------------------------------------------------
// Colour avoidance state
// ------------------------------------------------------------
enum ColourState {
    COLOR_NONE,
    COLOR_TURN_OUT,
    COLOR_DRIVE_OUT,
    COLOR_TURN_BACK,
    COLOR_DRIVE_BACK,
    COLOR_RETURN_HEADING
};

ColourState colourState = COLOR_NONE;

int colourDirection = 0;       // -1 = green/left, +1 = red/right
float headingBeforeColour = 0.0f;

long colourOutwardStartAngle = 0;
long colourOutwardDistance = 0;
long colourReturnStartAngle = 0;

unsigned long lastColourSeenTime = 0;
unsigned long lastColourActionTime = 0;

// ------------------------------------------------------------
// Heading state
// ------------------------------------------------------------
float targetHeading = 0.0f;
float lastValidHeading = 0.0f;

float lastParkErr = 0.0f;
unsigned long lastParkTime = 0;

// ------------------------------------------------------------
// Heading functions
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
    return readHeadingSafe();
}

// ------------------------------------------------------------
// Steering functions
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
// MaixCAM I2C receiver
//
// IMPORTANT:
// EVOX1 must be configured as an I2C slave at address 0x24
// on an unused I2C bus. Do not share I2C5 because it is already
// used by tofBack.
//
// The missing implementation is library-specific. Insert the
// callback/buffer implementation required by your EVOX1 I2C API.
// ------------------------------------------------------------
bool readMaixObject(MaixObject &obj) {
    // Example intended behaviour:
    //
    // if (maixI2CSlave.available() >= MAIX_PACKET_BYTES) {
    //     maixI2CSlave.readBytes((uint8_t*)&obj, MAIX_PACKET_BYTES);
    //     return true;
    // }
    //
    // return false;

    return false;
}

// ------------------------------------------------------------
// MaixCAM object validation
// ------------------------------------------------------------
bool isValidColourObject(const MaixObject &obj) {
    if (obj.classID != RED_ID && obj.classID != GREEN_ID) {
        return false;
    }

    int objectArea = (int)obj.width * (int)obj.height;

    if (objectArea < MIN_COLOR_AREA) {
        return false;
    }

    if (obj.confidence < MIN_COLOR_CONFIDENCE) {
        return false;
    }

    int yCenter = obj.y + ((int)obj.height / 2);

    if (yCenter > MAX_COLOR_Y_CENTER) {
        return false;
    }

    return true;
}

bool getColourDetection(MaixObject &obj) {
    if (!readMaixObject(obj)) {
        return false;
    }

    return isValidColourObject(obj);
}

// ------------------------------------------------------------
// Colour avoidance
// ------------------------------------------------------------
void startColourAvoidance(const MaixObject &obj) {
    headingBeforeColour = targetHeading;

    if (obj.classID == RED_ID) {
        colourDirection = 1;
    } else {
        colourDirection = -1;
    }

    targetHeading = normalize(
        headingBeforeColour + colourDirection * COLOR_AVOID_ANGLE
    );

    colourState = COLOR_TURN_OUT;

    lastColourSeenTime = millis();
    lastColourActionTime = millis();

    lastParkErr = 0.0f;
    lastParkTime = millis();
}

void updateColourAvoidance(bool colourSeenNow) {
    float heading = readHeadingSafe();
    float err = headingError(heading, targetHeading);

    headingDriveStep();
    driveMotor.run(DRIVE_SPEED);

    if (colourSeenNow) {
        lastColourSeenTime = millis();
    }

    if (colourState == COLOR_TURN_OUT) {
        if (abs(err) < TURN_COMPLETE_ERROR) {
            colourOutwardStartAngle = driveMotor.getAngle();
            colourState = COLOR_DRIVE_OUT;
        }

        return;
    }

    if (colourState == COLOR_DRIVE_OUT) {
        long currentAngle = driveMotor.getAngle();
        colourOutwardDistance =
            labs(currentAngle - colourOutwardStartAngle);

        bool colourLost =
            (millis() - lastColourSeenTime) > COLOR_LOST_TIMEOUT_MS;

        if (colourLost &&
            colourOutwardDistance >= COLOR_MIN_OUTWARD_DISTANCE) {

            targetHeading = normalize(
                targetHeading - colourDirection * 90.0f
            );

            colourState = COLOR_TURN_BACK;

            lastParkErr = 0.0f;
            lastParkTime = millis();
        }

        return;
    }

    if (colourState == COLOR_TURN_BACK) {
        if (abs(err) < TURN_COMPLETE_ERROR) {
            colourReturnStartAngle = driveMotor.getAngle();
            colourState = COLOR_DRIVE_BACK;
        }

        return;
    }

    if (colourState == COLOR_DRIVE_BACK) {
        long currentAngle = driveMotor.getAngle();

        long returnDistance =
            labs(currentAngle - colourReturnStartAngle);

        if (returnDistance >= colourOutwardDistance) {
            targetHeading = headingBeforeColour;
            colourState = COLOR_RETURN_HEADING;

            lastParkErr = 0.0f;
            lastParkTime = millis();
        }

        return;
    }

    if (colourState == COLOR_RETURN_HEADING) {
        if (abs(err) < TURN_COMPLETE_ERROR) {
            colourState = COLOR_NONE;
            colourDirection = 0;
            lastColourActionTime = millis();
        }
    }
}

const char* getStateText() {
    if (colourState == COLOR_TURN_OUT) return "C OUT";
    if (colourState == COLOR_DRIVE_OUT) return "C GO";
    if (colourState == COLOR_TURN_BACK) return "C X";
    if (colourState == COLOR_DRIVE_BACK) return "C RET";
    if (colourState == COLOR_RETURN_HEADING) return "C END";

    return "DRIVE";
}

// ------------------------------------------------------------
// Display
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
void driveAndTurnOnGapsWithColourAvoidance() {
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
        int leftDist = tofLeft.getDistance();
        int rightDist = tofRight.getDistance();
        int frontDist = tofFront.getDistance();
        int backDist = tofBack.getDistance();

        float heading = readHeadingSafe();

        // Read the newest MaixCAM packet, if one arrived.
        MaixObject colourObject;
        bool colourSeen = getColourDetection(colourObject);

        bool canStartColourAvoidance =
            (colourState == COLOR_NONE) &&
            ((millis() - lastColourActionTime) >
             COLOR_RETRIGGER_COOLDOWN_MS);

        if (colourSeen && canStartColourAvoidance) {
            startColourAvoidance(colourObject);
        }

        // Colour behaviour has priority over normal gap turns.
        if (colourState != COLOR_NONE) {
            updateColourAvoidance(colourSeen);

            if (millis() - lastDisplayTime >= 150) {
                showGapStatus(leftDist, rightDist, frontDist, backDist,
                              heading, getStateText());

                lastDisplayTime = millis();
            }

            previousLeftDist = leftDist;
            previousRightDist = rightDist;

            delay(10);
            continue;
        }

        // Normal heading hold while driving straight.
        headingDriveStep();
        driveMotor.run(DRIVE_SPEED);

        bool leftGapNow = leftDist > GAP_THRESHOLD_MM;
        bool rightGapNow = rightDist > GAP_THRESHOLD_MM;

        bool leftOpening =
            previousLeftDist > 0 &&
            leftDist > previousLeftDist + 150;

        bool rightOpening =
            previousRightDist > 0 &&
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

        bool leftGapConfirmed =
            leftGapCount >= GAP_CONFIRM_SAMPLES;

        bool rightGapConfirmed =
            rightGapCount >= GAP_CONFIRM_SAMPLES;

        bool cooldownDone =
            (millis() - lastTurnTime) > TURN_COOLDOWN_MS;

        if (cooldownDone &&
            (leftGapConfirmed || rightGapConfirmed)) {

            bool turnLeft = false;

            if (leftGapConfirmed && !rightGapConfirmed) {
                turnLeft = true;
            } else if (rightGapConfirmed && !leftGapConfirmed) {
                turnLeft = false;
            } else {
                turnLeft =
                    leftDist > rightDist + GAP_DIRECTION_MARGIN_MM;
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

            // Complete the 90-degree gap turn.
            while (true) {
                int turnFrontDist = tofFront.getDistance();

                float turnHeading = readHeadingSafe();
                float turnError =
                    headingError(turnHeading, targetHeading);

                headingDriveStep();
                driveMotor.run(DRIVE_SPEED);

                if (abs(turnError) < TURN_COMPLETE_ERROR) {
                    break;
                }

                if (turnFrontDist > 0 &&
                    turnFrontDist < FRONT_EMERGENCY_MM) {

                    driveMotor.brake();
                    delay(100);
                    driveMotor.run(DRIVE_SPEED);
                }

                delay(10);
            }

            // Move away from the intersection before detecting
            // another side gap.
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

    if (!isnan(x) && !isinf(x) &&
        x >= 0.0f && x < 360.0f) {

        lastValidHeading = normalize(x);
    } else {
        lastValidHeading = 0.0f;
    }

    targetHeading = snapHeading45(lastValidHeading);

    driveMotor.resetAngle();

    evo.clearDisplay();
    evo.writeLineToDisplay("Gap + colour mode", 0, true, true);
    evo.writeLineToDisplay("Starting...", 1, true, false);
    evo.drawDisplay();

    delay(500);

    driveMotor.runAngle(4000, 500);

    driveAndTurnOnGapsWithColourAvoidance();
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------
void loop() {
    // Navigation runs forever in setup().
}
