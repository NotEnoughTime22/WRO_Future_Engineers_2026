#include "Evo.h"


// Hardware setup
EVOX1 evo;


// IMU on I2C1, TOF on I2C. right sensor on i2c port 2, back sensor on port 5, left sensor on port 3, front sensor on port 4
EvoIMU bno(I2C1);
EvoTOF tofRight(I2C2);
EvoTOF tofBack(I2C5);
EvoTOF tofLeft(I2C3);
EvoTOF tofFront(I2C4);
// Motor on M1
EvoMotor driveMotor(M1, EVOMotor300, true);


// Servo on S1
EvoServo steerServo(SERVO1, GeekServo360Grey);



// Motion settings
const long STRAIGHT_DEGREES = 6000; // encoder degrees per edge
const int DRIVE_SPEED = 3000; // constant drive speed


const float SERVO_RATIO = 2.33f; // 2.33° servo = 1° wheel
const int SERVO_CENTER = 180;
const int MAX_WHEEL_ANGLE = 35; // max wheel angle


// Heading control (PD)
const float KP_HEADING = 1.7f; // proportional gain
const float KD_HEADING = 0.0f; // derivative gain


float targetHeading = 0.0f; // desired absolute heading


// IMU state to keep heading valid
float lastValidHeading = 0.0f;
// ---------- Gap navigation settings ----------

const int GAP_THRESHOLD_MM = 650;
const int GAP_CONFIRM_SAMPLES = 4;
const int GAP_DIRECTION_MARGIN_MM = 100;

const long POST_TURN_DISTANCE = 1000;
const unsigned long TURN_COOLDOWN_MS = 700;
const int TURN_COMPLETE_ERROR = 8;

struct ToFSmoother {
    static const int SIZE = 5;

    int samples[SIZE];
    int index;
    int count;
    int lastGood;

    ToFSmoother() : index(0), count(0), lastGood(4000) {}

    int update(int raw) {
        int value = cleanTof(raw, lastGood);
        lastGood = value;

        samples[index] = value;
        index = (index + 1) % SIZE;

        if (count < SIZE) count++;

        long sum = 0;
        for (int i = 0; i < count; i++) {
            sum += samples[i];
        }

        return (int)(sum / count);
    }
};

ToFSmoother leftGapFilter;
ToFSmoother rightGapFilter;
ToFSmoother frontGapFilter;


// Keeps the car pointed at targetHeading while driving.
void headingDriveStep() {
    float heading = readHeadingSafePark();
    float err = headingError(heading, targetHeading);

    unsigned long now = millis();
    float dt = (now - lastParkTime) * 0.001f;

    if (dt <= 0.0f) dt = 0.001f;

    float dErr = (err - lastParkErr) / dt;

    float wheelAngle = -(KP_HEADING * err + KD_HEADING * dErr);

    applySteering(wheelAngle);

    lastParkErr = err;
    lastParkTime = now;
}

// ---------- Utility ----------


// Keep existing normalize and headingError


float normalize(float a) {
    while (a < 0) a += 360;
    while (a >= 360) a -= 360;
    return a;
}


float headingError(float current, float target) {
    float e = target - current;
    if (e > 180) e -= 360;
    if (e < -180) e += 360;
    return e;
}


// NEW: snap to nearest 45 degrees to avoid off readings
float snapHeading45(float a) {
    a = normalize(a);
    // Add 22.5 then integer divide by 45 to round to nearest multiple
    int k = (int)((a + 22.5f) / 45.0f); // 0..7
    return (float)(k * 45); // 0,45,...,315
}


// Safer heading read: retries once, falls back to last good value
float readHeadingSafe() {
    float x, y, z;


    bno.getEuler(&x, &y, &z); // first try
    if (!isnan(x) && !isinf(x) && x >= 0 && x < 360) {
        lastValidHeading = normalize(x);
        return lastValidHeading;
    }
  

    // One retry
    delay(2);
    bno.getEuler(&x, &y, &z);
    if (!isnan(x) && !isinf(x) && x >= 0 && x < 360) {
        lastValidHeading = normalize(x);
        return lastValidHeading;
    }


    // Fall back: return last valid heading to avoid crazy jumps
    return lastValidHeading;
}


void printStatus(const char* line0, const char* line1, float heading = -1000.0f) {
    evo.clearDisplay();
    evo.writeLineToDisplay(line0, 0, true, true);
    evo.writeLineToDisplay(line1, 1, true, false);
    if (heading > -999.0f) {
        evo.writeToDisplay("H:", 0, 32);
        evo.writeToDisplay((int)heading, 20, 32);
        evo.writeToDisplay(" T:", 60, 32);
        evo.writeToDisplay((int)targetHeading, 85, 32);
    }
    evo.drawDisplay();
}


void applySteering(float wheelAngle) {
    wheelAngle = constrain(wheelAngle, -MAX_WHEEL_ANGLE, MAX_WHEEL_ANGLE);


    float servoOffset = wheelAngle * SERVO_RATIO; // wheel -> servo
    float servoCmd = SERVO_CENTER + servoOffset;


    servoCmd = constrain(servoCmd, 0, 360); // GeekServo 360 range


    steerServo.write((int)servoCmd);
}


int getFrontDist() { return tofFront.getDistance(); }
int getBackDist() { return tofBack.getDistance(); }
int getLeftDist() { return tofLeft.getDistance(); }
int getRightDist() { return tofRight.getDistance(); }


// PD steering based on IMU 
float lastParkErr = 0.0f;
unsigned long lastParkTime = 0;


float readHeadingSafePark() {
    float x, y, z;
    bno.getEuler(&x, &y, &z);
    if (!isnan(x) && !isinf(x) && x >= 0 && x < 360)
    return normalize(x);
    return targetHeading; // fall back
}


// Drive with heading PD for a signed encoder distance
void headingTrack(long degrees, float targetHeadingIn) {
    // snap the target given to this function
    float targetHeading = snapHeading45(targetHeadingIn);


    long startPos = driveMotor.getAngle();
    int dir = (degrees >= 0) ? 1 : -1;
    long targetDelta = labs(degrees);


    driveMotor.run(dir * DRIVE_SPEED);


    lastParkErr = 0.0f;
    lastParkTime = millis();


    while (true) {
        long currentPos = driveMotor.getAngle();
        long delta = labs(currentPos - startPos);
        if (delta >= targetDelta) break;


        float heading = readHeadingSafePark();
        float err = headingError(heading, targetHeading);


        unsigned long now = millis();
        float dt = (now - lastParkTime) * 0.001f;
        if (dt <= 0.0f) dt = 0.001f;
        float dErr = (err - lastParkErr) / dt;


        float wheelAngle = -(KP_HEADING * err + KD_HEADING * dErr);
        applySteering(wheelAngle);


        lastParkErr = err;
        lastParkTime = now;


        delay(10);
    }


    driveMotor.brake();
}


void headingTrackCoast(long degrees, float targetHeadingIn) {
    // snap the target given to this function
    float targetHeading = snapHeading45(targetHeadingIn);


    long startPos = driveMotor.getAngle();
    int dir = (degrees >= 0) ? 1 : -1;
    long targetDelta = labs(degrees);


    driveMotor.run(dir * DRIVE_SPEED);


    lastParkErr = 0.0f;
    lastParkTime = millis();


    while (true) {
        long currentPos = driveMotor.getAngle();
        long delta = labs(currentPos - startPos);
        if (delta >= targetDelta) break;


        float heading = readHeadingSafePark();
        float err = headingError(heading, targetHeading);


        unsigned long now = millis();
        float dt = (now - lastParkTime) * 0.001f;
        if (dt <= 0.0f) dt = 0.001f;
        float dErr = (err - lastParkErr) / dt;


        float wheelAngle = -(KP_HEADING * err + KD_HEADING * dErr);
        applySteering(wheelAngle);


        lastParkErr = err;
        lastParkTime = now;


        delay(10);
    }


    driveMotor.coast();
}


// Simple clean TOF read with clamp (mm)
int cleanTof(int raw, int lastGood) {
    if (raw <= 0) return lastGood;
    if (raw > 4000) return 4000;
    return raw;
}


void parallelParkWithTOF() {
    int frontdist = tofFront.getDistance();
    int backdist = tofBack.getDistance();
    int leftdist = tofLeft.getDistance();
    long rightdist = tofRight.getDistance();


    if (rightdist > 100){
        // too far from wall, heading track at 45 degree offset
        targetHeading = snapHeading45(readHeadingSafe() + 45.0f);
        while (0.7071067812 * rightdist > 100) {
            headingTrackCoast(10, targetHeading); // small steps
            rightdist = cleanTof(tofRight.getDistance(), rightdist);
        }
    }


    // now we are close to the wall, turn parallel and track toward the perpendicular wall until we are close enough
    targetHeading = snapHeading45(readHeadingSafe() - 45.0f);
    while(frontdist > 200) {
        headingTrackCoast(10, targetHeading); // small steps
        frontdist = cleanTof(tofFront.getDistance(), frontdist);
    }


    // now turn 45 degrees away from wall and track for set distance.
    targetHeading = snapHeading45(readHeadingSafe() - 45.0f);
    headingTrack(1000, targetHeading);


    targetHeading = snapHeading45(readHeadingSafe() + 45.0f);
    headingTrack(100, targetHeading); // short bit to straighten out


    targetHeading = snapHeading45(readHeadingSafe() + 45.0f);
    while (frontdist > 100) {
        headingTrackCoast(10, targetHeading); // small steps
        frontdist = cleanTof(tofFront.getDistance(), frontdist);
    }


    targetHeading = snapHeading45(readHeadingSafe() - 45.0f);
    headingTrack(200, targetHeading); // final bit to straighten out


    evo.writeLineToDisplay("Parked!", 0, true, true);
    evo.drawDisplay();
}


void squareWithPDSteering(int sides) {
    // Initial heading snapped to 45°
    targetHeading = snapHeading45(lastValidHeading);


    int currentEdge = 0;
    long edgeStartPos = driveMotor.getAngle();


    driveMotor.run(DRIVE_SPEED); // constant 4000 the whole time


    unsigned long lastPrint = 0;


    // PD state
    float lastError = 0.0f;
    unsigned long lastTime = millis();


    while (currentEdge < sides) {
        long currentPos = driveMotor.getAngle();
        long delta = labs(currentPos - edgeStartPos);


        // Check if this edge distance is done
        if (delta >= STRAIGHT_DEGREES) {
            currentEdge++;
            if (currentEdge >= sides) break; // square done


            // Add 90° to the desired heading and snap to 45° increments
            targetHeading = snapHeading45(targetHeading + 90.0f);


            // Start measuring distance for the next edge
            edgeStartPos = currentPos;


            // Reset derivative term when we change the target
            lastError = 0.0f;
            lastTime = millis();
        }


        // Continuous heading tracking with PD
        float heading = readHeadingSafe();
        float err = headingError(heading, targetHeading);


        unsigned long now = millis();
        float dt = (now - lastTime) * 0.001f; // seconds
        if (dt <= 0.0f) dt = 0.001f;


        float dErr = (err - lastError) / dt; // deg/s


        float wheelAngle = -(KP_HEADING * err + KD_HEADING * dErr);


        applySteering(wheelAngle);


        lastError = err;
        lastTime = now;


        // Status every ~900 ms
        if (now - lastPrint > 900) {
            char line0[24];
            sprintf(line0, "Edge %d", currentEdge + 1);
            char line1[30];
            sprintf(line1, "H=%.0f T=%.0f", heading, targetHeading);
            printStatus(line0, line1, heading);
            lastPrint = now;
        }


        delay(10);
    }


    driveMotor.brake();
    steerServo.write(SERVO_CENTER);
}


void driveAndTurnOnGaps() {
    // Start by holding the heading at which the robot was placed.
    targetHeading = snapHeading45(readHeadingSafe());

    int leftGapCount = 0;
    int rightGapCount = 0;

    int previousLeftDist = 0;
    int previousRightDist = 0;

    unsigned long lastTurnTime = 0;

    lastParkErr = 0.0f;
    lastParkTime = millis();

    driveMotor.run(DRIVE_SPEED);

    while (true) {
        // Read and smooth the sensors.
        int leftDist = leftGapFilter.update(tofLeft.getDistance());
        int rightDist = rightGapFilter.update(tofRight.getDistance());
        int frontDist = frontGapFilter.update(tofFront.getDistance());

        // Keep the vehicle travelling on targetHeading.
        headingDriveStep();

        // A gap can be detected by a far reading or a sudden increase.
        bool leftGapNow = leftDist > GAP_THRESHOLD_MM;
        bool rightGapNow = rightDist > GAP_THRESHOLD_MM;

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

        bool cooldownDone = millis() - lastTurnTime > TURN_COOLDOWN_MS;

        // Turn toward the confirmed gap.
        if (cooldownDone && (leftGapConfirmed || rightGapConfirmed)) {
            bool turnLeft = false;

            if (leftGapConfirmed && !rightGapConfirmed) {
                turnLeft = true;
            } else if (!leftGapConfirmed && rightGapConfirmed) {
                turnLeft = false;
            } else {
                // Both sides see an opening: use the larger clearance.
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

            // Drive continuously and use IMU heading control until
            // the vehicle has completed the 90-degree turn.
            while (true) {
                float heading = readHeadingSafePark();
                float err = headingError(heading, targetHeading);

                headingDriveStep();

                int turnFrontDist = frontGapFilter.update(tofFront.getDistance());

                // Heading is close enough to the requested 90-degree direction.
                if (abs(err) < TURN_COMPLETE_ERROR) {
                    break;
                }

                // Optional safety stop if about to hit a front wall.
                if (turnFrontDist > 0 && turnFrontDist < 80) {
                    driveMotor.brake();
                    delay(150);
                    driveMotor.run(DRIVE_SPEED);
                }

                delay(10);
            }

            // Continue forward after turning so that the same intersection
            // cannot immediately trigger another side-gap turn.
            long clearStartAngle = driveMotor.getAngle();

            while (labs(driveMotor.getAngle() - clearStartAngle) <
                   POST_TURN_DISTANCE) {

                headingDriveStep();

                delay(10);
            }

            // Reset confirmation counts after leaving the junction.
            leftGapCount = 0;
            rightGapCount = 0;
        }

        // Display useful live values.
        evo.clearDisplay();

        evo.writeToDisplay("L:", 0, 0);
        evo.writeToDisplay(leftDist, 16, 0);

        evo.writeToDisplay("R:", 62, 0);
        evo.writeToDisplay(rightDist, 78, 0);

        evo.writeToDisplay("F:", 0, 16);
        evo.writeToDisplay(frontDist, 16, 16);

        evo.writeToDisplay("H:", 62, 16);
        evo.writeToDisplay((int)readHeadingSafe(), 78, 16);

        evo.writeToDisplay("T:", 0, 32);
        evo.writeToDisplay((int)targetHeading, 16, 32);

        evo.drawDisplay();

        previousLeftDist = leftDist;
        previousRightDist = rightDist;

        delay(10);
    }
}
// ---------- Main behaviour: single-phase square with PD steering ----------


void setup() {
    evo.begin();
    tofRight.begin();
    tofBack.begin();
    tofLeft.begin();
    tofFront.begin();
    bno.begin();

    // IMU init with a basic check
    evo.clearDisplay();
    evo.writeLineToDisplay("Init IMU...", 0, true, true);
    evo.drawDisplay();



    evo.playTone(1000,10,false);
    delay(1000);


    // Take one good reading to seed lastValidHeading and targetHeading
    float x, y, z;
    bno.getEuler(&x, &y, &z);
    if (!isnan(x) && !isinf(x) && x >= 0 && x < 360) {
    lastValidHeading = normalize(x);
    } else {
    lastValidHeading = 0.0f; // safe default
    }


    driveMotor.begin();
    steerServo.begin();


    steerServo.write(SERVO_CENTER);
    evo.writeLineToDisplay("Setup done", 0, true, true);
    evo.drawDisplay();
    driveMotor.resetAngle();
    //driveMotor.runAngle(4000,500);
    // parallelParkWithTOF();
    //squareWithPDSteering(4);
    driveAndTurnOnGaps();
    steerServo.write(SERVO_CENTER);
}

void loop() {
    // one-shot run in setup()
    //evo.writeToDisplay(tofFront.getDistance(), 0, 0);
    //evo.drawDisplay();
    //evo.writeLineToDisplay(tofFront.getDistance(), 1, true, false);
    //evo.writeLineToDisplay(tofLeft.getDistance(), 2, false, false);
    //evo.writeLineToDisplay(tofRight.getDistance(), 3, false, false);
    //evo.writeLineToDisplay(tofBack.getDistance(), 4, false, false);
    //evo.drawDisplay();
    //delay(10);
}

