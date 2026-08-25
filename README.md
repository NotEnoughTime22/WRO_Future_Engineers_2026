# SSTitans

This repository contains the complete engineering materials for SSTactical's self-driving vehicle competing in the WRO Future Engineers 2026 season. It documents the hardware design, software architecture, machine learning pipeline, and control algorithms that enable the robot to navigate an autonomous track, detect gaps, perform parallel parking, and recognise visual markers.

## Repository Structure

* `Codes/` contains the Arduino sketch that runs on the EvolutionX1 microcontroller. The file `Open_Code.ino` implements the primary autonomous driving logic including PD heading control, gap navigation, square traversal, and parallel parking.
* `Documents/` contains the Engineering Journal maintained by the team throughout the season.
* `Libraries/` contains the custom C++ libraries written to interface with the vehicle hardware. These include motor control, sensor abstraction, pin mapping, and the core Evo system library.
* `ML/` contains the machine learning pipeline for visual object detection running on the MaixCam. This includes the trained YOLOv5 model, the inference script, training annotations, and configuration files.
* `Models/` contains 3D printable STL files for custom mounts and brackets used in the vehicle construction.
* `Photos/` contains documentation photographs of the team and the vehicle at various stages of development.
* `Videos/` contains videos of the robot performing the open and challenge runs. 

## Hardware Components

The vehicle is built around the following electromechanical components:

| Component | Model | Quantity | Purpose |
| :--- | :--- | :--- | :--- |
| Microcontroller | EvolutionX1 (ESP32-S3) | 1 | Central processing unit running all control logic |
| Drive Motor | EvoMotor300 | 1 | Provides propulsion via rear axle |
| Steering Servo | GeekServo 360 Grey | 1 | Controls front wheel steering angle |
| Camera | MaixCam | 1 | Runs YOLOv5 inference for visual detection |
| Distance Sensors | VL53L0X (Time of Flight) | 4 | Obstacle and gap detection (front, back, left, right) |
| Compass / IMU | BNO055 | 1 | Provides absolute heading for navigation |
| I2C Multiplexer | TCA9548A | 1 | Routes I2C signals to multiple sensors on shared bus |
| IO Expander | SX1506 | 1 | Provides PWM outputs for motor control |
| Battery | Panasonic NCR18650B | 2 | Power supply for all electronics |

### Wiring and Pin Assignments

The EvolutionX1 exposes four Lego sensor ports (S1 through S4) and four motor ports (M1 through M4). The pin mapping is defined in `Libraries/X1pins (1).h`. Each sensor port provides two GPIO pins for UART or I2C communication. Each motor port provides two PWM channels through the SX1506 IO expander and two tachometer pins for quadrature encoder feedback.

The I2C bus is multiplexed through the TCA9548A with the following channel assignments:

| I2C Channel | Device |
| :--- | :--- |
| 0 | IMU (BNO055) on I2C1 |
| 1 | ToF Right (VL53L0X) on I2C2 |
| 2 | ToF Left (VL53L0X) on I2C3 |
| 3 | ToF Front (VL53L0X) on I2C4 |
| 4 | ToF Back (VL53L0X) on I2C5 |
| 5 | SX1506 IO Expander |
| 6 | MPU9250 (reserved) |
| 7 | SSD1306 OLED Display |

The MaixCam communicates with the microcontroller over I2C5 at address `0x24`, transmitting detected object data in a binary protocol.

## Software Architecture

The software is split across two processors.

### Microcontroller (EvolutionX1 / ESP32-S3)

The main firmware is written in C++ using the Arduino framework. The entry point is `Codes/Open_Code.ino`. The architecture consists of:

**Hardware Abstraction Layer** provided by the custom libraries in `Libraries/`:

* `Evo (1).h / .cpp` wraps the core EVO singleton class. It manages I2C bus initialisation, TCA9548A channel selection, battery voltage reading via the BQ25887 charger IC, buzzer tone generation, and SSD1306 OLED display output.
* `EV3 Motor (1).h / .cpp` implements the `EV3Motor` class. Each motor instance is bound to a port (M1 to M4) and controls direction via the SX1509 IO expander PWM outputs. Encoder feedback is read through the `ESP32Encoder` library. A FreeRTOS background task runs a proportional controller for braking and target-position modes. The motor supports four states: RUN (continuous speed), TARGET (drive to encoder position), BRAKE (active hold), and COAST (passive stop).
* `VL53L0X Library.h / .cpp` wraps the Adafruit VL53L0X driver. Each sensor is assigned to an I2C multiplexer channel. The `getDistance()` method selects the correct channel, performs a ranging measurement, and returns the distance in millimetres. Readings below 10mm are treated as invalid and replaced with a sentinel value of 8191.
* `EV3 Sensor Port (1).h / .cpp` implements the Lego UART sensor protocol. It handles sensor discovery, mode negotiation, baud rate switching, and continuous data streaming using a dedicated FreeRTOS task pinned to core 0.
* `X1pins (1).h` defines all GPIO pin mappings, I2C addresses, and enumerations for motor ports, sensor ports, and I2C channels.

**Control Layer** implemented directly in `Open_Code.ino`:

The control layer uses a PD (Proportional-Derivative) controller to maintain heading while driving. The key parameters are:

* `KP_HEADING = 1.7` proportional gain
* `KD_HEADING = 0.0` derivative gain (currently disabled)
* `DRIVE_SPEED = 3000` constant motor speed in encoder degrees per second
* `STRAIGHT_DEGREES = 6000` encoder degrees per square edge
* `SERVO_RATIO = 2.33` maps wheel degrees to servo degrees
* `MAX_WHEEL_ANGLE = 35` maximum steering angle in degrees

The `applySteering()` function converts a desired wheel angle into a servo command by multiplying by the servo ratio and offsetting from the centre position (180 degrees). The result is constrained to the 0 to 360 degree range of the GeekServo 360.

Heading is read from the BNO055 IMU using `readHeadingSafe()`, which performs two attempts and falls back to the last valid reading if both fail. This prevents sudden heading jumps from corrupting the controller. All headings are normalised to the 0 to 360 degree range, and heading error is computed as the shortest angular distance (clamped to plus or minus 180 degrees).

The `snapHeading45()` function rounds any heading to the nearest 45 degree increment (0, 45, 90, 135, 180, 225, 270, 315). This is used throughout the navigation logic to ensure the robot only drives along axis-aligned and diagonal directions, which simplifies the track geometry.

### Driving Modes

The firmware implements three primary driving behaviours, selected in `setup()`:

**1. Square Traversal (`squareWithPDSteering`)**

Drives a square path of a given number of sides. The robot holds a constant speed and uses PD steering to maintain the current target heading. After travelling `STRAIGHT_DEGREES` encoder counts, the target heading is rotated by 90 degrees and snapped to the nearest 45 degree increment. The process repeats until all sides are completed, then the motor brakes and the servo centres.

**2. Gap Navigation (`driveAndTurnOnGaps`)**

This is the primary autonomous driving mode for the competition track. The robot drives forward while continuously monitoring the left and right Time of Flight sensors. Gap detection uses a `ToFSmoother` struct that maintains a 5-sample rolling average filter on each sensor. A gap is confirmed when either:

* The filtered distance exceeds `GAP_THRESHOLD_MM` (650mm), or
* The distance increases by more than 150mm between consecutive readings

Both conditions must persist for `GAP_CONFIRM_SAMPLES` (4) consecutive readings to avoid false triggers. A cooldown timer of `TURN_COOLDOWN_MS` (700ms) prevents multiple turns at the same intersection.

When a gap is confirmed, the robot computes the new target heading by rotating 90 degrees left or right. If gaps are detected on both sides, the robot turns toward the side with the larger clearance (plus a 100mm margin). The turn is executed while the motor continues running, using IMU heading control. The turn completes when the heading error drops below `TURN_COMPLETE_ERROR` (8 degrees). After turning, the robot drives forward for `POST_TURN_DISTANCE` (1000 encoder degrees) to clear the intersection before resuming gap detection.

A front-facing safety check stops the motor if an obstacle is detected within 80mm during a turn, then immediately resumes driving after a 150ms pause.

**3. Parallel Parking (`parallelParkWithTOF`)**

Executes a multi-phase parallel parking manoeuvre using all four ToF sensors:

* Phase 1: If the right sensor reads more than 100mm, the robot is too far from the wall. It angles 45 degrees toward the wall and drives in small increments until the diagonal distance (right distance multiplied by sin(45)) drops below 100mm.
* Phase 2: The robot turns to run parallel to the wall and drives forward until the front sensor reads less than 200mm.
* Phase 3: The robot angles 45 degrees away from the wall and drives a fixed distance of 1000 encoder degrees to pull alongside the parking spot.
* Phase 4: The robot straightens and drives forward until the front sensor reads less than 100mm, then completes with a final 200-degree adjustment.

### MaixCam (Vision Processor)

The MaixCam runs a Python script (`ML/main.py`) that performs real-time object detection using a YOLOv5 model. The model was trained on 147 (Based on 25 Aug, we are still training, final model should include 1000+) annotated images and compiled into a `.mud` and `.cvimodel` format for the Maix hardware accelerator.

The detection loop operates as follows:

1. Capture a frame from the camera at the model's input resolution (224x224, 3 channels).
2. Run YOLOv5 inference with a confidence threshold of 0.5 and IoU threshold of 0.45.
3. If objects are detected, encode their bounding boxes (x, y, width, height, class ID, confidence score) into a binary payload using the `encode_objs()` function. Each object occupies 14 bytes in the payload.
4. Transmit the payload to the EvolutionX1 microcontroller over I2C5 at address `0x24`.
5. Draw red bounding boxes and labels on the live preview displayed on the MaixCam screen.

The model configuration in `ML/app.yaml` identifies it as "Detector 317447" version 1.0.0, generated by MaixHub. The training report in `ML/report.json` shows the model converged to a final loss of approximately 0.68 over 100 epochs with a learning rate schedule decaying from 0.001 to 0.0001.

## 3D Printed Models

The `Models/` directory contains two STL files for custom mechanical parts:

* `EVO Brick Model.stl` is the enclosure case that holds the EvolutionX1 microcontroller brick in place on the chassis.
* `VL53L0X Lego Mount Model.stl` is a bracket that mounts a VL53L0X Time of Flight sensor to the Lego Technic frame at the correct height and angle.

## Setup Instructions

### Prerequisites

* Arduino IDE with ESP32 board support installed
* Board selection set to ESP32-S3
* USB cable capable of data transfer

### Library Installation

All libraries must be installed before compiling the code.

1. Install each `.zip` file found in the external libraries folder by opening the Arduino IDE, navigating to Sketch, then Include Library, then Add .ZIP Library, and selecting each archive.
2. Install the custom Evo library by selecting the `Evo.zip` archive through the same Add .ZIP Library menu.
3. All libraries are required for the code to compile and run correctly.

### Uploading the Firmware

1. Open `Codes/Open_Code.ino` in the Arduino IDE.
2. Verify that the board is set to ESP32-S3 under the Tools menu.
3. Select the correct serial port for the EvolutionX1.
4. Click Upload to flash the firmware to the microcontroller.

### Deploying the MaixCam Model

1. Copy the contents of the `ML/` directory to the MaixCam's SD card or internal storage.
2. Ensure the `model_317447.mud` and `model_317447.cvimodel` files are present alongside `main.py`.
3. Power on the MaixCam and the script will begin executing automatically.

## Pre-Run Checklist

### Software Verification

* Confirm the Arduino IDE board selection is set to ESP32-S3.
* Verify that all sensors have been calibrated for the competition venue lighting and surface conditions.
* Ensure the MaixCam model is loaded and the I2C address matches the microcontroller configuration.

### Hardware Inspection

* Check that every wire is connected to the correct terminal and that no loose connections exist.
* Test all four ToF sensors for consistent readings and replace any that return erratic values.
* Verify the IMU provides stable heading readings without drift.
* Inspect all structural parts for tightness and replace any cracked or weakened components.
* Ensure the wheels are securely fastened to the axles and do not wobble.
* Test the steering servo for smooth operation across the full 0 to 360 degree range.
* Confirm the battery voltage is above the minimum operating threshold.

### Robot Placement

* Place the robot on the track in a straight orientation aligned with the intended starting direction.
* The initial heading is captured during setup and used as the reference for all subsequent navigation.
