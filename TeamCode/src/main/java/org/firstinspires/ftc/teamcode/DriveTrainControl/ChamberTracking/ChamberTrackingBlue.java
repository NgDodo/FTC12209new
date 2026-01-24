package org.firstinspires.ftc.teamcode.DriveTrainControl.ChamberTracking;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Chamber Tracking [Blue]", group = "DriveTrainControl")
public class ChamberTrackingBlue extends OpMode {

    // ========================================================================
    // HARDWARE DECLARATIONS
    // ========================================================================

    // === Drive Train Motors ===
    // Four mecanum wheel motors for holonomic (omnidirectional) drive
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // === Mechanism Motors ===
    DcMotor m1;      // Intake motor - pulls game pieces into the robot
    DcMotorEx m2;    // Turret rotation motor - rotates the shooter left/right
    DcMotorEx m3;    // Flywheel motor - spins to launch game pieces
    DcMotorEx m0;    // Sorter motor - rotates the 3-chamber sorting system

    // === Servos ===
    CRServo s3;      // Continuous rotation servo - likely the flicker mechanism to push balls into flywheel
    Servo s2;        // Standard servo - likely a gate/blocker for the flicker

    // === Color Sensors ===
    // Two REV color sensors to detect game piece colors (green vs purple balls)
    private RevColorSensorV3 intakeColor;   // Sensor at intake to detect incoming balls
    private RevColorSensorV3 shooterColor;  // Sensor at shooter to detect balls being shot

    // === RGB LED Indicators ===
    private Servo led1;  // LED strip 1 - shows detected ball color
    private Servo led2;  // LED strip 2 - shows flywheel RPM status

    // === Vision & Sensors ===
    private Limelight3A limelight;  // Camera for AprilTag detection and distance measurement
    private IMU imu;                // Inertial Measurement Unit for robot orientation

    // ========================================================================
    // LIMELIGHT VISION CONFIGURATION
    // ========================================================================

    private static final String LIMELIGHT_NAME = "Webcam 2";
    private static final int APRILTAG_PIPELINE = 1;  // Pipeline configured for AprilTag detection

    // === Turret Return to Center ===
    private boolean turretWrapping = false;  // Flag for turret unwrapping logic (currently unused)

    // ========================================================================
    // TRACKING MODE SYSTEM
    // ========================================================================

    // Enum defining three tracking modes for the turret/robot
    private enum TrackingMode {
        OFF,          // No automatic tracking
        BODY_TRACK,   // Rotate entire robot body to face target
        TURRET_TRACK  // Only rotate turret to face target (current implementation)
    }
    private TrackingMode currentTrackingMode = TrackingMode.OFF;

    // ========================================================================
    // TURRET MOTOR CONFIGURATION
    // ========================================================================

    // Physical limits and conversion factors for the turret rotation
    private static final double TURRET_RANGE_DEG = 330.0;           // Turret can rotate 330 degrees
    private static final double TICKS_PER_REV = 1393.1;             // Encoder ticks per full rotation
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;  // Conversion factor

    // === Turret PID Control ===
    private double turretLastError = 0;  // Previous error for derivative calculation (unused currently)

    // === Body Tracking PID ===
    private double bodyLastError = 0.0;                      // Previous error for robot rotation
    private static final double BODY_TOLERANCE_DEG = 0.6;    // Acceptable error in degrees

    // === Smoothing & Prediction ===
    private double smoothedBearing = 0.0;      // Smoothed angle to target (unused currently)
    private boolean hasEverSeenTag = false;    // Flag if AprilTag ever detected (unused currently)

    // ========================================================================
    // SORTER SYSTEM CONFIGURATION
    // ========================================================================

    // The sorter is a rotating mechanism with 3 chambers (slots) that can hold game pieces
    // It rotates to position different chambers for intake or shooting

    private static final int FULL_ROT = 8192;        // Total encoder ticks for one complete rotation
    private static final int SLOT = FULL_ROT / 3;    // Ticks between chambers (120 degrees)
    private static final int OFFSET = (FULL_ROT / 2); // 180 degree offset for shooting mode (A goes to back)

    // Three chamber positions in intake mode (evenly spaced 120 degrees apart)
    private static final int CHAMBER_0_POS = 0;           // Chamber 0 at 0 degrees
    private static final int CHAMBER_1_POS = SLOT;        // Chamber 1 at 120 degrees
    private static final int CHAMBER_2_POS = 2 * SLOT;    // Chamber 2 at 240 degrees

    // === Chamber State Tracking ===
    private boolean[] chamberFull = new boolean[3];  // Tracks which chambers contain a ball
    private int currentChamber = 0;                  // Which chamber is currently at intake/shooter position
    private boolean shootingMode = false;            // false = intake mode, true = shooting mode

    // Button state tracking for edge detection
    private boolean lastY = false;           // Y button toggles shooting/intake mode
    private boolean lastDpadRight = false;   // DPad Right rotates to next chamber

    // ========================================================================
    // NON-BLOCKING SORTER MOVEMENT SYSTEM
    // ========================================================================

    // The sorter uses a state machine to move to target positions without blocking code execution
    // This allows other robot functions to continue while the sorter is moving

    private boolean sorterMoving = false;           // Is the sorter currently moving?
    private int sorterTargetPosition = 0;           // Target position in encoder ticks
    private ElapsedTime sorterTimer = new ElapsedTime();       // Timeout timer for movement
    private ElapsedTime sorterSettleTimer = new ElapsedTime(); // Timer for settling at position
    private boolean sorterSettling = false;         // Is sorter in settling phase?

    // Movement tolerances (in encoder ticks)
    private static final int COARSE_TOL = 1000;     // Threshold for starting to slow down
    private static final int FINE_TOL = 60;         // Fine positioning tolerance
    private static final int PERFECT_TOL = 30;      // Perfect position tolerance

    // Power limits for sorter motor
    private static final double MAX_POWER = 0.55;   // Maximum motor power
    private static final double MIN_POWER = 0.08;   // Minimum motor power

    // Timeouts
    private static final long SORTER_TIMEOUT_MS = 2000;  // Maximum time for movement (2 seconds)
    private static final long SETTLE_MS = 100;           // Time to settle at position (100ms)

    // ========================================================================
    // SHOOTER RPM CONTROL
    // ========================================================================

    private boolean lastXButton = false;        // X button toggles distance-based RPM mode
    private boolean distanceBasedRPM = false;   // Auto-adjust RPM based on distance to target
    private double lastValidDistance = 110.0;   // Last known distance to target (inches)

    // ========================================================================
    // TIMED COLOR DETECTION
    // ========================================================================

    // Color detection requires sustained detection to avoid false positives
    // Ball must be seen for DETECT_TIME_MS continuously before being registered

    private long colorStartTime = 0;                  // When color was first detected
    private boolean colorActive = false;              // Is color detection timer active?
    private static final long DETECT_TIME_MS = 25;    // Required detection time (25ms)

    // ========================================================================
    // EMPTY CHAMBER DETECTION
    // ========================================================================

    // When shooting, detect when a ball has left the chamber
    // Chamber is considered empty after shooter sensor sees "NONE" for EMPTY_DETECT_TIME_MS

    private long emptyStartTime = 0;                      // When "no ball" was first detected
    private boolean emptyDetectionActive = false;         // Is empty detection timer active?
    private static final long EMPTY_DETECT_TIME_MS = 200; // Required "no ball" time (200ms)

    // ========================================================================
    // SHOOTER FLYWHEEL PRESETS
    // ========================================================================

    // Preset RPM values for different shooting distances
    private final int[] rpmPresets = {2600, 3300};  // Low power (close) and high power (far)
    private int presetIndex = -1;                   // Current preset index (-1 = none selected)
    private double targetRPM = 0;                   // Current target RPM

    // Button state tracking
    private boolean lastRightBumper = false;   // Right bumper cycles through presets
    private boolean lastLeftBumper = false;    // Left bumper sets 1500 RPM
    private boolean lastDpadLeft = false;      // DPad left sets -4000 RPM (reverse)

    // Flywheel motor specifications
    private static final double TICKS_PER_REV_FLYWHEEL = 28.0;  // Encoder ticks per revolution
    private static final double RPM_TOLERANCE = 100.0;          // Acceptable RPM error
    private boolean lastDpadDown = false;                       // DPad down stops flywheel

    // ========================================================================
    // FLYWHEEL PID CONTROLLER
    // ========================================================================

    // PID controller maintains consistent flywheel speed for accurate shooting
    // Formula: output = Kp*error + Ki*integral + Kd*derivative + Kf*target

    private double flywheelKp = 0.0012;   // Proportional gain - responds to current error
    private double flywheelKi = 0.00001;  // Integral gain - eliminates steady-state error
    private double flywheelKd = 0.0;      // Derivative gain - dampens oscillations (currently 0)
    private double flywheelKF = 0.00025;  // Feedforward gain - baseline power for target RPM

    private double flywheelIntegral = 0;   // Accumulated error over time
    private double flywheelLastError = 0;  // Previous error for derivative
    private long flywheelLastTime = 0;     // Previous timestamp for dt calculation

    // ========================================================================
    // PEDRO PATHING (ODOMETRY) SYSTEM
    // ========================================================================

    // Pedro Pathing provides precise robot localization using odometry
    // Tracks robot's position and orientation on the field

    private Follower follower;              // Pedro Pathing follower object
    public static Pose startingPose;        // Robot's starting position on field

    // ========================================================================
    // CHAMBER COLOR TRACKING SYSTEM
    // ========================================================================

    // The sorter has 3 chambers labeled A, B, C (conceptually, moving clockwise)
    //
    // IMPORTANT CONCEPT:
    // - The chamber POSITIONS are fixed: A is always at index 0, B at 1, C at 2
    // - But the CONTENTS rotate when the sorter physically rotates
    // - When rotating clockwise: A→B, B→C, C→A (contents shift, not labels)
    // - chamberColors[0] always represents the "front" chamber (intake or shooter position)
    //
    // INTAKE MODE:
    // - Chamber at position A (index 0) is at the intake
    // - When ball detected, chamberColors[0] is updated with color
    // - Sorter rotates to next empty chamber
    //
    // SHOOTING MODE:
    // - Entire sorter shifts 60 degrees (OFFSET)
    // - Chamber at position A (index 0) is now at the shooter
    // - When ball shoots, chamberColors[0] is cleared to "NONE"
    //
    // COLOR UPDATE:
    // - Only chamberColors[0] is actively updated based on sensor
    // - Other positions maintain their last known color until rotated to position A

    private String[] chamberColors = {"NONE", "NONE", "NONE"};

    // ========================================================================
    // INITIALIZATION
    // ========================================================================

    @Override
    public void init() {
        // === Pedro Pathing Setup ===
        // Set starting position on field (x=20.9, y=123.1, heading=144°)
        startingPose = new Pose(20.9, 123.1, Math.toRadians(144));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        // === DriveTrain Initialization ===
        // Configure four mecanum wheel motors
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "fL");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "bL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "bR");

        // Set all drive motors to brake when power is zero (better control)
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse left side motors for correct mecanum drive
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // === Mechanism Motors Initialization ===
        m1 = hardwareMap.get(DcMotor.class, "m1");      // Intake
        m2 = hardwareMap.get(DcMotorEx.class, "m2");    // Turret
        m3 = hardwareMap.get(DcMotorEx.class, "m3");    // Flywheel
        m0 = hardwareMap.get(DcMotorEx.class, "m0");    // Sorter

        // === Servo Initialization ===
        s2 = hardwareMap.get(Servo.class, "s2");         // Flicker gate
        s3 = hardwareMap.get(CRServo.class, "s3");       // Flicker wheel
        s3.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setPosition(.68);  // Start with gate closed

        // Set all mechanism motors to brake mode
        for (DcMotor motor : new DcMotor[]{m1, m3, m0}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // === Turret Motor Setup ===
        // Reset encoder and use encoder feedback for turret positioning
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === Flywheel Motor Setup ===
        // Run without encoder for direct velocity control
        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // === Using back right motor encoder for sorter position ===
        // The sorter (m0) doesn't have its own encoder, so we use backRightMotor's encoder
        // This is a common technique when encoder ports are limited
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // === Flywheel Encoder Setup ===
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        // === Intake Motor Direction ===
        m1.setDirection(DcMotorSimple.Direction.REVERSE);

        // === Color Sensor Initialization ===
        intakeColor  = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        shooterColor = hardwareMap.get(RevColorSensorV3.class, "shooterColor");

        // === RGB LED Initialization ===
        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");
        led1.setPosition(1.0);  // LEDs off initially
        led2.setPosition(1.0);

        // === IMU (Gyroscope) Setup ===
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();  // Zero the heading

        // === Limelight Vision Setup ===
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);              // Update at 100Hz
        limelight.pipelineSwitch(APRILTAG_PIPELINE); // Switch to AprilTag detection
        limelight.start();

        // Initialize flywheel PID timer
        flywheelLastTime = System.nanoTime();

        telemetry.addLine("Integrated Turret Tracking Initialized");
        telemetry.addLine("B: Cycle Tracking Mode (OFF/Body/Turret)");
        telemetry.update();
    }

    // ========================================================================
    // START (Called once when driver presses PLAY)
    // ========================================================================

    public void start() {
        // Start Pedro Pathing's teleop drive mode
        follower.startTeleopDrive();

        // Reset turret encoder to establish zero position
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ========================================================================
    // MAIN LOOP (Called repeatedly during teleop)
    // ========================================================================

    @Override
    public void loop() {
        // Update odometry to track robot position
        follower.update();

        // ====================================================================
        // MECANUM DRIVE CONTROL
        // ====================================================================

        // Read joystick inputs with deadzone applied
        double y = applyDeadzone(-gamepad1.left_stick_y);   // Forward/backward
        double x = applyDeadzone(gamepad1.left_stick_x);    // Strafe left/right
        double rx = applyDeadzone(gamepad1.right_stick_x);  // Rotate

        // Mecanum drive calculation
        // Each wheel gets a combination of forward, strafe, and rotation
        double fl = y + x + rx;  // Front left
        double bl = y - x + rx;  // Back left
        double fr = y - x - rx;  // Front right
        double br = y + x - rx;  // Back right

        // Normalize so no wheel exceeds 100% power
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));

        // Apply power to motors, clipping very low values to zero
        frontLeftMotor.setPower(clipLowPower(fl / max));
        backLeftMotor.setPower(clipLowPower(bl / max));
        frontRightMotor.setPower(clipLowPower(fr / max));
        backRightMotor.setPower(clipLowPower(br / max));

        // ====================================================================
        // SORTER CONTROL
        // ====================================================================

        // Read current sorter position from back right motor encoder
        int rawPos = backRightMotor.getCurrentPosition();
        int normPos = normalize(rawPos);  // Wrap to 0-8192 range

        // === Y Button: Toggle Shooting/Intake Mode ===
        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) {  // Edge detection (only on button press, not hold)
            shootingMode = !shootingMode;  // Toggle mode

            // Calculate new target position for current chamber in new mode
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            startSorterMove(targetPos);  // Begin movement
        }
        lastY = yPressed;

        // === DPad Right: Rotate to Next Chamber ===
        boolean dpadRightPressed = gamepad1.dpad_right;
        if (dpadRightPressed && !lastDpadRight) {
            currentChamber = nextChamber(currentChamber);  // Get next chamber (0→2→1→0)

            // Calculate target position for new current chamber
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            startSorterMove(targetPos);
        }
        lastDpadRight = dpadRightPressed;

        // === Update Sorter Movement ===
        // This runs every loop to continue moving sorter toward target
        updateSorterMovement();

        // ====================================================================
        // COLOR DETECTION
        // ====================================================================

        // In intake mode: detect when ball enters and fill chamber
        if (!shootingMode) {
            autoIntakeColorCheck();
        }
        // In shooting mode: detect when ball leaves chamber
        else {
            checkChamberEmpty();
        }

        // Get current color at shooter position and update LED indicator
        String shooterColorDetected = detectShooterColor();
        updateColorLEDs(shooterColorDetected);

        // ====================================================================
        // INTAKE CONTROL
        // ====================================================================

        // Only run intake in intake mode (not during shooting)
        if (!shootingMode) {
            // Right trigger = intake, Left trigger = outtake
            double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
            m1.setPower(triggerPower);
        } else {
            m1.setPower(0);  // Intake off during shooting
        }

        // ====================================================================
        // SHOOTER/FLICKER CONTROL
        // ====================================================================

        // A button fires the ball
        if (gamepad1.a) {
            s2.setPosition(0);      // Open gate
            s3.setPower(1.0);       // Spin flicker wheel to push ball
        } else {
            s2.setPosition(.68);    // Close gate
            s3.setPower(0.0);       // Stop flicker
        }

        // ====================================================================
        // FLYWHEEL RPM MODE SELECTION
        // ====================================================================

        // X button toggles between distance-based and manual RPM modes
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastXButton) {
            distanceBasedRPM = !distanceBasedRPM;
        }
        lastXButton = xPressed;

        // === Right Bumper: Set Target RPM ===
        if (gamepad1.right_bumper && !lastRightBumper) {
            if (distanceBasedRPM) {
                // AUTO MODE: Choose RPM based on distance to goal
                double distance = getAprilTagDistance();
                if (distance > 0) {
                    lastValidDistance = distance;
                    targetRPM = (distance < 110.0) ? 2500 : 3000;  // Close = 2500, Far = 3000
                } else {
                    // No AprilTag visible, use last known distance
                    targetRPM = (lastValidDistance < 110.0) ? 2500 : 3000;
                }
            } else {
                // MANUAL MODE: Cycle through preset RPMs
                presetIndex = (presetIndex + 1) % rpmPresets.length;
                targetRPM = rpmPresets[presetIndex];
            }
        }
        // === DPad Down: Stop Flywheel ===
        else if (gamepad1.dpad_down && !lastDpadDown) {
            targetRPM = 0;
        }
        // === DPad Left: Reverse Flywheel ===
        else if (gamepad1.dpad_left && !lastDpadLeft) {
            targetRPM = -4000;  // Negative for reverse spin
        }

        // === Left Bumper: Set 1500 RPM ===
        boolean leftBumperPressed = gamepad1.left_bumper;
        if (leftBumperPressed && !lastLeftBumper) {
            targetRPM = 1500;
        }

        // Update button states for next loop
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = leftBumperPressed;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadDown = gamepad1.dpad_down;

        // ====================================================================
        // FLYWHEEL PID CONTROL
        // ====================================================================

        // Get current flywheel speed
        double currentVelocity = m3.getVelocity();  // Ticks per second
        double currentRPM = (currentVelocity / TICKS_PER_REV_FLYWHEEL) * 60.0;

        // Calculate time since last update (for integral and derivative)
        long currentTime = System.nanoTime();
        double dt = (currentTime - flywheelLastTime) / 1e9;  // Convert to seconds

        // Calculate error
        double error = targetRPM - currentRPM;

        // Integral term (accumulate error over time, with anti-windup)
        flywheelIntegral += error * dt;
        flywheelIntegral = Math.max(-10000, Math.min(10000, flywheelIntegral));

        // Derivative term (rate of change of error)
        double derivative = (error - flywheelLastError) / dt;

        // Feedforward term (baseline power needed for target speed)
        double feedforward = flywheelKF * targetRPM;

        // Combine all PID terms
        double pidOutput = (flywheelKp * error) + (flywheelKi * flywheelIntegral) +
                (flywheelKd * derivative) + feedforward;

        // Clamp output to valid motor power range
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
        m3.setPower(pidOutput);

        // Save values for next iteration
        flywheelLastError = error;
        flywheelLastTime = currentTime;

        // Update RPM status LED
        updateRPMLED();

        // Update driver station telemetry
        updateTelemetry(normPos, shooterColorDetected);

        // ====================================================================
        // TURRET TRACKING (AUTO-AIM AT GOAL)
        // ====================================================================

        // Define goal position on field (x=10, y=134)
        Pose GOAL_POST = new Pose(10, 134, 0);

        // 1. Calculate distances from robot to goal
        double y_goal_distance = follower.getPose().getY() - GOAL_POST.getY();
        double x_goal_distance = follower.getPose().getX() - GOAL_POST.getX();

        // 2. Calculate absolute angle to goal in field coordinates
        double angle_to_goal = Math.atan2(y_goal_distance, x_goal_distance);

        // 3. Calculate turret offset relative to robot heading
        // This converts the field-relative angle to robot-relative angle
        // Add PI to face the goal (not away from it)
        double turretDesiredRelativeOffset = normalizeAngle(-angle_to_goal + follower.getHeading() + Math.PI);

        // 4. Move turret to calculated angle
        moveTurretToOffset(m2, turretDesiredRelativeOffset);

        // ====================================================================
        // POSE RESET (FOR TESTING)
        // ====================================================================

        // DPad Up resets robot position to center of field
        if (gamepad1.dpad_up) {
            follower.setPose(new Pose(72, 72, Math.PI / 2));
        }
    }

    // ========================================================================
    // APRILTAG DISTANCE MEASUREMENT
    // ========================================================================

    /**
     * Gets distance to goal using AprilTag detection
     * Returns 80 inches if tag is large (close), 120 inches if small (far)
     * Returns -1 if no valid tag detected
     */
    private double getAprilTagDistance() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            // Look for goal AprilTags (IDs 20 or 24)
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24) {
                    // Found a goal AprilTag (ID 20 or 24)
                    // Use tag size/area as rough distance estimate
                    double area = fiducial.getTargetArea();
                    if (area > 3.0) {
                        return 80.0;   // Large tag = close (80 inches)
                    } else {
                        return 120.0;  // Small tag = far (120 inches)
                    }
                }
            }
        }
        return -1;  // No valid tag found
    }

    // ========================================================================
    // RPM LED INDICATOR
    // ========================================================================

    /**
     * Updates LED2 to show flywheel RPM status
     * - In intake mode: Shows white if ball detected
     * - In shooting mode: Shows green when RPM is ready, off otherwise
     */
    private void updateRPMLED() {
        // In intake mode, prioritize showing ball detection
        if (!shootingMode) {
            String intakeDetected = detectIntakeColor();
            if (!intakeDetected.equals("NONE")) {
                led2.setPosition(1.0);  // White light = ball detected
                return;
            }
        }

        // If flywheel is off, turn LED off
        if (targetRPM == 0) {
            led2.setPosition(0);
            return;
        }

        // Check if flywheel is at target speed
        double currentRPM = (m3.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);

        if (rpmError <= RPM_TOLERANCE) {
            led2.setPosition(0.3);  // Green light = ready to shoot
        } else {
            led2.setPosition(0);    // Off = still spinning up
        }
    }

    // ========================================================================
    // SORTER MOVEMENT CONTROL (NON-BLOCKING)
    // ========================================================================

    /**
     * Updates the sorter motor to move toward target position
     * This is called every loop iteration to continue movement
     * Uses proportional control with settling time for accuracy
     */
    private void updateSorterMovement() {
        // If not moving, nothing to do
        if (!sorterMoving) return;

        // Get current position and calculate error to target
        int pos = normalize(backRightMotor.getCurrentPosition());
        int error = calculateShortestError(pos, sorterTargetPosition);

        // Safety timeout - stop if movement takes too long (2 seconds)
        if (sorterTimer.milliseconds() > SORTER_TIMEOUT_MS) {
            m0.setPower(0);
            sorterMoving = false;
            sorterSettling = false;
            return;
        }

        // Tighter tolerances for faster completion
        int FAST_PERFECT_TOL = 50;   // Within 50 ticks = close enough
        int FAST_FINE_TOL = 100;     // Within 100 ticks = fine positioning

        // ===== SETTLING PHASE =====
        // When very close to target, enter settling phase
        if (Math.abs(error) <= FAST_PERFECT_TOL) {
            if (!sorterSettling) {
                // Just entered settling phase
                sorterSettling = true;
                sorterSettleTimer.reset();
                m0.setPower(0);  // Stop motor to settle
            }

            // Wait for settle time to ensure position is stable
            long FAST_SETTLE_MS = 30;  // 30ms settle time
            if (sorterSettleTimer.milliseconds() >= FAST_SETTLE_MS) {
                // Successfully settled at target position
                m0.setPower(0);
                sorterMoving = false;
                sorterSettling = false;
                return;
            }

            // If error increases during settling, exit settling phase
            if (Math.abs(error) > FAST_FINE_TOL) {
                sorterSettling = false;
            } else {
                return;  // Still settling, wait
            }
        } else {
            sorterSettling = false;  // Too far from target for settling
        }

        // ===== PROPORTIONAL SPEED CONTROL =====
        double power;
        int absError = Math.abs(error);

        if (absError > COARSE_TOL) {
            // Far from target: use maximum power
            power = MAX_POWER;
        } else {
            // Close to target: slow down proportionally
            // ratio goes from 1.0 (at COARSE_TOL) to 0.0 (at target)
            double ratio = (double) absError / COARSE_TOL;
            power = MIN_POWER + (MAX_POWER - MIN_POWER) * ratio;
            power = Math.max(MIN_POWER, Math.min(MAX_POWER, power));
        }

        // Apply power with correct direction (sign of error)
        m0.setPower(Math.signum(error) * power);
    }

    // ========================================================================
    // SORTER MOVEMENT INITIATION
    // ========================================================================

    /**
     * Starts a new sorter movement to target position
     * Resets all movement state variables
     */
    private void startSorterMove(int targetPosition) {
        sorterTargetPosition = targetPosition;
        sorterMoving = true;
        sorterSettling = false;
        sorterTimer.reset();
    }

    // ========================================================================
    // COLOR LED INDICATOR
    // ========================================================================

    /**
     * Updates LED1 to show the detected ball color
     * Green ball = green light, Purple ball = purple light, No ball = off
     */
    private void updateColorLEDs(String color) {
        if (color.equals("GREEN")) {
            led1.setPosition(0.5);    // Green light
        } else if (color.equals("PURPLE")) {
            led1.setPosition(0.722);  // Purple light
        } else {
            led1.setPosition(0);      // Off (no ball detected)
        }
    }

    // ========================================================================
    // CHAMBER POSITION CALCULATOR
    // ========================================================================

    /**
     * Calculates the encoder position for a given chamber
     * In shooting mode, adds 60-degree offset to align chamber with shooter
     *
     * @param chamber Which chamber (0, 1, or 2)
     * @param shooting Whether in shooting mode (true) or intake mode (false)
     * @return Target encoder position in ticks
     */
    private int getChamberPosition(int chamber, boolean shooting) {
        int basePos;
        switch(chamber) {
            case 0: basePos = CHAMBER_0_POS; break;  // 0 degrees
            case 1: basePos = CHAMBER_1_POS; break;  // 120 degrees
            case 2: basePos = CHAMBER_2_POS; break;  // 240 degrees
            default: basePos = CHAMBER_0_POS;
        }

        // In shooting mode, rotate entire sorter by 60 degrees
        if (shooting) basePos = normalize(basePos + OFFSET);

        return basePos;
    }

    // ========================================================================
    // AUTOMATIC INTAKE COLOR DETECTION
    // ========================================================================

    /**
     * Continuously checks for ball at intake during intake mode
     * When ball detected for DETECT_TIME_MS, fills current chamber and rotates to next
     * Uses timed detection to avoid false positives from brief color flashes
     */
    private void autoIntakeColorCheck() {
        // Don't check while sorter is moving (wait for it to settle)
        if (sorterMoving) return;

        String detected = detectIntakeColor();

        // No ball detected - reset timer
        if (detected.equals("NONE")) {
            colorActive = false;
            colorStartTime = 0;
            return;
        }

        // Ball detected - start/continue timer
        if (!colorActive) {
            colorActive = true;
            colorStartTime = System.currentTimeMillis();
        }

        // Ball has been detected continuously for required time
        if (System.currentTimeMillis() - colorStartTime >= DETECT_TIME_MS) {
            // Only fill chamber if it's not already full
            if (!chamberFull[currentChamber]) {
                chamberFull[currentChamber] = true;           // Mark chamber as full
                currentChamber = nextChamber(currentChamber); // Move to next chamber

                // Rotate sorter to position next empty chamber at intake
                int target = getChamberPosition(currentChamber, false);
                startSorterMove(target);
            }

            // Reset detection timer
            colorActive = false;
            colorStartTime = 0;
        }
    }

    // ========================================================================
    // CHAMBER EMPTY DETECTION
    // ========================================================================

    /**
     * Checks if current chamber has emptied after shooting
     * When shooter sensor sees "NONE" for EMPTY_DETECT_TIME_MS, marks chamber as empty
     * Only active in shooting mode for chambers marked as full
     */
    private void checkChamberEmpty() {
        // Only check if in shooting mode and chamber is marked full
        if (!shootingMode || !chamberFull[currentChamber]) {
            emptyDetectionActive = false;
            emptyStartTime = 0;
            return;
        }

        String detected = detectShooterColor();

        // Ball still present - reset timer
        if (!detected.equals("NONE")) {
            emptyDetectionActive = false;
            emptyStartTime = 0;
            return;
        }

        // No ball detected - start/continue timer
        if (!emptyDetectionActive) {
            emptyDetectionActive = true;
            emptyStartTime = System.currentTimeMillis();
        }

        // No ball detected continuously for required time - chamber is empty
        if (System.currentTimeMillis() - emptyStartTime >= EMPTY_DETECT_TIME_MS) {
            chamberFull[currentChamber] = false;  // Mark chamber as empty
            emptyDetectionActive = false;
            emptyStartTime = 0;
        }
    }

    // ========================================================================
    // COLOR DETECTION FUNCTIONS
    // ========================================================================

    /**
     * Detects ball color at intake sensor
     * Analyzes RGB values to determine if ball is green, purple, or not present
     *
     * @return "GREEN", "PURPLE", or "NONE"
     */
    private String detectIntakeColor() {
        int r = intakeColor.red();
        int g = intakeColor.green();
        int b = intakeColor.blue();

        // Green ball: green channel dominant and in valid range
        if (g > r && g > b && g > 80 && g < 600) return "GREEN";

        // Purple ball: blue channel dominant and in valid range
        if (b > r && b > g && b > 80 && b < 600) return "PURPLE";

        return "NONE";  // No ball or unrecognized color
    }

    /**
     * Detects ball color at shooter sensor
     * Same logic as intake sensor but for shooter position
     *
     * @return "GREEN", "PURPLE", or "NONE"
     */
    private String detectShooterColor() {
        int r = shooterColor.red();
        int g = shooterColor.green();
        int b = shooterColor.blue();

        if (g > r && g > b && g > 80 && g < 600) return "GREEN";
        if (b > r && b > g && b > 80 && b < 600) return "PURPLE";
        return "NONE";
    }

    // ========================================================================
    // CHAMBER STATUS HELPER
    // ========================================================================

    /**
     * Checks if all three chambers are full
     * @return true if all chambers contain balls
     */
    private boolean allChambersFull() {
        return chamberFull[0] && chamberFull[1] && chamberFull[2];
    }

    // ========================================================================
    // CHAMBER ROTATION SEQUENCE
    // ========================================================================

    /**
     * Gets the next chamber in rotation sequence
     * Rotation order: 0 → 2 → 1 → 0 (counter-clockwise from above)
     * This matches the physical counter-clockwise rotation of the sorter
     *
     * @param c Current chamber (0, 1, or 2)
     * @return Next chamber in sequence
     */
    private int nextChamber(int c) {
        if (c == 1) return 0;
        if (c == 0) return 2;
        return 1;  // c == 2, return 1
    }

    // ========================================================================
    // ENCODER POSITION NORMALIZATION
    // ========================================================================

    /**
     * Normalizes encoder ticks to 0-8192 range (one full rotation)
     * Handles negative values and values beyond one rotation
     * Example: -100 becomes 8092, 8300 becomes 108
     *
     * @param ticks Raw encoder ticks
     * @return Normalized ticks in range [0, FULL_ROT)
     */
    private int normalize(int ticks) {
        return ((ticks % FULL_ROT) + FULL_ROT) % FULL_ROT;
    }

    // ========================================================================
    // SHORTEST PATH CALCULATION
    // ========================================================================

    /**
     * Calculates the shortest rotational error between current and target positions
     * Since the sorter is circular, it can rotate either direction
     * This function chooses the shorter path
     *
     * Example: Current=100, Target=8000
     * - Clockwise: 8000-100 = 7900 ticks
     * - Counter-clockwise: 100+192-8000 = 292 ticks (SHORTER!)
     * - Returns: -292 (negative = counter-clockwise)
     *
     * @param current Current encoder position (normalized)
     * @param target Target encoder position (normalized)
     * @return Shortest error (positive = clockwise, negative = counter-clockwise)
     */
    private int calculateShortestError(int current, int target) {
        int error = target - current;

        // If error is more than half a rotation, go the other way
        if (error > FULL_ROT / 2) {
            error -= FULL_ROT;  // Subtract full rotation to get shorter path
        } else if (error < -FULL_ROT / 2) {
            error += FULL_ROT;  // Add full rotation to get shorter path
        }

        return error;
    }

    // ========================================================================
    // DRIVER STATION TELEMETRY
    // ========================================================================

    /**
     * Updates the driver station display with current robot status
     * Shows sorter position, chamber status, shooter RPM, and controls
     */
    private void updateTelemetry(int normPos, String shooterColorDetected) {
        // Calculate flywheel status
        double currentRPM = (m3.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);
        boolean rpmReady = (targetRPM > 0) && (rpmError <= RPM_TOLERANCE);

        telemetry.addLine("=== Tracking Mode (B) ===");
        telemetry.addLine();

        // === Sorter Status ===
        telemetry.addLine("=== Sorter ===");
        telemetry.addData("Pos", normPos);                      // Encoder position
        telemetry.addData("Chamber", currentChamber + 1);       // Current chamber (1-3 for display)
        telemetry.addData("Moving", sorterMoving);              // Is sorter moving?

        // Chamber status: O = full, X = empty
        String ch1 = chamberFull[0] ? "O" : "X";
        String ch2 = chamberFull[1] ? "O" : "X";
        String ch3 = chamberFull[2] ? "O" : "X";
        telemetry.addData("Ch1/2/3", ch1 + "/" + ch2 + "/" + ch3);

        telemetry.addData("Mode", shootingMode ? "SHOOT (Y)" : "INTAKE (Y)");
        telemetry.addData("Color", shooterColorDetected);       // Color at shooter
        telemetry.addLine();

        // === Shooter Status ===
        telemetry.addLine("=== Shooter ===");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", String.format("%.0f", currentRPM));
        telemetry.addData("Ready", rpmReady ? "YES" : "NO");    // Is flywheel at speed?
        telemetry.addData("Shooter", gamepad1.a ? "FIRING" : "Ready");
        telemetry.addData("Distance Mode", distanceBasedRPM ? "AUTO (X)" : "MANUAL (X)");
        telemetry.addLine();

        // === Control Reference ===
        telemetry.addLine("=== Controls ===");
        telemetry.addLine("B: Cycle Track Mode");
        telemetry.addLine("Y: Mode | DpadRight: Chamber");
        telemetry.addLine("A: Shoot");
        telemetry.addLine("RB: RPM | LB: 1500 | DpadDown: Off");

        telemetry.update();
    }

    // ========================================================================
    // TURRET CONTROL
    // ========================================================================

    /**
     * Moves turret to desired angle offset relative to robot
     * Uses simple bang-bang control with deadzone tolerance
     *
     * @param turretMotor The turret motor (m2)
     * @param turretDesiredRelativeOffset Desired angle in radians (robot-relative)
     * @return Error in rotations
     */
    private double moveTurretToOffset(DcMotorEx turretMotor, double turretDesiredRelativeOffset) {
        // Convert desired angle to degrees
        double turretDesiredDegrees = Math.toDegrees(turretDesiredRelativeOffset);

        // Get current turret position in rotations
        double turretRotations = turretMotor.getCurrentPosition() / TICKS_PER_REV;

        // Convert desired degrees to rotations
        double desiredRotations = turretDesiredDegrees / 360.0;

        // Calculate error
        double error = desiredRotations - turretRotations;

        // If error is significant, move turret
        if (Math.abs(error) > 0.02) { // 0.02 rotations = ~7.2 degrees
            // Move in direction of error at 30% power
            // Note: Commented line would allow driver to control speed with trigger
            turretMotor.setPower(error / Math.abs(error) * 0.3);
        } else {
            turretMotor.setPower(0);  // Within tolerance, stop
        }

        return error;
    }

    // ========================================================================
    // CHAMBER SORTING OPERATIONS (UNUSED)
    // ========================================================================

    /**
     * Helper class for rotating chamber color arrays
     * Currently unused - was intended for tracking ball colors as chambers rotate
     * Kept for potential future implementation
     */
    private static class ChamberSortingOperations {
        /**
         * Rotates chamber array clockwise
         * Example: [A, B, C] → [C, A, B]
         */
        private String[] rotateClockwise (String[] sorter) {
            String[] out = {sorter[2], sorter[0], sorter[1]};
            return out;
        }

        /**
         * Rotates chamber array counter-clockwise
         * Example: [A, B, C] → [B, C, A]
         */
        private String[] rotateCounterClockwise (String[] sorter) {
            String[] out = {sorter[1], sorter[2], sorter[0]};
            return out;
        }
    }

    // ========================================================================
    // UTILITY FUNCTIONS
    // ========================================================================

    /**
     * Normalizes an angle to the range [-PI, PI]
     * Ensures angle wraps correctly for circular calculations
     * Example: 370° → 10°, -190° → 170°
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Clips very low motor powers to zero
     * Prevents motor stalling/buzzing at low voltages
     * Powers below 4% are set to 0
     */
    private double clipLowPower(double p) {
        return Math.abs(p) < 0.04 ? 0 : p;
    }

    /**
     * Applies deadzone to joystick input
     * Ignores small joystick movements to prevent drift
     * Values below 5% are set to 0
     */
    private double applyDeadzone(double v) {
        return Math.abs(v) < 0.05 ? 0 : v;
    }
}