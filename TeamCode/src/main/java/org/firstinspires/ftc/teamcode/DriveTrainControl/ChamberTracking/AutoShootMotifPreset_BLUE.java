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

import org.firstinspires.ftc.teamcode.Subsystems.Sorter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "[BLUE]---Auto Shoot Motif Preset + Turret Tracking Test", group = "!Test")
public class AutoShootMotifPreset_BLUE extends OpMode {
    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotorEx m1;
    DcMotorEx m2; // Turret motor
    DcMotorEx m3, m0;
    CRServo s3;
    Servo s2;

    // === Color Sensors & RGB LEDs ===
    private RevColorSensorV3 intakeColor;
    private RevColorSensorV3 shooterColor;
    private Servo led1;
    private Servo led2;

    // === Vision & IMU ===
    private Limelight3A limelight;
    private IMU imu;
    private static final String LIMELIGHT_NAME = "Webcam 2";
    private static final int APRILTAG_PIPELINE = 1;

    // === Turret Configuration ===
    private static final double TICKS_PER_REV = 1393.1;

    // === Tracking Mode ===
    private enum TrackingMode {
        OFF,
        LIMELIGHT_AND_ODOMETRY
    }
    private TrackingMode currentTrackingMode = TrackingMode.OFF;
    private boolean lastBButton = false;

    // === Sorter constants ===
    private static final int FULL_ROT = 8192;
    private static final int SLOT = FULL_ROT / 3;
    private static final int OFFSET = (int) (FULL_ROT / 2);
    private static final int CHAMBER_0_POS = 0;
    private static final int CHAMBER_1_POS = SLOT;
    private static final int CHAMBER_2_POS = 2 * SLOT;

    private boolean[] chamberFull = new boolean[3];
    private int currentChamber = 0;
    private boolean shootingMode = false;
    private boolean lastY = false;
    private boolean lastDpadRight = false;

    // === Non-blocking sorter movement ===
    private boolean sorterMoving = false;
    private int sorterTargetPosition = 0;
    private ElapsedTime sorterTimer = new ElapsedTime();
    // === PID State ===
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0;

    // === PID GAINS (Editable in FTC Dashboard) ===
    public static double kP = 0.001;
    public static double kI = 0.0;
    public static double kD = 0.000039;

    private boolean lastXButton = false;
    private boolean distanceBasedRPM = false;
    private double lastValidDistance = 110.0;

    // === Timed color detection ===
    private long colorStartTime = 0;
    private boolean colorActive = false;
    private static final long DETECT_TIME_MS = 25;

    // === Empty chamber detection ===
    private long emptyStartTime = 0;
    private boolean emptyDetectionActive = false;
    private static final long EMPTY_DETECT_TIME_MS = 200;

    // === Shooter presets ===
    private final int[] rpmPresets = {2600 * 2, 3300* 2};
    private int presetIndex = -1;
    private double targetRPM = 0;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadDown = false;

    private static final double TICKS_PER_REV_FLYWHEEL = 28.0;
    private static final double RPM_TOLERANCE = 100.0;

    // === Flywheel PID ===
    private double flywheelKp = 0.0012;
    private double flywheelKi = 0.00001;
    private double flywheelKd = 0.0;
    private double flywheelKF = 0.00025;
    private double flywheelIntegral = 0;
    private double flywheelLastError = 0;
    private long flywheelLastTime = 0;

    // === Auto Shoot Sequence ===
    private boolean autoShootActive = false;
    private int autoShootState = 0;
    private ElapsedTime autoShootTimer = new ElapsedTime();
    private boolean lastAButton = false;

    // Auto shoot timing constants (from autonomous)
    private static final double SHOOT_DURATION = 0.45;
    private static final double SERVO_RETRACT_DELAY = 0.2;
    private static final double SORTER_WAIT_TIME = 0.15;
    private static final double MODE_TOGGLE_WAIT_TIME = 0.75;
    private int shotsComplete = 0;

    private Follower follower;
    public static Pose startingPose;

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
    private enum MOTIF {
        GPP,
        PGP,
        PPG
    }
    private MOTIF currentMotif;

    private ElapsedTime loopTime = new ElapsedTime();

    private boolean runTelemetry = false;
    private ElapsedTime telemetryLimiter = new ElapsedTime();

    @Override
    public void init() {
        startingPose = new Pose(72, 72, Math.toRadians(90));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        // === DriveTrain ===
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "fL");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "bL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "bR");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // === Mechanisms ===
        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m0 = hardwareMap.get(DcMotorEx.class, "m0");

        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");
        s3.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setPosition(.68);

        for (DcMotor motor : new DcMotor[]{m1, m3, m0}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // === Turret Setup ===
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setDirection(DcMotorSimple.Direction.REVERSE);

        // === Color Sensors ===
        intakeColor  = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        shooterColor = hardwareMap.get(RevColorSensorV3.class, "shooterColor");

        // === RGB LEDs ===
        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");
        led1.setPosition(1.0);
        led2.setPosition(1.0);

        // === IMU Setup ===
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        // === Limelight Setup ===
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        flywheelLastTime = System.nanoTime();

        telemetry.addLine("=== Auto Shoot + Turret Tracking Test ===");
        telemetry.addLine("A: Auto Shoot All 3 Balls");
        telemetry.addLine("B: Toggle Turret Tracking");
        telemetry.update();
        currentMotif = MOTIF.GPP;
    }

    public void start() {
        follower.startTeleopDrive();
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentTrackingMode = TrackingMode.LIMELIGHT_AND_ODOMETRY;
    }

    @Override
    public void loop() {
        follower.update();
        loopTime.reset();

        // === Tracking Mode Toggle ===
        boolean bPressed = gamepad1.b;
        if (bPressed && !lastBButton) {
            follower.setPose(new Pose(72, 72, Math.PI / 2));
        }
        lastBButton = bPressed;

        // === Drive Train ===
        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
        double rx = applyDeadzone(gamepad1.right_stick_x);

        double fl = y + x + rx, bl = y - x + rx, fr = y - x - rx, br = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));
        frontLeftMotor.setPower(clipLowPower(fl / max));
        backLeftMotor.setPower(clipLowPower(bl / max));
        frontRightMotor.setPower(clipLowPower(fr / max));
        backRightMotor.setPower(clipLowPower(br / max));

        // === Turret Tracking Logic ===
        if (currentTrackingMode == TrackingMode.LIMELIGHT_AND_ODOMETRY) {
            Pose GOAL_POST = new Pose(10, 134, 0);
            boolean LimelightTracking = false;

            // IF apriltag in view, do LimeLight tracking.
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                LLResultTypes.FiducialResult validTarget = null;
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == 20) {
                        telemetry.addData("Found Tag: ", fiducial.getFiducialId());
                        validTarget = fiducial;
                        LimelightTracking = true;
                        double bearing = validTarget.getTargetXDegrees();
                        double turretRotatePower = 0.067 * bearing / 20.0;

                        if (Math.abs(bearing) > 2) {
                            m2.setPower(turretRotatePower);
                            telemetry.addLine("===TURRET ALIGNED===");
                        } else {
                            m2.setPower(0);
                        }
                        telemetry.addData("Bearing: ", bearing);
                        telemetry.addData("Turret Rotate Power: ", turretRotatePower);
                    }
                }
            }

            // ELSE IF apriltag NOT in view, do odometry position tracking.
            if (LimelightTracking == false) {
                // 1. Calculate component distances from goal
                double y_goal_distance = follower.getPose().getY() - GOAL_POST.getY();
                double x_goal_distance = follower.getPose().getX() - GOAL_POST.getX();

                // 2. Calculate absolute angle to goal in field coordinates
                double angle_to_goal = Math.atan2(y_goal_distance, x_goal_distance);

                // 3. Calculate turret offset relative to robot heading
                // Normalize the angle difference to [-PI, PI]
                double turretDesiredRelativeOffset = normalizeAngle(-angle_to_goal + follower.getHeading() + Math.PI);

                // 4. Move turret to track the goal
                double error = moveTurretToOffset(m2, turretDesiredRelativeOffset);
            }
        } else {
            // Tracking OFF - hold position
            m2.setPower(0);
        }

        // === Switch through Motif States ===
        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) {
            switch (currentMotif) {
                case GPP:
                    currentMotif = MOTIF.PGP;
                    return;
                case PGP:
                    currentMotif = MOTIF.PPG;
                    return;
                case PPG:
                    currentMotif = MOTIF.GPP;
                    return;
            }
        }
        lastY = yPressed;


        // === Sorter ===
        int rawPos = backRightMotor.getCurrentPosition();
        int normPos = normalize(rawPos);

        boolean dpadRightPressed = gamepad1.dpad_right;
        boolean dpadLeftPressed = gamepad1.dpad_left;
        if (dpadRightPressed && !lastDpadRight && !autoShootActive) {
            currentChamber = nextChamber(currentChamber);
            rotateChamberColorsClockwise();
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            startSorterMove(targetPos);
        }
        else if (dpadLeftPressed && !lastDpadLeft && !autoShootActive) {
            currentChamber = prevChamber(currentChamber);
            rotateChamberColorsCounterClockwise();
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            startSorterMove(targetPos);
        }
        lastDpadRight = dpadRightPressed;
        lastDpadLeft = dpadLeftPressed;

        updateSorterPIDMove();

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

        // === Intake ===
        if (!shootingMode && !autoShootActive) {
            double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
            m1.setPower(triggerPower);
        } else {
            m1.setPower(0);
        }

        // === Auto Shoot Sequence Trigger ===
        boolean aPressed = gamepad1.a;
        if (aPressed && !lastAButton && !autoShootActive) {
            // Start auto shoot sequence
            autoShootActive = true;
            autoShootState = 0;
            shotsComplete = 0;
            autoShootTimer.reset();

            // Make sure we're in shooting mode
            if (!shootingMode) {
                shootingMode = true;
                // Find which chamber (A, B, or C) has the green ball
                int greenIndex = indexOfColor(chamberColors, "GREEN", true);

                // Figure out how much extra to turn chamber, based on motif
                int rotationCompensateForMotif = 0;
                if (currentMotif.equals(MOTIF.GPP)) {
                    rotationCompensateForMotif = -1;
                }
                if (currentMotif.equals(MOTIF.PGP)) {
                    rotationCompensateForMotif = 1;
                }
                if (currentMotif.equals(MOTIF.PPG)) {
                    rotationCompensateForMotif = 0;
                }

                if (greenIndex != -1) {  // Found a green ball (or next best if no green)
                    // Calculate how many rotations needed to bring that chamber to position A
                    int rotationsNeeded = 0 + rotationCompensateForMotif;

                    if (greenIndex == 0) {
                        // Green is already in A, no rotation needed
                        rotationsNeeded = 0 + rotationCompensateForMotif;
                    } else if (greenIndex == 1) {
                        // Green is in B, need to rotate CCW once to make B→A
                        rotationsNeeded = -1 + rotationCompensateForMotif;  // Negative = counter-clockwise
                    } else if (greenIndex == 2) {
                        // Green is in C, need to rotate CW once to make C→A
                        // OR rotate CCW twice (but CW is shorter)
                        rotationsNeeded = -2 + rotationCompensateForMotif;  // Positive = clockwise
                    }

                    // Apply the rotations to the array
                    for (int i = 0; i < Math.abs(rotationsNeeded); i++) {
                        if (rotationsNeeded > 0) {
                            rotateChamberColorsClockwise(); // rotates chamberColors[] clockwise
                            currentChamber = nextChamber(currentChamber); // CW rotation = prev chamber
                        } else if (rotationsNeeded < 0) {
                            rotateChamberColorsCounterClockwise(); // rotates chamberColors[] counterclockwise
                            currentChamber = prevChamber(currentChamber); // CCW rotation = next chamber
                        }
                    }

                    // Calculate the new target position in shooting mode
                    int targetPos = getChamberPosition(currentChamber, shootingMode);
                    startSorterMove(targetPos);
                }
            }
        }
        lastAButton = aPressed;

        // === Auto Shoot State Machine ===
        if (autoShootActive) {
            updateAutoShootSequence();
        }

        // === Flywheel RPM ===
        // ====================================================================
        // X BUTTON: ROTATE TO GREEN BALL (SHOOTING MODE ONLY)
        // ====================================================================

        // X button finds and rotates to green ball in shooting mode
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastXButton && shootingMode) {
        }
        lastXButton = xPressed;


        if (gamepad1.right_bumper && !lastRightBumper) {
            if (distanceBasedRPM) {
                double distance = getAprilTagDistance();
                if (distance > 0) {
                    lastValidDistance = distance;
                    targetRPM = (distance < 110.0) ? 2500 : 3000;
                } else {
                    targetRPM = (lastValidDistance < 110.0) ? 2500 : 3000;
                }
            } else {
                presetIndex = (presetIndex + 1) % rpmPresets.length;
                targetRPM = rpmPresets[presetIndex];
            }
        } else if (gamepad1.dpad_down && !lastDpadDown) {
            targetRPM = 0;
        }

        boolean leftBumperPressed = gamepad1.left_bumper;
        if (leftBumperPressed && !lastLeftBumper) {
            targetRPM = 1500;
        }

        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = leftBumperPressed;
        lastDpadDown = gamepad1.dpad_down;

        // === Flywheel PID Control ===
        double currentVelocity = m3.getVelocity();
        double currentRPM = (currentVelocity / TICKS_PER_REV_FLYWHEEL) * 60.0;

        long currentTime = System.nanoTime();
        double dt = (currentTime - flywheelLastTime) / 1e9;

        double error = targetRPM - currentRPM;

        flywheelIntegral += error * dt;
        flywheelIntegral = Math.max(-10000, Math.min(10000, flywheelIntegral));

        double derivative = (error - flywheelLastError) / dt;
        double feedforward = flywheelKF * targetRPM;
        double pidOutput = (flywheelKp * error) + (flywheelKi * flywheelIntegral) +
                (flywheelKd * derivative) + feedforward;

        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
        m3.setPower(pidOutput);

        flywheelLastError = error;
        flywheelLastTime = currentTime;

        updateRPMLED();

        // Update driver station telemetry
        updateTelemetry(normPos, shooterColorDetected);

        // === Pose Reset System ===
        if (gamepad1.dpad_up) {
            follower.setPose(new Pose(72, 72, Math.PI / 2));
        }

        loopTime.reset();
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
    private void updateSorterPIDMove() {
        if (!sorterMoving) return;

        // Get current position and calculate error to target
        int rawPos = backRightMotor.getCurrentPosition();
        int pos = normalize(rawPos);
        int error = calculateShortestError(pos, sorterTargetPosition);


        // Calculate time delta
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }

        // PID terms
        double pTerm = kP * error;

        integral += error * dt;
        integral = Math.max(-5000, Math.min(5000, integral)); // Anti-windup
        double iTerm = kI * integral;

        double derivative = (error - lastError) / dt;
        double dTerm = kD * derivative;

        lastError = error;

        // Total output
        double power = pTerm + iTerm + dTerm;

        // Clamp output
        power = Math.max(-1.0, Math.min(1.0, power));

        m0.setPower(power);
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
     * In shooting mode, adds 180-degree offset to align chamber A with shooter
     *
     * @param chamber Which chamber (0, 1, or 2)
     * @param shooting Whether in shooting mode (true) or intake mode (false)
     * @return Target encoder position in ticks
     */
    /**
     * Calculates the encoder position for a given chamber
     * In shooting mode, adds 180-degree offset to align chamber A with shooter
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
                chamberColors[0] = detected;                  // Mark chamber A color (at intake position)
                rotateChamberColorsClockwise();
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
            chamberColors[0] = "NONE";           // Mark chamber A color (at shooter position)
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
    private void setAllChambersEmpty() {
        chamberFull[0] = false;
        chamberFull[1] = false;
        chamberFull[2] = false;
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
    /**
     * Gets the previous chamber in the nextChamber sequence
     * Since nextChamber goes: 0→2→1→0
     * prevChamber goes backwards: 0→1→2→0
     *
     * @param c Current chamber (0, 1, or 2)
     * @return Previous chamber
     */
    private int prevChamber(int c) {
        if (c == 0) return 1;  // Backwards from 0 is 1
        if (c == 1) return 2;  // Backwards from 1 is 2
        return 0;              // c == 2, backwards is 0
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
    /**
     * Auto shoot sequence - shoots all 3 balls automatically
     * Based on autonomous shooting sequence
     */
    private void updateAutoShootSequence() {
        switch (autoShootState) {
            case 0: // Wait for mode toggle to complete
                if (autoShootTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    rotateSorter();
                    rotateChamberColorsClockwise();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 1: // Wait for sorter rotation
                if (autoShootTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 2: // Shoot ball 1
                if (autoShootTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    chamberColors[0] = "NONE";
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 3: // Wait for servo retract
                if (autoShootTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
                    rotateChamberColorsClockwise();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 4: // Wait for sorter rotation
                if (autoShootTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 5: // Shoot ball 2
                if (autoShootTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    chamberColors[0] = "NONE";
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 6: // Wait for servo retract
                if (autoShootTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
                    rotateChamberColorsClockwise();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 7: // Wait for sorter rotation
                if (autoShootTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 8: // Shoot ball 3
                if (autoShootTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    chamberColors[0] = "NONE";
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 9: // Wait for servo retract, then back to intake mode
                if (autoShootTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    toggleShootingMode();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 10: // Wait for mode toggle, then finish
                if (autoShootTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    autoShootActive = false;
                    autoShootState = 0;
                    shootingMode = false;
                    setAllChambersEmpty();
                    gamepad1.rumble(500); // Signal completion
                }
                break;
        }
    }

    // === Shooting Helper Methods (from autonomous) ===

    private void toggleShootingMode() {
        shootingMode = !shootingMode;
        int target = getChamberPosition(currentChamber, shootingMode);
        startSorterMove(target);
    }

    private void rotateSorter() {
        currentChamber = nextChamber(currentChamber);
        int target = getChamberPosition(currentChamber, shootingMode);
        startSorterMove(target);
    }

    private void activateShooter() {
        s2.setPosition(0);
        s3.setPower(1.0);
    }

    private void deactivateShooter() {
        s2.setPosition(0.68);
        s3.setPower(0.0);
    }

    private void updateTelemetry(int normPos, String shooterColorDetected) {
        if (runTelemetry) {
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

            String ch1 = chamberFull[0] ? chamberColors[0] : "X";
            String ch2 = chamberFull[1] ? chamberColors[1] : "X";
            String ch3 = chamberFull[2] ? chamberColors[2] : "X";
            telemetry.addData("Ch1/2/3", chamberColors[0] + "/" + chamberColors[1] + "/" + chamberColors[2]);
            telemetry.addData("Current Motif State: ", currentMotif);
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

            telemetry.addData("Loop Time: ", 1.0 / loopTime.seconds());
        }
        if (telemetryLimiter.seconds() > 0.5) {
            // === Update Loop Time Tracking ===
            telemetry.addData("Loop Time (Hz)", 1.0 / loopTime.seconds());

            telemetry.update();
            telemetryLimiter.reset();
        }
    }

    // === Turret Movement Helper ===

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
        if (Math.abs(error) > 0.02) { // 0.01 rotations = ~5.4 degrees
            // Move in direction of error at 30% power
            // Note: Commented line would allow driver to control speed with trigger
            turretMotor.setPower(error / Math.abs(error) * 0.3);
        } else {
            turretMotor.setPower(0);  // Within tolerance, stop
        }

        return error;
    }

    // ========================================================================
    // CHAMBER SORTING OPERATIONS
    // ========================================================================

    /**
     * Helper class for rotating chamber color arrays
     * Currently unused - was intended for tracking ball colors as chambers rotate
     * Kept for potential future implementation
     */
    /**
     * Rotates chamber array clockwise
     * Example: [A, B, C] → [C, A, B]
     */
    private void rotateChamberColorsClockwise () {
        String[] out = {chamberColors[0], chamberColors[1], chamberColors[2]};
        chamberColors[0] = out[2];
        chamberColors[1] = out[0];
        chamberColors[2] = out[1];
    }

    /**
     * Rotates chamber array counter-clockwise
     * Example: [A, B, C] → [B, C, A]
     */
    private void rotateChamberColorsCounterClockwise () {
        String[] out = {chamberColors[0], chamberColors[1], chamberColors[2]};
        chamberColors[0] = out[1];
        chamberColors[1] = out[2];
        chamberColors[2] = out[0];
    }

    /**
     * Finds the index of chamber with ball of desired color, based on A (0), B (1), or C (2)
     * If no chamber has the desired ball color, returns -1
     * bool returnNextBest: returns the next best chamber if desired color is not found
     */
    private int indexOfColor (String[] chamberColors, String desiredColor, boolean returnNextBest) {
        for (int i = 0; i <= 2; i++) {
            if (chamberColors[i].equals(desiredColor)){
                return i;
            }
        }
        if (returnNextBest) {
            for (int i = 0; i<= 2; i++) {
                if (!chamberColors[i].equals("NONE")) {
                    return i;
                }
            }
        }
        return -1;
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