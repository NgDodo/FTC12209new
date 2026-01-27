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

import java.util.List;

@TeleOp(name = "Chamber Tracking [Blue] - Fixed", group = "DriveTrainControl")
public class ChamberTrackingBlueFixed extends OpMode {

    // ========================================================================
    // HARDWARE DECLARATIONS
    // ========================================================================

    // === Drive Train Motors ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // === Mechanism Motors ===
    DcMotor m1;      // Intake motor
    DcMotorEx m2;    // Turret rotation motor
    DcMotorEx m3;    // Flywheel motor
    DcMotorEx m0;    // Sorter motor

    // === Servos ===
    CRServo s3;      // Continuous rotation servo - flicker mechanism
    Servo s2;        // Standard servo - flicker gate

    // === Color Sensors ===
    private RevColorSensorV3 intakeColor;
    private RevColorSensorV3 shooterColor;

    // === RGB LED Indicators ===
    private Servo led1;  // LED strip 1 - ball color
    private Servo led2;  // LED strip 2 - RPM status

    // === Vision & Sensors ===
    private Limelight3A limelight;
    private IMU imu;

    // ========================================================================
    // LIMELIGHT VISION CONFIGURATION
    // ========================================================================

    private static final String LIMELIGHT_NAME = "Webcam 2";
    private static final int APRILTAG_PIPELINE = 1;

    // === Turret Return to Center ===
    private boolean turretWrapping = false;

    // ========================================================================
    // TRACKING MODE SYSTEM
    // ========================================================================

    private enum TrackingMode {
        OFF,
        BODY_TRACK,
        TURRET_TRACK
    }
    private TrackingMode currentTrackingMode = TrackingMode.OFF;

    // ========================================================================
    // TURRET MOTOR CONFIGURATION
    // ========================================================================

    private static final double TURRET_RANGE_DEG = 330.0;
    private static final double TICKS_PER_REV = 1393.1;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;

    // === Turret PID Control ===
    private double turretLastError = 0;

    // === Body Tracking PID ===
    private double bodyLastError = 0.0;
    private static final double BODY_TOLERANCE_DEG = 0.6;

    // === Smoothing & Prediction ===
    private double smoothedBearing = 0.0;
    private boolean hasEverSeenTag = false;

    // ========================================================================
    // SORTER SYSTEM CONFIGURATION
    // ========================================================================

    private static final int FULL_ROT = 8192;
    private static final int SLOT = FULL_ROT / 3;
    private static final int OFFSET = (FULL_ROT / 2);

    private static final int CHAMBER_0_POS = 0;
    private static final int CHAMBER_1_POS = SLOT;
    private static final int CHAMBER_2_POS = 2 * SLOT;

    // === Chamber State Tracking ===
    private boolean[] chamberFull = new boolean[3];
    private int currentChamber = 0;
    private boolean shootingMode = false;

    // Button state tracking
    private boolean lastY = false;
    private boolean lastDpadRight = false;

    // ========================================================================
    // NON-BLOCKING SORTER MOVEMENT SYSTEM
    // ========================================================================

    private boolean sorterMoving = false;
    private int sorterTargetPosition = 0;
    private ElapsedTime sorterTimer = new ElapsedTime();
    private ElapsedTime sorterSettleTimer = new ElapsedTime();
    private boolean sorterSettling = false;

    private static final int COARSE_TOL = 1000;
    private static final int FINE_TOL = 60;
    private static final int PERFECT_TOL = 30;
    private static final double MAX_POWER = 0.55;
    private static final double MIN_POWER = 0.08;
    private static final long SORTER_TIMEOUT_MS = 2000;
    private static final long SETTLE_MS = 100;

    // ========================================================================
    // SHOOTER RPM CONTROL
    // ========================================================================

    private boolean lastXButton = false;
    private boolean distanceBasedRPM = false;
    private double lastValidDistance = 110.0;

    // ========================================================================
    // TIMED COLOR DETECTION
    // ========================================================================

    private long colorStartTime = 0;
    private boolean colorActive = false;
    private static final long DETECT_TIME_MS = 25;

    // ========================================================================
    // EMPTY CHAMBER DETECTION
    // ========================================================================

    private long emptyStartTime = 0;
    private boolean emptyDetectionActive = false;
    private static final long EMPTY_DETECT_TIME_MS = 200;

    // ========================================================================
    // SHOOTER FLYWHEEL PRESETS
    // ========================================================================

    private final int[] rpmPresets = {2600, 3300};
    private int presetIndex = -1;
    private double targetRPM = 0;

    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastDpadLeft = false;

    private static final double TICKS_PER_REV_FLYWHEEL = 28.0;
    private static final double RPM_TOLERANCE = 100.0;
    private boolean lastDpadDown = false;

    // ========================================================================
    // FLYWHEEL PID CONTROLLER
    // ========================================================================

    private double flywheelKp = 0.0012;
    private double flywheelKi = 0.00001;
    private double flywheelKd = 0.0;
    private double flywheelKF = 0.00025;

    private double flywheelIntegral = 0;
    private double flywheelLastError = 0;
    private long flywheelLastTime = 0;

    // ========================================================================
    // PEDRO PATHING (ODOMETRY) SYSTEM
    // ========================================================================

    private Follower follower;
    public static Pose startingPose;

    // ========================================================================
    // CHAMBER COLOR TRACKING SYSTEM
    // ========================================================================

    private String[] chamberColors = {"NONE", "NONE", "NONE"};

    // ========================================================================
    // INITIALIZATION
    // ========================================================================

    @Override
    public void init() {
        // === Pedro Pathing Setup ===
        startingPose = new Pose(20.9, 123.1, Math.toRadians(144));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        // === DriveTrain Initialization ===
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

        // === Mechanism Motors Initialization ===
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m0 = hardwareMap.get(DcMotorEx.class, "m0");

        // === Servo Initialization ===
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");
        s3.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setPosition(.68);

        for (DcMotor motor : new DcMotor[]{m1, m3, m0}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // === Turret Motor Setup ===
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === Flywheel Motor Setup ===
        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setDirection(DcMotorSimple.Direction.REVERSE);

        // === Color Sensor Initialization ===
        intakeColor  = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        shooterColor = hardwareMap.get(RevColorSensorV3.class, "shooterColor");

        // === RGB LED Initialization ===
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

        // === Limelight Vision Setup ===
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        flywheelLastTime = System.nanoTime();

        telemetry.addLine("Integrated Turret Tracking Initialized");
        telemetry.addLine("B: Cycle Tracking Mode (OFF/Body/Turret)");
        telemetry.addLine("X: Rotate to Green Ball (FIXED)");
        telemetry.update();
    }

    // ========================================================================
    // START
    // ========================================================================

    public void start() {
        follower.startTeleopDrive();
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ========================================================================
    // MAIN LOOP
    // ========================================================================

    @Override
    public void loop() {
        follower.update();

        // ====================================================================
        // MECANUM DRIVE CONTROL
        // ====================================================================

        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
        double rx = applyDeadzone(gamepad1.right_stick_x);

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeftMotor.setPower(clipLowPower(fl / max));
        backLeftMotor.setPower(clipLowPower(bl / max));
        frontRightMotor.setPower(clipLowPower(fr / max));
        backRightMotor.setPower(clipLowPower(br / max));

        // ====================================================================
        // SORTER CONTROL
        // ====================================================================

        int rawPos = backRightMotor.getCurrentPosition();
        int normPos = normalize(rawPos);

        // === Y Button: Toggle Shooting/Intake Mode ===
        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) {
            shootingMode = !shootingMode;
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            startSorterMove(targetPos);
        }
        lastY = yPressed;

        // === DPad Right: Rotate to Next Chamber ===
        boolean dpadRightPressed = gamepad1.dpad_right;
        if (dpadRightPressed && !lastDpadRight) {
            currentChamber = nextChamber(currentChamber);
            rotateChamberColorsClockwise();
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            startSorterMove(targetPos);
        }
        lastDpadRight = dpadRightPressed;

        updateSorterMovement();

        // ====================================================================
        // COLOR DETECTION
        // ====================================================================

        if (!shootingMode) {
            autoIntakeColorCheck();
        } else {
            checkChamberEmpty();
        }

        String shooterColorDetected = detectShooterColor();
        updateColorLEDs(shooterColorDetected);

        // ====================================================================
        // INTAKE CONTROL
        // ====================================================================

        if (!shootingMode) {
            double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
            m1.setPower(triggerPower);
        } else {
            m1.setPower(0);
        }

        // ====================================================================
        // SHOOTER/FLICKER CONTROL
        // ====================================================================

        if (gamepad1.a) {
            s2.setPosition(0);
            s3.setPower(1.0);
        } else {
            s2.setPosition(.68);
            s3.setPower(0.0);
        }

        // ====================================================================
        // X BUTTON: ROTATE TO GREEN BALL (SHOOTING MODE ONLY) - FIXED
        // ====================================================================

        boolean xPressed = gamepad1.x;
        if (xPressed && !lastXButton && shootingMode) {
            // Find which chamber has the green ball
            int greenIndex = indexOfColor(chamberColors, "GREEN", true);

            if (greenIndex != -1) {
                int rotationsNeeded = 0;

                if (greenIndex == 0) {
                    // Green is already at shooter position A
                    rotationsNeeded = 0;
                } else if (greenIndex == 1) {
                    // Green is at position B
                    // Rotate PHYSICALLY CLOCKWISE once to make B→A
                    rotationsNeeded = 1;  // FIXED: Was -1, now 1
                } else if (greenIndex == 2) {
                    // Green is at position C
                    // Rotate PHYSICALLY COUNTER-CLOCKWISE once to make C→A
                    rotationsNeeded = -1;  // FIXED: Was 1, now -1
                }

                // Apply the rotations
                for (int i = 0; i < Math.abs(rotationsNeeded); i++) {
                    if (rotationsNeeded > 0) {
                        // Physical CLOCKWISE rotation
                        // Array rotates COUNTER-CLOCKWISE to track what's now at A
                        rotateChamberColorsCounterClockwise();
                        currentChamber = prevChamber(currentChamber);
                    } else if (rotationsNeeded < 0) {
                        // Physical COUNTER-CLOCKWISE rotation
                        // Array rotates CLOCKWISE to track what's now at A
                        rotateChamberColorsClockwise();
                        currentChamber = nextChamber(currentChamber);
                    }
                }

                // Move sorter to new position
                int targetPos = getChamberPosition(currentChamber, shootingMode);
                startSorterMove(targetPos);
            }
        }
        lastXButton = xPressed;

        // ====================================================================
        // FLYWHEEL RPM PRESETS
        // ====================================================================

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
        } else if (gamepad1.dpad_left && !lastDpadLeft) {
            targetRPM = -4000;
        }

        boolean leftBumperPressed = gamepad1.left_bumper;
        if (leftBumperPressed && !lastLeftBumper) {
            targetRPM = 1500;
        }

        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = leftBumperPressed;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadDown = gamepad1.dpad_down;

        // ====================================================================
        // FLYWHEEL PID CONTROL
        // ====================================================================

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
        updateTelemetry(normPos, shooterColorDetected);

        // ====================================================================
        // TURRET TRACKING
        // ====================================================================

        Pose GOAL_POST = new Pose(10, 134, 0);

        double y_goal_distance = follower.getPose().getY() - GOAL_POST.getY();
        double x_goal_distance = follower.getPose().getX() - GOAL_POST.getX();

        double angle_to_goal = Math.atan2(y_goal_distance, x_goal_distance);

        double turretDesiredRelativeOffset = normalizeAngle(-angle_to_goal + follower.getHeading() + Math.PI);

        moveTurretToOffset(m2, turretDesiredRelativeOffset);

        // ====================================================================
        // POSE RESET
        // ====================================================================

        if (gamepad1.dpad_up) {
            follower.setPose(new Pose(72, 72, Math.PI / 2));
        }
    }

    // ========================================================================
    // APRILTAG DISTANCE MEASUREMENT
    // ========================================================================

    private double getAprilTagDistance() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24) {
                    double area = fiducial.getTargetArea();
                    if (area > 3.0) {
                        return 80.0;
                    } else {
                        return 120.0;
                    }
                }
            }
        }
        return -1;
    }

    // ========================================================================
    // RPM LED INDICATOR
    // ========================================================================

    private void updateRPMLED() {
        if (!shootingMode) {
            String intakeDetected = detectIntakeColor();
            if (!intakeDetected.equals("NONE")) {
                led2.setPosition(1.0);
                return;
            }
        }

        if (targetRPM == 0) {
            led2.setPosition(0);
            return;
        }

        double currentRPM = (m3.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);

        if (rpmError <= RPM_TOLERANCE) {
            led2.setPosition(0.3);
        } else {
            led2.setPosition(0);
        }
    }

    // ========================================================================
    // SORTER MOVEMENT CONTROL
    // ========================================================================

    private void updateSorterMovement() {
        if (!sorterMoving) return;

        int pos = normalize(backRightMotor.getCurrentPosition());
        int error = calculateShortestError(pos, sorterTargetPosition);

        if (sorterTimer.milliseconds() > SORTER_TIMEOUT_MS) {
            m0.setPower(0);
            sorterMoving = false;
            sorterSettling = false;
            return;
        }

        int FAST_PERFECT_TOL = 50;
        int FAST_FINE_TOL = 100;

        if (Math.abs(error) <= FAST_PERFECT_TOL) {
            if (!sorterSettling) {
                sorterSettling = true;
                sorterSettleTimer.reset();
                m0.setPower(0);
            }

            long FAST_SETTLE_MS = 30;
            if (sorterSettleTimer.milliseconds() >= FAST_SETTLE_MS) {
                m0.setPower(0);
                sorterMoving = false;
                sorterSettling = false;
                return;
            }

            if (Math.abs(error) > FAST_FINE_TOL) {
                sorterSettling = false;
            } else {
                return;
            }
        } else {
            sorterSettling = false;
        }

        double power;
        int absError = Math.abs(error);

        if (absError > COARSE_TOL) {
            power = MAX_POWER;
        } else {
            double ratio = (double) absError / COARSE_TOL;
            power = MIN_POWER + (MAX_POWER - MIN_POWER) * ratio;
            power = Math.max(MIN_POWER, Math.min(MAX_POWER, power));
        }

        m0.setPower(Math.signum(error) * power);
    }

    private void startSorterMove(int targetPosition) {
        sorterTargetPosition = targetPosition;
        sorterMoving = true;
        sorterSettling = false;
        sorterTimer.reset();
    }

    // ========================================================================
    // COLOR LED INDICATOR
    // ========================================================================

    private void updateColorLEDs(String color) {
        if (color.equals("GREEN")) {
            led1.setPosition(0.5);
        } else if (color.equals("PURPLE")) {
            led1.setPosition(0.722);
        } else {
            led1.setPosition(0);
        }
    }

    // ========================================================================
    // CHAMBER POSITION CALCULATOR
    // ========================================================================

    private int getChamberPosition(int chamber, boolean shooting) {
        int basePos;
        switch(chamber) {
            case 0: basePos = CHAMBER_0_POS; break;
            case 1: basePos = CHAMBER_1_POS; break;
            case 2: basePos = CHAMBER_2_POS; break;
            default: basePos = CHAMBER_0_POS;
        }

        if (shooting) basePos = normalize(basePos + OFFSET);

        return basePos;
    }

    // ========================================================================
    // AUTOMATIC INTAKE COLOR DETECTION
    // ========================================================================

    private void autoIntakeColorCheck() {
        if (sorterMoving) return;

        String detected = detectIntakeColor();

        if (detected.equals("NONE")) {
            colorActive = false;
            colorStartTime = 0;
            return;
        }

        if (!colorActive) {
            colorActive = true;
            colorStartTime = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - colorStartTime >= DETECT_TIME_MS) {
            if (!chamberFull[currentChamber]) {
                chamberFull[currentChamber] = true;
                chamberColors[0] = detected;
                currentChamber = nextChamber(currentChamber);

                int target = getChamberPosition(currentChamber, false);
                startSorterMove(target);
            }

            colorActive = false;
            colorStartTime = 0;
        }
    }

    // ========================================================================
    // CHAMBER EMPTY DETECTION
    // ========================================================================

    private void checkChamberEmpty() {
        if (!shootingMode || !chamberFull[currentChamber]) {
            emptyDetectionActive = false;
            emptyStartTime = 0;
            return;
        }

        String detected = detectShooterColor();

        if (!detected.equals("NONE")) {
            emptyDetectionActive = false;
            emptyStartTime = 0;
            return;
        }

        if (!emptyDetectionActive) {
            emptyDetectionActive = true;
            emptyStartTime = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - emptyStartTime >= EMPTY_DETECT_TIME_MS) {
            chamberFull[currentChamber] = false;
            chamberColors[0] = "NONE";
            emptyDetectionActive = false;
            emptyStartTime = 0;
        }
    }

    // ========================================================================
    // COLOR DETECTION FUNCTIONS
    // ========================================================================

    private String detectIntakeColor() {
        int r = intakeColor.red();
        int g = intakeColor.green();
        int b = intakeColor.blue();

        if (g > r && g > b && g > 80 && g < 600) return "GREEN";
        if (b > r && b > g && b > 80 && b < 600) return "PURPLE";
        return "NONE";
    }

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

    private boolean allChambersFull() {
        return chamberFull[0] && chamberFull[1] && chamberFull[2];
    }

    // ========================================================================
    // CHAMBER ROTATION SEQUENCE
    // ========================================================================

    private int nextChamber(int c) {
        if (c == 1) return 0;
        if (c == 0) return 2;
        return 1;
    }

    private int prevChamber(int c) {
        if (c == 0) return 1;
        if (c == 1) return 2;
        return 0;
    }

    // ========================================================================
    // ENCODER POSITION NORMALIZATION
    // ========================================================================

    private int normalize(int ticks) {
        return ((ticks % FULL_ROT) + FULL_ROT) % FULL_ROT;
    }

    // ========================================================================
    // SHORTEST PATH CALCULATION
    // ========================================================================

    private int calculateShortestError(int current, int target) {
        int error = target - current;

        if (error > FULL_ROT / 2) {
            error -= FULL_ROT;
        } else if (error < -FULL_ROT / 2) {
            error += FULL_ROT;
        }

        return error;
    }

    // ========================================================================
    // DRIVER STATION TELEMETRY
    // ========================================================================

    private void updateTelemetry(int normPos, String shooterColorDetected) {
        double currentRPM = (m3.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);
        boolean rpmReady = (targetRPM > 0) && (rpmError <= RPM_TOLERANCE);

        telemetry.addLine("=== Tracking Mode (B) ===");
        telemetry.addLine();

        telemetry.addLine("=== Sorter ===");
        telemetry.addData("Pos", normPos);
        telemetry.addData("Chamber", currentChamber + 1);
        telemetry.addData("Moving", sorterMoving);

        String ch1 = chamberFull[0] ? chamberColors[0] : "X";
        String ch2 = chamberFull[1] ? chamberColors[1] : "X";
        String ch3 = chamberFull[2] ? chamberColors[2] : "X";
        telemetry.addData("Ch1/2/3", ch1 + "/" + ch2 + "/" + ch3);

        telemetry.addData("Mode", shootingMode ? "SHOOT (Y)" : "INTAKE (Y)");
        telemetry.addData("Color", shooterColorDetected);
        telemetry.addLine();

        telemetry.addLine("=== Shooter ===");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", String.format("%.0f", currentRPM));
        telemetry.addData("Ready", rpmReady ? "YES" : "NO");
        telemetry.addData("Shooter", gamepad1.a ? "FIRING" : "Ready");
        telemetry.addData("Distance Mode", distanceBasedRPM ? "AUTO (X)" : "MANUAL (X)");
        telemetry.addLine();

        telemetry.addLine("=== Controls ===");
        telemetry.addLine("B: Cycle Track Mode");
        telemetry.addLine("Y: Mode | DpadRight: Chamber");
        telemetry.addLine("X: Rotate to Green (FIXED)");
        telemetry.addLine("A: Shoot");
        telemetry.addLine("RB: RPM | LB: 1500 | DpadDown: Off");

        telemetry.update();
    }

    // ========================================================================
    // TURRET CONTROL
    // ========================================================================

    private double moveTurretToOffset(DcMotorEx turretMotor, double turretDesiredRelativeOffset) {
        double turretDesiredDegrees = Math.toDegrees(turretDesiredRelativeOffset);
        double turretRotations = turretMotor.getCurrentPosition() / TICKS_PER_REV;
        double desiredRotations = turretDesiredDegrees / 360.0;
        double error = desiredRotations - turretRotations;

        if (Math.abs(error) > 0.02) {
            turretMotor.setPower(error / Math.abs(error) * 0.3);
        } else {
            turretMotor.setPower(0);
        }

        return error;
    }

    // ========================================================================
    // CHAMBER SORTING OPERATIONS
    // ========================================================================

    private void rotateChamberColorsClockwise() {
        String[] out = {chamberColors[0], chamberColors[1], chamberColors[2]};
        chamberColors[0] = out[2];
        chamberColors[1] = out[0];
        chamberColors[2] = out[1];
    }

    private void rotateChamberColorsCounterClockwise() {
        String[] out = {chamberColors[0], chamberColors[1], chamberColors[2]};
        chamberColors[0] = out[1];
        chamberColors[1] = out[2];
        chamberColors[2] = out[0];
    }

    private int indexOfColor(String[] chamberColors, String desiredColor, boolean returnNextBest) {
        for (int i = 0; i <= 2; i++) {
            if (chamberColors[i].equals(desiredColor)) {
                return i;
            }
        }
        if (returnNextBest) {
            for (int i = 0; i <= 2; i++) {
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

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clipLowPower(double p) {
        return Math.abs(p) < 0.04 ? 0 : p;
    }

    private double applyDeadzone(double v) {
        return Math.abs(v) < 0.05 ? 0 : v;
    }
}