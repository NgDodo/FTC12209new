package org.firstinspires.ftc.teamcode.DriveTrainControl;

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

@TeleOp(name = "Auto Shoot + Turret Tracking Test", group = "Test")
public class AutoShootTurretTest extends OpMode {
    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1;
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
    private static final int OFFSET = (int) (FULL_ROT / 6);
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
    private ElapsedTime sorterSettleTimer = new ElapsedTime();
    private boolean sorterSettling = false;
    private static final int COARSE_TOL = 1000;
    private static final double MAX_POWER = 0.55;
    private static final double MIN_POWER = 0.08;
    private static final long SORTER_TIMEOUT_MS = 2000;

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
    private final int[] rpmPresets = {2600, 3300};
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
    private static final double SHOOT_DURATION = 0.3;
    private static final double SERVO_RETRACT_DELAY = 0.2;
    private static final double SORTER_WAIT_TIME = 0.15;
    private static final double MODE_TOGGLE_WAIT_TIME = 0.75;
    private int shotsComplete = 0;

    private Follower follower;
    public static Pose startingPose;

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
        m1 = hardwareMap.get(DcMotor.class, "m1");
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
    }

    public void start() {
        follower.startTeleopDrive();
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        follower.update();

        // === Tracking Mode Toggle ===
        boolean bPressed = gamepad1.b;
        if (bPressed && !lastBButton) {
            if (currentTrackingMode == TrackingMode.OFF) {
                currentTrackingMode = TrackingMode.LIMELIGHT_AND_ODOMETRY;
                gamepad1.rumble(100);
            } else {
                currentTrackingMode = TrackingMode.OFF;
                m2.setPower(0);
                gamepad1.rumble(50);
            }
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

                telemetry.addData("Angle to Goal", "%.1f°", Math.toDegrees(angle_to_goal));
                telemetry.addData("Rotations to Goal", "%.2f", Math.toDegrees(angle_to_goal) / 360.0);
                telemetry.addData("Turret Relative Offset", "%.1f°", Math.toDegrees(turretDesiredRelativeOffset));
                telemetry.addData("Turret Rotations Error", error);
                telemetry.addData("Turret Rotations", "%.2f", m2.getCurrentPosition() / TICKS_PER_REV);
            }
        } else {
            // Tracking OFF - hold position
            m2.setPower(0);
        }

        // === Sorter ===
        int rawPos = backRightMotor.getCurrentPosition();
        int normPos = normalize(rawPos);

        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY && !autoShootActive) {
            shootingMode = !shootingMode;
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            startSorterMove(targetPos);
        }
        lastY = yPressed;

        boolean dpadRightPressed = gamepad1.dpad_right;
        if (dpadRightPressed && !lastDpadRight && !autoShootActive) {
            currentChamber = nextChamber(currentChamber);
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            startSorterMove(targetPos);
        }
        lastDpadRight = dpadRightPressed;

        updateSorterMovement();

        if (!shootingMode && !autoShootActive) {
            autoIntakeColorCheck();
        } else {
            checkChamberEmpty();
        }

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
                int targetPos = getChamberPosition(currentChamber, shootingMode);
                startSorterMove(targetPos);
            }
        }
        lastAButton = aPressed;

        // === Auto Shoot State Machine ===
        if (autoShootActive) {
            updateAutoShootSequence();
        }

        // === Flywheel RPM ===
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastXButton) {
            distanceBasedRPM = !distanceBasedRPM;
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

        // === Pose Reset System ===
        if (gamepad1.dpad_up) {
            follower.setPose(new Pose(72, 72, Math.PI / 2));
        }

        updateTelemetry(normPos, shooterColorDetected);
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
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 3: // Wait for servo retract
                if (autoShootTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
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
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 6: // Wait for servo retract
                if (autoShootTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
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

    // === Turret Movement Helper ===

    private double moveTurretToOffset(DcMotorEx turretMotor, double turretDesiredRelativeOffset) {
        double turretDesiredDegrees = Math.toDegrees(turretDesiredRelativeOffset);
        double turretRotations = turretMotor.getCurrentPosition() / TICKS_PER_REV;

        double desiredRotations = turretDesiredDegrees / 360.0;

        double error = desiredRotations - turretRotations;

        if (Math.abs(error) > 0.02) { // 0.02 rotations tolerance
            turretMotor.setPower(error / Math.abs(error) * 0.2);
        } else {
            turretMotor.setPower(0);
        }
        return error;
    }

    // === Distance Detection ===

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

    private void updateRPMLED() {
        if (!shootingMode && !autoShootActive) {
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

    private void updateColorLEDs(String color) {
        if (color.equals("GREEN")) {
            led1.setPosition(0.5);
        } else if (color.equals("PURPLE")) {
            led1.setPosition(0.722);
        } else {
            led1.setPosition(0);
        }
    }

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
                currentChamber = nextChamber(currentChamber);
                int target = getChamberPosition(currentChamber, false);
                startSorterMove(target);
            }
            colorActive = false;
            colorStartTime = 0;
        }
    }

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
            emptyDetectionActive = false;
            emptyStartTime = 0;
        }
    }

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

    private int nextChamber(int c) {
        if (c == 1) return 0;
        if (c == 0) return 2;
        return 1;
    }

    private int normalize(int ticks) {
        return ((ticks % FULL_ROT) + FULL_ROT) % FULL_ROT;
    }

    private int calculateShortestError(int current, int target) {
        int error = target - current;
        if (error > FULL_ROT / 2) {
            error -= FULL_ROT;
        } else if (error < -FULL_ROT / 2) {
            error += FULL_ROT;
        }
        return error;
    }

    private void updateTelemetry(int normPos, String shooterColorDetected) {
        double currentRPM = (m3.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);
        boolean rpmReady = (targetRPM > 0) && (rpmError <= RPM_TOLERANCE);

        telemetry.addLine("=== AUTO SHOOT + TURRET TRACKING ===");
        telemetry.addData("Auto Shoot Active", autoShootActive);
        telemetry.addData("Auto Shoot State", autoShootState);
        telemetry.addData("Shots Complete", shotsComplete + "/3");
        telemetry.addLine();

        telemetry.addLine("=== Sorter ===");
        telemetry.addData("Pos", normPos);
        telemetry.addData("Chamber", currentChamber + 1);
        telemetry.addData("Moving", sorterMoving);
        String ch1 = chamberFull[0] ? "O" : "X";
        String ch2 = chamberFull[1] ? "O" : "X";
        String ch3 = chamberFull[2] ? "O" : "X";
        telemetry.addData("Ch1/2/3", ch1 + "/" + ch2 + "/" + ch3);
        telemetry.addData("Mode", shootingMode ? "SHOOT (Y)" : "INTAKE (Y)");
        telemetry.addData("Color", shooterColorDetected);
        telemetry.addLine();

        telemetry.addLine("=== Shooter ===");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", String.format("%.0f", currentRPM));
        telemetry.addData("Ready", rpmReady ? "YES" : "NO");
        telemetry.addData("Distance Mode", distanceBasedRPM ? "AUTO (X)" : "MANUAL (X)");
        telemetry.addLine();

        telemetry.addData("Robot Position",
                String.format("(%.1f, %.1f, %.1f°)",
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        Math.toDegrees(follower.getHeading())));
        telemetry.addLine();

        telemetry.addLine("=== Controls ===");
        telemetry.addLine("A: AUTO SHOOT ALL 3");
        telemetry.addLine("B: Toggle Turret Tracking");
        telemetry.addLine("Y: Manual Mode Toggle");
        telemetry.addLine("DpadRight: Manual Chamber");
        telemetry.addLine("RB: RPM | LB: 1500 | DpadDown: Off");

        telemetry.update();
    }

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