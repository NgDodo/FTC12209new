package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name="Hybrid Turret Tracking", group="DriveTrainControl")
public class HybridTurretTracking extends OpMode {
    private Follower follower;
    public static Pose startingPose;

    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1;
    DcMotorEx m2; // Turret motor
    DcMotorEx m3, m0;
    CRServo s3;
    Servo s2;

    // === Limelight ===
    private Limelight3A limelight;
    private static final String LIMELIGHT_NAME = "Webcam 2";
    private static final int APRILTAG_PIPELINE = 1;

    // === Turret Constants ===
    private static final double TICKS_PER_REV = 1393.1;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    private static final int MAX_POSITION = (int)((330.0 / 2.0) * TICKS_PER_DEGREE);
    private static final int MIN_POSITION = -MAX_POSITION;

    // === Hybrid Tracking State ===
    private double turretEstimatedOffsetRad = 0.0;
    private double limelightWeight = 0.0;
    private long lastLimelightUpdate = 0;
    private static final long LIMELIGHT_TIMEOUT_MS = 500;

    // === Tracking Mode ===
    private enum TrackingMode {
        OFF,
        HYBRID_TRACK
    }
    private TrackingMode currentTrackingMode = TrackingMode.OFF;
    private boolean lastBButton = false;

    // === Smoothing for Limelight ===
    private static final double LL_SMOOTHING_ALPHA = 0.7;
    private double smoothedLimelightTx = 0.0;
    private boolean limelightInitialized = false;

    // === PID for Turret ===
    private double turretKp = 0.014;
    private double turretKi = 0.002;
    private double turretKd = 0.0005;
    private double turretIntegral = 0;
    private double turretLastError = 0;
    private long turretLastTime = 0;
    private static final double TURRET_MAX_POWER = 0.85;
    private static final double TURRET_MIN_POWER = 0.06;

    // === Weight Adjustment Rates ===
    private static final double WEIGHT_INCREASE_RATE = 0.15;  // How fast to trust Limelight
    private static final double WEIGHT_DECREASE_RATE = 0.08;  // How fast to fall back to odometry
    private static final double MAX_LIMELIGHT_WEIGHT = 0.95;  // Max trust in Limelight
    private static final double MIN_LIMELIGHT_WEIGHT = 0.0;   // Min trust (full odometry)

    // === Speed Thresholds ===
    private static final double SLOW_SPEED_THRESHOLD = 0.25;  // Below this, prefer Limelight
    private static final double FAST_SPEED_THRESHOLD = 0.6;   // Above this, prefer odometry

    // === Goal Position ===
    private static final Pose GOAL_POST = new Pose(10, 134, 0);

    @Override
    public void init() {
        startingPose = new Pose(72, 72, Math.toRadians(90));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        // === DriveTrain ===
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fL");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "bR");

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
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === Limelight Setup ===
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        turretLastTime = System.nanoTime();

        telemetry.addLine("Hybrid Turret Tracking Initialized");
        telemetry.addLine("B: Toggle Tracking ON/OFF");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        // === Tracking Mode Toggle (B Button) ===
        boolean bPressed = gamepad1.b;
        if (bPressed && !lastBButton) {
            if (currentTrackingMode == TrackingMode.OFF) {
                currentTrackingMode = TrackingMode.HYBRID_TRACK;
                resetTracking();
            } else {
                currentTrackingMode = TrackingMode.OFF;
                resetTracking();
                m2.setPower(0);
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

        // === Hybrid Turret Tracking ===
        if (currentTrackingMode == TrackingMode.HYBRID_TRACK) {
            updateHybridTurretTracking();
        }

        // === Pose Reset System ===
        if (gamepad1.dpad_down) {
            follower.setPose(new Pose(72, 72, Math.PI / 2));
        }

        // === Telemetry ===
        updateTelemetry();
    }

    /**
     * Hybrid tracking that combines odometry-based tracking with Limelight corrections
     */
    private void updateHybridTurretTracking() {
        // === Step 1: Calculate odometry-based angle to goal ===
        double y_goal_distance = follower.getPose().getY() - GOAL_POST.getY();
        double x_goal_distance = follower.getPose().getX() - GOAL_POST.getX();
        double angle_to_goal = Math.atan2(y_goal_distance, x_goal_distance);
        double odometryBasedOffset = normalizeAngle(-angle_to_goal + follower.getHeading() + Math.PI);

        // === Step 2: Get Limelight correction ===
        double limelightCorrectionRad = 0.0;
        boolean limelightValid = false;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            // Look for goal AprilTags (ID 20 or 24)
            LLResultTypes.FiducialResult validTarget = null;
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                //if (id == 20 || id == 24) {
                if (id == 20) {
                    validTarget = fiducial;
                    break;
                }
            }

            if (validTarget != null) {
                double rawTx = validTarget.getTargetXDegrees();

                // Smooth the Limelight reading
                if (!limelightInitialized) {
                    smoothedLimelightTx = rawTx;
                    limelightInitialized = true;
                } else {
                    smoothedLimelightTx = LL_SMOOTHING_ALPHA * rawTx +
                            (1.0 - LL_SMOOTHING_ALPHA) * smoothedLimelightTx;
                }

                // Convert to radians (negative because Limelight tx is opposite direction)
                limelightCorrectionRad = Math.toRadians(-smoothedLimelightTx);
                limelightValid = true;
                lastLimelightUpdate = System.currentTimeMillis();
            }
        }

        // === Step 3: Calculate dynamic weight based on conditions ===
        double robotSpeed = Math.sqrt(
                Math.pow(gamepad1.left_stick_x, 2) +
                        Math.pow(gamepad1.left_stick_y, 2)
        );

        long timeSinceLimelightMs = System.currentTimeMillis() - lastLimelightUpdate;
        boolean limelightFresh = timeSinceLimelightMs < LIMELIGHT_TIMEOUT_MS;

        // Adjust weight based on conditions
        if (limelightValid && limelightFresh && robotSpeed < SLOW_SPEED_THRESHOLD) {
            // Conditions are good for Limelight - increase trust
            limelightWeight = Math.min(MAX_LIMELIGHT_WEIGHT,
                    limelightWeight + WEIGHT_INCREASE_RATE);
        } else if (robotSpeed > FAST_SPEED_THRESHOLD || !limelightFresh) {
            // Robot moving fast or Limelight stale - decrease trust rapidly
            limelightWeight = Math.max(MIN_LIMELIGHT_WEIGHT,
                    limelightWeight - WEIGHT_DECREASE_RATE * 2);
        } else {
            // Moderate conditions - slowly decrease trust
            limelightWeight = Math.max(MIN_LIMELIGHT_WEIGHT,
                    limelightWeight - WEIGHT_DECREASE_RATE);
        }

        // === Step 4: Blend the estimates ===
        if (limelightValid) {
            // When Limelight sees the target, apply its correction to our estimate
            // The correction is weighted - at weight=1.0, we fully trust Limelight's offset
            // At weight=0.0, we ignore it and use pure odometry
            turretEstimatedOffsetRad = (1.0 - limelightWeight) * odometryBasedOffset +
                    limelightWeight * (turretEstimatedOffsetRad + limelightCorrectionRad);
        } else {
            // No Limelight data - drift toward odometry estimate
            double blendFactor = 0.05; // How quickly to drift toward odometry when no LL
            turretEstimatedOffsetRad = (1.0 - blendFactor) * turretEstimatedOffsetRad +
                    blendFactor * odometryBasedOffset;
        }

        // Normalize the blended estimate
        turretEstimatedOffsetRad = normalizeAngle(turretEstimatedOffsetRad);

        // === Step 5: Move turret using PID to the blended estimate ===
        moveTurretToOffset(m2, turretEstimatedOffsetRad);
    }

    /**
     * PID control to move turret to desired offset angle
     */
    private void moveTurretToOffset(DcMotorEx turretMotor, double desiredOffsetRad) {
        // Convert current turret position to radians
        double turretCurrentRad = (turretMotor.getCurrentPosition() / TICKS_PER_REV) * 2.0 * Math.PI;

        // Calculate error
        double error = normalizeAngle(desiredOffsetRad - turretCurrentRad);

        // PID calculation
        long currentTime = System.nanoTime();
        double dt = (currentTime - turretLastTime) / 1e9;
        if (dt <= 0) dt = 1e-6;

        turretIntegral += error * dt;
        turretIntegral = Math.max(-1000.0, Math.min(1000.0, turretIntegral));

        double derivative = (error - turretLastError) / dt;
        double pidOutput = (turretKp * error) + (turretKi * turretIntegral) + (turretKd * derivative);

        // Clamp output
        pidOutput = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, pidOutput));

        // Apply minimum power threshold
        if (Math.abs(pidOutput) > 0 && Math.abs(pidOutput) < TURRET_MIN_POWER) {
            pidOutput = Math.copySign(TURRET_MIN_POWER, pidOutput);
        }

        // Stop if close enough
        if (Math.abs(error) < Math.toRadians(1.0)) { // 1 degree tolerance
            pidOutput = 0;
            turretIntegral = 0;
        }

        // Apply limits
        int currentPosition = turretMotor.getCurrentPosition();
        if (currentPosition >= MAX_POSITION && pidOutput > 0) {
            pidOutput = 0;
        } else if (currentPosition <= MIN_POSITION && pidOutput < 0) {
            pidOutput = 0;
        }

        turretMotor.setPower(pidOutput);
        turretLastError = error;
        turretLastTime = currentTime;
    }

    private void resetTracking() {
        limelightWeight = 0.0;
        turretIntegral = 0.0;
        turretLastError = 0.0;
        limelightInitialized = false;
        smoothedLimelightTx = 0.0;
        turretLastTime = System.nanoTime();
        lastLimelightUpdate = 0;
    }

    /**
     * Normalizes an angle to the range [-PI, PI]
     */
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

    private void updateTelemetry() {
        telemetry.addData("Tracking Mode", currentTrackingMode);
        telemetry.addData("Position (x, y, heading)",
                String.format("%.1f, %.1f, %.1f°",
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        Math.toDegrees(follower.getHeading())));

        if (currentTrackingMode == TrackingMode.HYBRID_TRACK) {
            // Calculate robot speed
            double robotSpeed = Math.sqrt(
                    Math.pow(gamepad1.left_stick_x, 2) +
                            Math.pow(gamepad1.left_stick_y, 2)
            );

            // Calculate odometry-based angle for comparison
            double y_goal_distance = follower.getPose().getY() - GOAL_POST.getY();
            double x_goal_distance = follower.getPose().getX() - GOAL_POST.getX();
            double angle_to_goal = Math.atan2(y_goal_distance, x_goal_distance);
            double odometryBasedOffset = normalizeAngle(-angle_to_goal + follower.getHeading() + Math.PI);

            telemetry.addData("Robot Speed", "%.2f", robotSpeed);
            telemetry.addData("Limelight Weight", "%.1f%%", limelightWeight * 100);
            telemetry.addData("Odometry Weight", "%.1f%%", (1.0 - limelightWeight) * 100);
            telemetry.addData("Odometry Angle", "%.1f°", Math.toDegrees(odometryBasedOffset));
            telemetry.addData("Blended Estimate", "%.1f°", Math.toDegrees(turretEstimatedOffsetRad));
            telemetry.addData("Limelight TX", "%.1f°", smoothedLimelightTx);
            telemetry.addData("Time Since LL", "%d ms", System.currentTimeMillis() - lastLimelightUpdate);
        }

        telemetry.addData("Turret Position", "%.2f rot", m2.getCurrentPosition() / TICKS_PER_REV);
        telemetry.update();
    }
}