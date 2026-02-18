package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Config
@Configurable
public class Turret {
    // === Flywheel PID ===
    public static double flywheelKp = 0.0022;
    public static double flywheelKi = 0.00001;
    public static double flywheelKd = 0.0;
    public static double flywheelKF = 0.00025;
    private double flywheelIntegral = 0;
    private double flywheelLastError = 0;
    private long flywheelLastTime = 0;

    // === Flywheel PID ===
    public static double turretRotKp = 0.0;
    public static double turretRotKi = 0.0;
    public static double turretRotKd = 0.0;
    public static double turretRotKf = 0.0;
    private double turretRotIntegral = 0;
    private double turretRotLastErrr = 0;
    private long turretRotLastTime = 0;


    private DcMotorEx turretRotationMotor;
    private DcMotorEx flywheelMotor;

    boolean leftBumperPressed, rightBumperPressed = false;

    // === Vision & IMU ===
    private Limelight3A limelight;
    private IMU imu;
    private static final String LIMELIGHT_NAME = "Webcam 2";
    private static final int APRILTAG_PIPELINE = 1;

    // === Turret Configuration ===
    public static final double TURRET_TICKS_PER_REV = 1393.1;

    public boolean limelightTracking = false;

    public static double turretMotorPowerMultiplier = 0.25;
    public static double turretMotorLIMELIGHTPowerMultiplier = 0.067;

    // === Tracking Mode ===
    private enum TrackingMode {
        OFF,
        LIMELIGHT_AND_ODOMETRY
    }

    // === Shooting parameters ===
    private static final double IDLE_RPM = 2000;

    // === Shooter presets ===
    private final int[] rpmPresets = {3900, 4950};
    private int presetIndex = -1;
    public double targetRPM = 0;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadDown = false;

    public static final double TICKS_PER_REV_FLYWHEEL = 28.0;
    private static final double RPM_TOLERANCE = 100.0;

    private TrackingMode currentTrackingMode = TrackingMode.LIMELIGHT_AND_ODOMETRY;
    private boolean lastBButton = false;

    private Pose GoalLocation;

    public String allianceColor;

    public Turret (HardwareMap hardwareMap, String _allianceColor) {;
        switch (_allianceColor) {
            case "RED":
                GoalLocation = FIELD_CONSTANTS.RED_GOAL_POST;
                break;
            case "BLUE":
                GoalLocation = FIELD_CONSTANTS.BLUE_GOAL_POST;
                break;
            default:
                GoalLocation = FIELD_CONSTANTS.BLUE_GOAL_POST;
                break;
        }
        this.allianceColor = _allianceColor;

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        flywheelLastTime = System.nanoTime();

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "m3");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRotationMotor = hardwareMap.get(DcMotorEx.class, "m2");

        // === Turret Setup ===
        turretRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void updateTurret(Follower follower) {
        updateTurretRotation(follower);
        updateFlywheelSpeed(follower);
    }
    public void updateTurret(Follower follower, Gamepad gamepad1) {
        leftBumperPressed = gamepad1.left_bumper;
        rightBumperPressed = gamepad1.right_bumper;

        if (leftBumperPressed && !lastLeftBumper) {
            targetRPM = IDLE_RPM;
        }

        if (rightBumperPressed && !lastRightBumper) {
            presetIndex = (presetIndex + 1) % rpmPresets.length;
            targetRPM = rpmPresets[presetIndex];
        }

        updateTurretRotation(follower);
        updateFlywheelSpeed(follower);

        lastLeftBumper = leftBumperPressed;
        lastRightBumper = rightBumperPressed;
    }

    private void updateFlywheelSpeed(Follower follower) {
        /// TODO: make flywheel speed update based on 1) follower distance, and 2) limelight distance

        double currentVelocity = flywheelMotor.getVelocity();
        double currentRPM = (currentVelocity / TICKS_PER_REV_FLYWHEEL) * 60.0;

        long currentTime = System.nanoTime();
        double dt = (currentTime - flywheelLastTime) / 1e9;

        double error = targetRPM - currentRPM;

        flywheelIntegral += error * dt;
        flywheelIntegral = Math.max(-10000, Math.min(10000, flywheelIntegral));

        double derivative = (error - flywheelLastError) / dt;
        double feedforward = flywheelKF * targetRPM;
        double pidOutput = (flywheelKp * error) + (flywheelKi * flywheelIntegral) + (flywheelKd * derivative) + feedforward;

        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        flywheelMotor.setPower(pidOutput);

        flywheelLastError = error;
        flywheelLastTime = currentTime;
    }
    private void updateTurretRotation(Follower follower) {
        /// TODO: make turret PID controlled

        this.limelightTracking = false;

        // Try Limelight tracking first (if AprilTag visible)
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fiducial : fiducials) {

                if (fiducial.getFiducialId() == 24 && this.allianceColor.equals("RED")
                        || (fiducial.getFiducialId() == 20 && this.allianceColor.equals("BLUE"))) {
                    this.limelightTracking = true;
                    double bearing = fiducial.getTargetXDegrees();
                    double turretRotatePower = turretMotorLIMELIGHTPowerMultiplier * bearing / 20.0;

                    if (Math.abs(bearing) > 2) {
                        turretRotationMotor.setPower(turretRotatePower);
                    } else {
                        turretRotationMotor.setPower(0);
                    }
                    break; // Found tag 20, stop searching
                }
            }
        }

        // If Limelight not tracking, use odometry-based tracking
        if (!this.limelightTracking) {
            // 1. Calculate component distances from goal
            double y_goal_distance = follower.getPose().getY() - GoalLocation.getY();
            double x_goal_distance = follower.getPose().getX() - GoalLocation.getX();

            // 2. Calculate absolute angle to goal in field coordinates
            double angle_to_goal = Math.atan2(y_goal_distance, x_goal_distance);

            // 3. Calculate turret offset relative to robot heading
            double turretDesiredRelativeOffset = normalizeAngle(-angle_to_goal + follower.getHeading() + Math.PI);

            // 4. Move turret to track the goal
            moveTurretToOffset(turretDesiredRelativeOffset);
        }
    }
    /**
     * Moves turret to desired offset angle (odometry-based tracking)
     */
    private double moveTurretToOffset(double turretDesiredRelativeOffset) {
        double turretDesiredDegrees = Math.toDegrees(turretDesiredRelativeOffset);
        double turretRotations = turretRotationMotor.getCurrentPosition() / TURRET_TICKS_PER_REV;
        double desiredRotations = turretDesiredDegrees / 360.0;
        double error = desiredRotations - turretRotations;

        if (Math.abs(error) > 0.01) { // 0.015 rotations tolerance
            turretRotationMotor.setPower(error / Math.abs(error) * turretMotorPowerMultiplier);
        } else {
            turretRotationMotor.setPower(0);
        }
        return error;
    }

    /**
     * Normalizes an angle to the range [-PI, PI]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public void postTelemetry(Telemetry telemetry) {
        // Calculate flywheel status
        double currentRPM = (flywheelMotor.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);
        boolean rpmReady = (targetRPM > 0) && (rpmError <= RPM_TOLERANCE);

        telemetry.addLine("=== Shooter ===");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", String.format("%.0f", currentRPM));
        telemetry.addData("Ready", rpmReady ? "YES" : "NO");    // Is flywheel at speed?
        telemetry.addLine();
    }

    public void setFlywheelRPM(String shootingLocation) {
        if (shootingLocation.equals("FAR")) {
            targetRPM = rpmPresets[1];
        }
        else {
            targetRPM = rpmPresets[0];
        }
    }
    public void stopFlywheel() {
        targetRPM = 0;
    }

    public DcMotorEx getFlywheelMotor (){
        return flywheelMotor;
    }
}
