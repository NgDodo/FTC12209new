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
public class TurretV2 {
    // === Flywheel PID ===
    public static double flywheelKp = 0.0022;
    public static double flywheelKi = 0.00001;
    public static double flywheelKd = 0.0;
    public static double flywheelKF = 0.00025;
    private double flywheelIntegral = 0;
    private double flywheelLastError = 0;
    private long flywheelLastTime = 0;

    // === Turret Odometry PID ===
    /// if it's oversshooting the turret rotation while detecting limelight, increase llKd or reduce llKp
    /// if it's overshooting while not detecting limelight, decrease kP
    // Used when limelight cannot see the target — rougher, longer-range correction
    public static double odomKp = 4.75;
    public static double odomKi = 0.0;
    public static double odomKd = 0.18;
    public static double odomKf = 0.005;
    public static double odomDeadband = 0.005;       // rotations — skip PID inside this range
    public static double odomIntegralClamp = 5.0;   // max integral accumulation
    private double odomIntegral = 0;
    private double odomLastError = 0;
    private long odomLastTime = 0;

    // === Rotational Velocity Feedforward ===
    // Drives the turret to counter-rotate at the same rate the robot is spinning,
    // so fast robot rotations don't cause the turret to lag behind the goal.
    // Uses follower.getVelocity().getAngularVelocity() directly — no differentiation needed.
    //
    // How to tune odomKff_rotation:
    //   1. Cover the limelight so only odom-PID is active.
    //   2. Set odomKff_rotation = 0. Spin the robot fast. Observe how far the
    //      turret lags after the robot stops.
    //   3. Increase odomKff_rotation in small steps (e.g. 0.005 at a time) and
    //      repeat until the turret stays roughly on target during the spin.
    //   4. If the turret now overshoots after the robot stops, back off slightly
    //      or increase odomKd to dampen it.
    //
    // Units: motor power per (rad/s of robot angular velocity)
    // Negative because when the robot rotates CCW (+), the turret must rotate
    // CW (-) to hold its absolute heading — i.e. counter-rotation.
    public static double odomKff_rotation = 0.0;    // start here, tune upward

    // === Turret Limelight PID ===
    // Used when limelight sees the target — finer, short-range correction
    public static double llKp = 4;
    public static double llKi = 0.0;
    public static double llKd = 0.25;
    public static double llKf = 0.0;
    public static double llDeadband = 1.0;          // degrees — skip PID inside this range
    public static double llIntegralClamp = 5.0;     // max integral accumulation
    private double llIntegral = 0;
    private double llLastError = 0;
    private long llLastTime = 0;

    private DcMotorEx turretRotationMotor;
    private DcMotorEx flywheelMotor;

    boolean leftBumperPressed, rightBumperPressed = false;

    // === Vision & IMU ===
    private Limelight3A limelight;
    private IMU imu;
    private static final String LIMELIGHT_NAME = "Webcam 2";
    private static final int APRILTAG_PIPELINE = 1;

    // === Turret Configuration ===
    public static final double TURRET_TICKS_PER_REV = 2596.363; // equals the (gear ratio x motor resolution x gearbox on motor)
    // here, this equals (204/44) x (28 ticks/rev) x (20)

    public boolean limelightTracking = false;

    public static double turretMotorPowerMultiplier = 0.25;
    public static double turretMotorLIMELIGHTPowerMultiplier = 0.077;

    // === Tracking Mode ===
    public enum TurretTrackingMode {
        OFF,
        GOAL_TRACKING,
        OBELISK_TRACKING
    }

    // === Shooting parameters ===
    private static final double IDLE_RPM = 2000;

    // === Shooter presets ===
    private final int[] rpmPresets = {3100, 3800};
    private int presetIndex = -1;
    public double targetRPM = 0;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadDown = false;

    public static final double TICKS_PER_REV_FLYWHEEL = 28.0;
    private static final double RPM_TOLERANCE = 100.0;

    public TurretTrackingMode currentTrackingMode = TurretTrackingMode.GOAL_TRACKING;
    private boolean lastBButton = false;

    public static double ARTIFACT_SHOOT_EXIT_VELOCITY = 80.0; /// in/sec
    public Pose GoalLocation, ObeliskLocation;

    public enum TurretMOTIF {
        GPP,
        PGP,
        PPG,
        UNKNOWN
    }

    public TurretMOTIF currentMotif;
    public String allianceColor;
    public double manualTeleopOffset = 0.0;

    public TurretV2(HardwareMap hardwareMap, String _allianceColor) {
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
        ObeliskLocation = FIELD_CONSTANTS.OBELISK_LOCATION;
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
        odomLastTime = System.nanoTime();
        llLastTime = System.nanoTime();

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "m3");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRotationMotor = hardwareMap.get(DcMotorEx.class, "m2");

        turretRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        currentTrackingMode = TurretTrackingMode.GOAL_TRACKING;
        currentMotif = TurretMOTIF.UNKNOWN;
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
        if (currentTrackingMode.equals(TurretTrackingMode.OBELISK_TRACKING) && currentMotif.equals(TurretMOTIF.UNKNOWN)) {
            double y_goal_distance = follower.getPose().getY() - ObeliskLocation.getY();
            double x_goal_distance = follower.getPose().getX() - ObeliskLocation.getX();
            double angle_to_goal = Math.atan2(y_goal_distance, x_goal_distance);
            double turretDesiredRelativeOffset = normalizeAngle(-angle_to_goal + follower.getHeading() + Math.PI);
            runOdomPID(turretDesiredRelativeOffset, follower);

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    switch (fiducial.getFiducialId()) {
                        case 21:
                            currentMotif = TurretMOTIF.GPP;
                            currentTrackingMode = TurretTrackingMode.GOAL_TRACKING;
                            break;
                        case 22:
                            currentMotif = TurretMOTIF.PGP;
                            currentTrackingMode = TurretTrackingMode.GOAL_TRACKING;
                            break;
                        case 23:
                            currentMotif = TurretMOTIF.PPG;
                            currentTrackingMode = TurretTrackingMode.GOAL_TRACKING;
                            break;
                    }
                }
            }
        }

        this.limelightTracking = false;

        if (currentTrackingMode.equals(TurretTrackingMode.GOAL_TRACKING)) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if ((fiducial.getFiducialId() == 24 && this.allianceColor.equals("RED"))
                            || (fiducial.getFiducialId() == 20 && this.allianceColor.equals("BLUE"))) {
                        this.limelightTracking = true;

                        double bearingDegrees = fiducial.getTargetXDegrees();
                        if (Math.abs(bearingDegrees) > llDeadband) {
                            // Error in rotations — limelight bearing is a relative offset so no
                            // position subtraction needed, just convert degrees to rotations
                            double error = bearingDegrees / 360.0;
                            double velocityCompensatedAngleAdj = calculateDesiredGoalAngleLIMELIGHTOffset(follower);
                            turretRotationMotor.setPower(runLimelightPID(error + velocityCompensatedAngleAdj));
                        } else {
                            // Inside deadband — hold position and reset limelight PID state
                            resetLimelightPID();
                            turretRotationMotor.setPower(0);
                        }
                        break;
                    }
                }
            }

            // Limelight lost the target — fall back to odometry, reset LL PID so it
            // doesn't have stale integral/derivative when it reacquires
            if (!this.limelightTracking) {
                resetLimelightPID();

                double y_goal_distance = follower.getPose().getY() - GoalLocation.getY();
                double x_goal_distance = follower.getPose().getX() - GoalLocation.getX();
                Pose velocityWiseGoalOffset = calculateDesiredGoalPositionODOMETRYOffset(follower);

                double angle_to_goal = Math.atan2(y_goal_distance + velocityWiseGoalOffset.getY(), x_goal_distance + velocityWiseGoalOffset.getX());
                double turretDesiredRelativeOffset = normalizeAngle(-angle_to_goal + follower.getHeading() + Math.PI);
                runOdomPID(turretDesiredRelativeOffset, follower);
            }
        }
    }

    private Pose calculateDesiredGoalPositionODOMETRYOffset(Follower follower){
        double dx = GoalLocation.getX() - follower.getPose().getX();
        double dy = GoalLocation.getY() - follower.getPose().getY();
        double distanceToTarget = Math.sqrt(dx * dx + dy * dy); /// in inches

        double shooting_time = distanceToTarget / ARTIFACT_SHOOT_EXIT_VELOCITY;

        double x_velocity = follower.getVelocity().getXComponent();
        double y_velocity = follower.getVelocity().getYComponent();


        return new Pose(x_velocity * shooting_time, y_velocity * shooting_time); // negative to be in reverse direction
    }

    private double calculateDesiredGoalAngleLIMELIGHTOffset(Follower follower){
        double dx = GoalLocation.getX() - follower.getPose().getX();
        double dy = GoalLocation.getY() - follower.getPose().getY();
        double distanceToTarget = Math.sqrt(dx * dx + dy * dy); /// in inches

        double shooting_time = distanceToTarget / ARTIFACT_SHOOT_EXIT_VELOCITY;

        double x_velocity = follower.getVelocity().getXComponent();
        double y_velocity = follower.getVelocity().getYComponent();


        Pose offsetFromActualGoal = new Pose(x_velocity * shooting_time, y_velocity * shooting_time); // negative to be in reverse direction
        double angleOffset = Math.atan2(offsetFromActualGoal.getY() - follower.getPose().getY(), offsetFromActualGoal.getX() - follower.getPose().getX());
        return angleOffset / 360.0; /// returns in rotations
    }

    /**
     * Odometry-based PID — converts a desired relative angle offset into motor power.
     * Error is in rotations.
     *
     * The rotational-velocity feedforward term (-odomKff_rotation * angularVelocity)
     * is added directly to the PID output. When the robot spins CCW (positive rad/s),
     * the turret must spin CW (negative power) to hold its absolute heading, hence the
     * negative sign. This keeps the turret on-target during fast robot rotations before
     * positional error has a chance to build up.
     */
    private double runOdomPID(double turretDesiredRelativeOffset, Follower follower) {
        double turretDesiredDegrees = Math.toDegrees(turretDesiredRelativeOffset);
        double currentRotations = turretRotationMotor.getCurrentPosition() / TURRET_TICKS_PER_REV - manualTeleopOffset;
        double desiredRotations = turretDesiredDegrees / 360.0;
        double error = desiredRotations - currentRotations;

        // velocity in rad/s directly from Pedro's localizer — no differentiation needed.
        double angularVelocity = follower.getAngularVelocity();
        double rotationFeedforward = -odomKff_rotation * angularVelocity;

        if (Math.abs(error) < odomDeadband) {
            // Still apply feedforward inside the deadband so the turret actively
            // counter-rotates with the robot even when position error is tiny.
            turretRotationMotor.setPower(Math.max(-1.0, Math.min(1.0, rotationFeedforward)));
            resetOdomPID();
            return error;
        }

        long currentTime = System.nanoTime();
        double dt = (currentTime - odomLastTime) / 1e9;
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        odomIntegral += error * dt;
        odomIntegral = Math.max(-odomIntegralClamp, Math.min(odomIntegralClamp, odomIntegral));

        double derivative = (error - odomLastError) / dt;
        double feedforward = odomKf * Math.signum(error);

        double output = (odomKp * error)
                + (odomKi * odomIntegral)
                + (odomKd * derivative)
                + feedforward
                + rotationFeedforward;  // <-- counter-rotation feedforward

        output = Math.max(-1.0, Math.min(1.0, output));
        turretRotationMotor.setPower(output);

        odomLastError = error;
        odomLastTime = currentTime;

        return error;
    }

    /**
     * Limelight-based PID — error is the bearing in rotations (degrees/360).
     * Since bearing is already a relative offset from center, no position math needed.
     */
    private double runLimelightPID(double error) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - llLastTime) / 1e9;
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        llIntegral += error * dt;
        llIntegral = Math.max(-llIntegralClamp, Math.min(llIntegralClamp, llIntegral));

        double derivative = (error - llLastError) / dt;
        double feedforward = llKf * Math.signum(error);

        double output = (llKp * error)
                + (llKi * llIntegral)
                + (llKd * derivative)
                + feedforward;

        output = Math.max(-1.0, Math.min(1.0, output));

        llLastError = error;
        llLastTime = currentTime;

        return output;
    }

    private void resetOdomPID() {
        odomIntegral = 0;
        odomLastError = 0;
        odomLastTime = System.nanoTime();
    }

    private void resetLimelightPID() {
        llIntegral = 0;
        llLastError = 0;
        llLastTime = System.nanoTime();
    }

    private double normalizeAngle(double angle) {
        while (angle > 0.7*Math.PI) angle -= 2 * Math.PI;
        while (angle < -1.3*Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public void updateManualOffset (double amount_to_change) {
        manualTeleopOffset += amount_to_change;
    }
    public void postTelemetry(Telemetry telemetry) {
        double currentRPM = (flywheelMotor.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);
        boolean rpmReady = (targetRPM > 0) && (rpmError <= RPM_TOLERANCE);

        telemetry.addLine("=== Shooter ===");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", String.format("%.0f", currentRPM));
        telemetry.addData("Ready", rpmReady ? "YES" : "NO");
        telemetry.addLine("=== Turret ===");
        telemetry.addData("Tracking Mode", currentTrackingMode);
        telemetry.addData("Limelight Active", limelightTracking);
        telemetry.addData("Turret Position (ticks)", turretRotationMotor.getCurrentPosition());
        telemetry.addData("Turret Position (rot)", turretRotationMotor.getCurrentPosition() / TURRET_TICKS_PER_REV);
        telemetry.addLine();
    }

    public void postTelemetryOnlyGoalPose(Telemetry telemetry, Follower follower) {
        double y_goal_distance = follower.getPose().getY() - GoalLocation.getY();
        double x_goal_distance = follower.getPose().getX() - GoalLocation.getX();
        Pose velocityWiseGoalOffset = calculateDesiredGoalPositionODOMETRYOffset(follower);

        telemetry.addData("Desired Goal X: ", x_goal_distance + velocityWiseGoalOffset.getX());
        telemetry.addData("Desired Goal Y: ", y_goal_distance + velocityWiseGoalOffset.getY());
        telemetry.addLine();
    }

    public void setFlywheelRPM(String shootingLocation) {
        if (shootingLocation.equals("FAR")) {
            targetRPM = rpmPresets[1];
        } else {
            targetRPM = rpmPresets[0];
        }
    }

    public void setFlywheelRPM(int desiredFlywheelRPM) {
        targetRPM = desiredFlywheelRPM;
    }

    public boolean flywheelReachedDesiredRPM() {
        return Math.abs(targetRPM - ((flywheelMotor.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0)) < 200;
    }

    public void stopFlywheel() {
        targetRPM = 0;
    }

    public DcMotorEx getFlywheelMotor() {
        return flywheelMotor;
    }

    public double getFlywheelRPM() {
        return (flywheelMotor.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
    }
}