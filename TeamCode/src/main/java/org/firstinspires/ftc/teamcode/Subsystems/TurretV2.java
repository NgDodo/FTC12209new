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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Config
@Configurable
public class TurretV2 {
    // === Flywheel PID ===
    public static double flywheelKp = 0.005;
    public static double flywheelKi = 0.00001;
    public static double flywheelKd = 0.0;
    public static double flywheelKF = 0.00035;
    private double flywheelIntegral = 0;
    private double flywheelLastError = 0;
    private long flywheelLastTime = 0;

    // === Turret Odometry PID ===
    public static double odomKp = 4.75;
    public static double odomKi = 0.0;
    public static double odomKd = 0.18;
    public static double odomKf = 0.005;
    public static double odomDeadband = 0.005;
    public static double odomIntegralClamp = 5.0;
    private double odomIntegral = 0;
    private double odomLastError = 0;
    private long odomLastTime = 0;

    // === Rotational Velocity Feedforward ===
    public static double odomKff_rotation = 0.0;

    // === Turret Limelight PID ===
    public static double llKp = 7.0;
    public static double llKi = 0.0;
    public static double llKd = 0.0;
    public static double llKf = 0.01;
    public static double llDeadband = 1.0;
    public static double llIntegralClamp = 5.0;
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
    public static final double TURRET_TICKS_PER_REV = 2596.363;

    public boolean limelightTracking = false;

    public static double turretMotorPowerMultiplier = 0.25;
    public static double turretMotorLIMELIGHTPowerMultiplier = 0.077;

    // === Tracking Mode ===
    public enum TurretTrackingMode {
        OFF,
        GOAL_TRACKING,
        OBELISK_TRACKING
    }

    // ========================================================================
    // RPM DISTANCE BRACKETS
    // ========================================================================
    // Three distance zones — tune the RPMs and thresholds in FTC Dashboard.
    // Distance is measured in inches from the robot to the goal.
    //
    //   CLOSE:  distance < DIST_THRESHOLD_CLOSE
    //   MEDIUM: DIST_THRESHOLD_CLOSE <= distance < DIST_THRESHOLD_MEDIUM
    //   FAR:    distance >= DIST_THRESHOLD_MEDIUM
    //
    // To tune:
    //   1. Drive to each zone and note the distance logged in telemetry.
    //   2. Adjust RPM until balls score consistently from that zone.
    //   3. Set the thresholds to match the physical zones on your field.
    public static double DIST_THRESHOLD_CLOSE  = 70.0;   // inches — closer than this = CLOSE
    public static double DIST_THRESHOLD_MEDIUM = 120.0;   // inches — closer than this = MEDIUM, else FAR

    public static double RPM_CLOSE  = 2950;
    public static double RPM_MEDIUM = 3200;
    public static double RPM_FAR    = 3600;

    public static double RPM_FF = 125;
    // ========================================================================
    // MEGATAG LIMELIGHT TARGETING
    // ========================================================================
    // When MegaTag field-space localization is active, the limelight returns a
    // full 3D robot pose in field coordinates. We use that pose to compute an
    // exact angle to the goal — much more accurate than bearing-only tracking,
    // especially at steep angles or long range.
    //
    // Set USE_MEGATAG_POSE = true to enable. Falls back to bearing-only if
    // MegaTag pose is unavailable in a given frame.
    public static boolean USE_MEGATAG_POSE = false;

    // === Shooting parameters ===
    private static final double IDLE_RPM = 2000;

    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadDown = false;

    public static final double TICKS_PER_REV_FLYWHEEL = 28.0;
    private static final double RPM_TOLERANCE = 100.0;

    public double goalOffsetForLimelight = -2;

    public TurretTrackingMode currentTrackingMode = TurretTrackingMode.GOAL_TRACKING;
    private boolean lastBButton = false;

    public static double ARTIFACT_SHOOT_EXIT_VELOCITY = 80.0;
    public static double FLYWHEEL_DISTANCE_VELOCITY_RATIO = 70.0;
    public Pose GoalLocation, ObeliskLocation;

    public enum TurretMOTIF {
        GPP,
        PGP,
        PPG,
        UNKNOWN
    }

    public enum FlywheelStateFSM {
        OFF,
        AutomaticDistancing,
        PresetLow1000,
        Preset3100,
        Preset3600
    }

    public FlywheelStateFSM flywheelState = FlywheelStateFSM.OFF;

    public TurretMOTIF currentMotif;
    public String allianceColor;
    public double manualTeleopOffset = 0.0;

    // Cached last-known distance for telemetry
    private double lastDistanceToGoal = 0.0;
    private String lastRPMZone = "UNKNOWN";

    public TurretV2(HardwareMap hardwareMap, String _allianceColor) {
        switch (_allianceColor) {
            case "RED":
                GoalLocation = FIELD_CONSTANTS.RED_GOAL_POST;
                goalOffsetForLimelight = -2;   // ← added
                break;
            case "BLUE":
                GoalLocation = FIELD_CONSTANTS.BLUE_GOAL_POST;
                goalOffsetForLimelight = 1.5;   // ← added
                break;
            default:
                GoalLocation = FIELD_CONSTANTS.BLUE_GOAL_POST;
                goalOffsetForLimelight = 1.5;   // ← added
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

        flywheelState = FlywheelStateFSM.OFF;
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
        updateMegaTagRelocalization(follower); // add this line
        updateTurretRotation(follower);
        updateFlywheelSpeedBasedOnDistance(follower);

        flywheelState = FlywheelStateFSM.AutomaticDistancing;
    }

    public void updateTurret(Follower follower, Gamepad gamepad1) {
        leftBumperPressed = gamepad1.left_bumper;
        rightBumperPressed = gamepad1.right_bumper;

        if (leftBumperPressed && !lastLeftBumper) {
            flywheelState = FlywheelStateFSM.PresetLow1000;
        }
        if (rightBumperPressed && !lastRightBumper) {
            flywheelState = FlywheelStateFSM.AutomaticDistancing;
        }

        updateMegaTagRelocalization(follower); // add this line
        updateTurretRotation(follower);
        updateFlywheelSpeedBasedOnDistance(follower);

        lastLeftBumper = leftBumperPressed;
        lastRightBumper = rightBumperPressed;
    }

    // ========================================================================
    // RPM CALCULATION — distance bracket presets
    // ========================================================================

    /**
     * Returns the correct RPM for the current distance to goal using three zones.
     * All thresholds and RPMs are tunable in FTC Dashboard.
     */
    private double getRPMForDistance(double distanceInches) {
        if (distanceInches < DIST_THRESHOLD_CLOSE) {
            lastRPMZone = "CLOSE";
            return RPM_CLOSE;
        } else if (distanceInches < DIST_THRESHOLD_MEDIUM) {
            lastRPMZone = "MEDIUM";
            return RPM_MEDIUM;
        } else {
            lastRPMZone = "FAR";
            return RPM_FAR;
        }
    }

    private void _calculateDesiredFlywheelRPM(Follower follower) {
        switch (flywheelState) {
            case OFF:
                targetRPM = 0;
                lastRPMZone = "OFF";
                break;

            case AutomaticDistancing:
                // Use MegaTag pose if available and enabled, otherwise fall back to odometry pose
                double[] goalPos = getMegaTagGoalDistance(follower);
                lastDistanceToGoal = goalPos[0];
                targetRPM = getRPMForDistance(lastDistanceToGoal) + RPM_FF;
                break;

            case PresetLow1000:
                targetRPM = 1000;
                lastRPMZone = "MANUAL";
                break;

            case Preset3100:
                targetRPM = 3100;
                lastRPMZone = "MANUAL";
                break;

            case Preset3600:
                targetRPM = 3600;
                lastRPMZone = "MANUAL";
                break;
        }
    }

    /**
     * Returns [distance, robotX, robotY] using MegaTag field-space pose if
     * USE_MEGATAG_POSE is true and a valid MegaTag result exists, otherwise
     * falls back to Pedro odometry pose.
     *
     * MegaTag gives us the robot's absolute field position as seen by the
     * limelight's 3D solver — more accurate than odometry drift over a match.
     */
    private double[] getMegaTagGoalDistance(Follower follower) {
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        if (USE_MEGATAG_POSE) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose(); // MT2 is more accurate
                if (botpose != null) {
                    robotX = botpose.getPosition().x * 39.3701;
                    robotY = botpose.getPosition().y * 39.3701;
                }
            }
        }

        double dx = robotX - GoalLocation.getX();
        double dy = robotY - GoalLocation.getY();
        return new double[]{Math.sqrt(dx * dx + dy * dy), robotX, robotY};
    }

    private void updateFlywheelSpeedBasedOnDistance(Follower follower) {
        _calculateDesiredFlywheelRPM(follower);

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
    // === MegaTag Relocalization ===
// Instead of tracking directly from MegaTag pose, use it to correct
// Pedro's odometry when a tag is visible. The odom PID then runs as normal
// off the corrected pose — no twitching from frame-to-frame vision noise.
    public static boolean USE_MEGATAG_RELOCALIZATION = false;
    public static double MEGATAG_RELOCALIZE_TRUST = 0.15; // 0.0 = full odom, 1.0 = full vision
// Lower trust = smoother but slower correction. Start at 0.15, raise if odom drifts badly.
    /**
     * If a MegaTag2 pose is available, nudges Pedro's current pose toward the
     * vision-derived pose by MEGATAG_RELOCALIZE_TRUST (a blend factor).
     * Called once per loop — smooth, noise-resistant, never jerky.
     */
    private void updateMegaTagRelocalization(Follower follower) {
        if (!USE_MEGATAG_RELOCALIZATION) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return;

        double visionX = botpose.getPosition().x * 39.3701;
        double visionY = botpose.getPosition().y * 39.3701;

        // Blend current odom pose toward vision pose
        double currentX = follower.getPose().getX();
        double currentY = follower.getPose().getY();
        double currentH = follower.getPose().getHeading();

        double correctedX = currentX + MEGATAG_RELOCALIZE_TRUST * (visionX - currentX);
        double correctedY = currentY + MEGATAG_RELOCALIZE_TRUST * (visionY - currentY);

        // Heading comes from IMU directly — don't blend vision heading, it's noisy
        follower.setPose(new Pose(correctedX, correctedY, currentH));
    }
    // ========================================================================
    // TURRET ROTATION — MegaTag-enhanced limelight + odometry fallback
    // ========================================================================

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

                        // ── MegaTag path ──────────────────────────────────────────────────────
                        // If MegaTag is enabled and we have a valid field-space pose, compute
                        // the exact angle to the goal from that pose rather than using the raw
                        // bearing. This removes lens-angle error and gives consistent accuracy
                        // across the whole field.
                        if (USE_MEGATAG_POSE) {
                            Pose3D botpose = result.getBotpose();
                            if (botpose != null) {
                                double megaTagX   = botpose.getPosition().x * 39.3701;
                                double megaTagY   = botpose.getPosition().y * 39.3701;
                                double megaTagYaw = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

                                double dx = GoalLocation.getX() - megaTagX;
                                double dy = GoalLocation.getY() - megaTagY;

                                double fieldAngleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));
                                double turretRelativeDeg   = fieldAngleToGoalDeg - megaTagYaw;

                                while (turretRelativeDeg >  180.0) turretRelativeDeg -= 360.0;
                                while (turretRelativeDeg < -180.0) turretRelativeDeg += 360.0;

                                double errorRotations = turretRelativeDeg / 360.0;
                                double velocityAdj    = calculateDesiredGoalAngleLIMELIGHTOffset(follower);

                                if (Math.abs(turretRelativeDeg) > llDeadband) {
                                    turretRotationMotor.setPower(runLimelightPID(errorRotations + velocityAdj));
                                } else {
                                    resetLimelightPID();
                                    turretRotationMotor.setPower(0);
                                }
                                break;
                            }
                            // MegaTag pose unavailable — fall through to bearing-only
                        }

                        // ── Bearing-only path (original behaviour, fallback) ──────────────────
                        double llGoalOffset;
                        if (allianceColor.equals("RED")) {
                            llGoalOffset = -goalOffsetForLimelight;
                        }
                        else {llGoalOffset = goalOffsetForLimelight;}

                        double bearingDegrees = fiducial.getTargetXDegrees() + llGoalOffset;
                        if (Math.abs(bearingDegrees) > llDeadband) {
                            double error = (bearingDegrees + llGoalOffset) / 360.0;
                            double velocityCompensatedAngleAdj = calculateDesiredGoalAngleLIMELIGHTOffset(follower);
                            turretRotationMotor.setPower(runLimelightPID(error + velocityCompensatedAngleAdj));
                        } else {
                            resetLimelightPID();
                            turretRotationMotor.setPower(0);
                        }
                        break;
                    }
                }
            }

            // Limelight lost target — fall back to odometry
            if (!this.limelightTracking) {
                resetLimelightPID();

                double y_goal_distance = follower.getPose().getY() - GoalLocation.getY();
                double x_goal_distance = follower.getPose().getX() - GoalLocation.getX();
                double angle_to_goal = Math.atan2(y_goal_distance, x_goal_distance);
                double turretDesiredRelativeOffset = normalizeAngle(-angle_to_goal + follower.getHeading() + Math.PI);
                runOdomPID(turretDesiredRelativeOffset, follower);
            }
        }
    }

    // ========================================================================
    // VELOCITY LEAD COMPENSATION (unchanged)
    // ========================================================================

    private Pose calculateDesiredGoalPositionODOMETRYOffset(Follower follower) {
        double dx = GoalLocation.getX() - follower.getPose().getX();
        double dy = GoalLocation.getY() - follower.getPose().getY();
        double distanceToTarget = Math.sqrt(dx * dx + dy * dy);
        double shooting_time = distanceToTarget / ARTIFACT_SHOOT_EXIT_VELOCITY;
        double x_velocity = follower.getVelocity().getXComponent();
        double y_velocity = follower.getVelocity().getYComponent();
        return new Pose(x_velocity * shooting_time, y_velocity * shooting_time);
    }

    private double calculateDesiredGoalAngleLIMELIGHTOffset(Follower follower) {
        double dx = GoalLocation.getX() - follower.getPose().getX();
        double dy = GoalLocation.getY() - follower.getPose().getY();
        double distanceToTarget = Math.sqrt(dx * dx + dy * dy);
        double shooting_time = distanceToTarget / ARTIFACT_SHOOT_EXIT_VELOCITY;
        double x_velocity = follower.getVelocity().getXComponent();
        double y_velocity = follower.getVelocity().getYComponent();
        Pose offsetFromActualGoal = new Pose(x_velocity * shooting_time, y_velocity * shooting_time);
        double angleOffset = Math.atan2(
                offsetFromActualGoal.getY() - follower.getPose().getY(),
                offsetFromActualGoal.getX() - follower.getPose().getX()
        );
        return angleOffset / 360.0;
    }

    // ========================================================================
    // PID CONTROLLERS (unchanged)
    // ========================================================================

    private double runOdomPID(double turretDesiredRelativeOffset, Follower follower) {
        double turretDesiredDegrees = Math.toDegrees(turretDesiredRelativeOffset);
        double currentRotations = turretRotationMotor.getCurrentPosition() / TURRET_TICKS_PER_REV - manualTeleopOffset;
        double desiredRotations = turretDesiredDegrees / 360.0;
        double error = desiredRotations - currentRotations;

        double angularVelocity = follower.getAngularVelocity();
        double rotationFeedforward = -odomKff_rotation * angularVelocity;

        if (Math.abs(error) < odomDeadband) {
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
                + rotationFeedforward;

        output = Math.max(-1.0, Math.min(1.0, output));
        turretRotationMotor.setPower(output);

        odomLastError = error;
        odomLastTime = currentTime;
        return error;
    }

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
        while (angle >  1.3  * Math.PI) angle -= 2 * Math.PI;
        while (angle < -0.7 * Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ========================================================================
    // PUBLIC API (unchanged)
    // ========================================================================

    public double targetRPM = 0;

    public void updateManualOffset(double amount_to_change) {
        manualTeleopOffset += amount_to_change;
    }

    public void postTelemetry(Telemetry telemetry) {
        double currentRPM = (flywheelMotor.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);
        boolean rpmReady = (targetRPM > 0) && (rpmError <= RPM_TOLERANCE);

        telemetry.addLine("=== Shooter ===");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", String.format("%.0f", currentRPM));
        telemetry.addData("RPM Zone", lastRPMZone);
        telemetry.addData("Distance to Goal (in)", String.format("%.1f", lastDistanceToGoal));
        telemetry.addData("Ready", rpmReady ? "YES" : "NO");
        telemetry.addLine("=== Turret ===");
        telemetry.addData("Tracking Mode", currentTrackingMode);
        telemetry.addData("Limelight Active", limelightTracking);
        telemetry.addData("MegaTag Enabled", USE_MEGATAG_POSE);
        telemetry.addData("MegaTag Relocalization", USE_MEGATAG_RELOCALIZATION ? "ON" : "OFF");
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
            targetRPM = RPM_FAR;
        } else if (shootingLocation.equals("MEDIUM")) {
            targetRPM = RPM_MEDIUM;
        } else {
            targetRPM = RPM_CLOSE;
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