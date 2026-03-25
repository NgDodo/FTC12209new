package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
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

    // ========================================================================
    // FLYWHEEL PID
    // ========================================================================
    public static double flywheelKp = 0.005;
    public static double flywheelKi = 0.00001;
    public static double flywheelKd = 0.0;
    public static double flywheelKF = 0.00035;
    private double flywheelIntegral  = 0;
    private double flywheelLastError = 0;
    private long   flywheelLastTime  = 0;

    // ========================================================================
    // TURRET SERVO PID
    // ========================================================================
    // The GoBilda SuperSpeed continuous servo is driven by power in [-1, 1].
    // The REV Throughbore Encoder (wired into the bL motor port, read-only —
    // exactly like bR is used for the sorter) gives position feedback in ticks.
    //
    // Tuning tips (use FTC Dashboard):
    //   servoKp     — start ~0.003, raise until turret snaps on target without
    //                 oscillating. This is your primary gain.
    //   servoKi     — leave at 0 unless you see a consistent steady-state offset.
    //   servoKd     — add ~0.0002 to damp oscillation once Kp is settled.
    //   servoKf     — static-friction feedforward; adds a small kick in the
    //                 direction of error. Start ~0.03, lower if servo drifts.
    //   servoDeadband — error in ticks where we stop the servo.
    //                   REV Throughbore = 8192 ticks/rev.
    //                   10 ticks ≈ 0.44°. Widen to ~30 if the servo hunts.
    public static double servoKp           = 0.003;
    public static double servoKi           = 0.0;
    public static double servoKd           = 0.0002;
    public static double servoKf           = 0.03;
    public static double servoDeadband     = 10.0;      // ticks
    public static double servoIntegralClamp = 500.0;

    private double servoIntegral  = 0;
    private double servoLastError = 0;
    private long   servoLastTime  = 0;

    // ========================================================================
    // THROUGHBORE ENCODER CONFIGURATION
    // ========================================================================
    // REV Throughbore Encoder = 8192 counts per full revolution.
    // Flip ENCODER_REVERSED to true if the turret reads backwards on your robot.
    public static final double ENCODER_TICKS_PER_REV = 37981.091;
    public static       boolean ENCODER_REVERSED      = false;

    // ========================================================================
    // TURRET LIMELIGHT PID  (bearing-only and MegaTag correction path)
    // ========================================================================
    public static double llKp           = 7.0;
    public static double llKi           = 0.0;
    public static double llKd           = 0.0;
    public static double llKf           = 0.01;
    public static double llDeadband     = 1.0;          // degrees
    public static double llIntegralClamp = 5.0;

    private double llIntegral  = 0;
    private double llLastError = 0;
    private long   llLastTime  = 0;

    // ========================================================================
    // HARDWARE
    // ========================================================================

    // --- Turret rotation ---
    // CRServo:      GoBilda SuperSpeed continuous servo ("s1")
    // DcMotorEx:    bL port used as phantom encoder — NEVER apply power to this.
    //               Pedro Pathing owns the actual bL drive motor; we only
    //               steal the encoder port's tick reading, identical to how
    //               bR is used for the sorter encoder.
    private CRServo   turretServo;
    private DcMotorEx turretEncoderPort;   // "bL" — encoder reads only, no power

    // --- Flywheel ---
    // Two motors, physically mirrored (one faces left, one faces right).
    // To push the ball the same direction both motors must spin oppositely.
    // If the ball is ejected the wrong way after build, flip FLYWHEEL_M3_REVERSED.
    private DcMotorEx flywheelMotorA;               // m2  (left-facing)
    private DcMotorEx flywheelMotorB;               // m3  (right-facing)
    public static boolean FLYWHEEL_M3_REVERSED = true;   // tune after assembly

    // ========================================================================
    // VISION & IMU
    // ========================================================================
    private Limelight3A limelight;
    private IMU         imu;
    private static final String LIMELIGHT_NAME    = "Webcam 2";
    private static final int    APRILTAG_PIPELINE = 1;

    // ========================================================================
    // RPM DISTANCE BRACKETS
    // ========================================================================
    public static double DIST_THRESHOLD_CLOSE  = 70.0;   // inches
    public static double DIST_THRESHOLD_MEDIUM = 120.0;  // inches
    public static double RPM_CLOSE  = 2950;
    public static double RPM_MEDIUM = 3200;
    public static double RPM_FAR    = 3600;
    public static double RPM_FF     = 125;

    // ========================================================================
    // MEGATAG
    // ========================================================================
    public static boolean USE_MEGATAG_POSE = false;

    // ========================================================================
    // SHOOTING PARAMETERS
    // ========================================================================
    public static final double TICKS_PER_REV_FLYWHEEL        = 28.0;
    private static final double RPM_TOLERANCE                 = 100.0;
    public static double ARTIFACT_SHOOT_EXIT_VELOCITY         = 80.0;
    public static double FLYWHEEL_DISTANCE_VELOCITY_RATIO     = 70.0;

    // ========================================================================
    // MEGATAG RELOCALIZATION
    // ========================================================================
    public static boolean USE_MEGATAG_RELOCALIZATION = false;
    public static double  MEGATAG_RELOCALIZE_TRUST   = 0.15;

    // ========================================================================
    // TRACKING MODE & FSM STATE
    // ========================================================================
    public enum TurretTrackingMode { OFF, GOAL_TRACKING, OBELISK_TRACKING }
    public enum TurretMOTIF        { GPP, PGP, PPG, UNKNOWN }
    public enum FlywheelStateFSM   { OFF, AutomaticDistancing, PresetLow1000, Preset3100, Preset3600 }

    public TurretTrackingMode currentTrackingMode = TurretTrackingMode.GOAL_TRACKING;
    public TurretMOTIF        currentMotif        = TurretMOTIF.UNKNOWN;
    public FlywheelStateFSM   flywheelState       = FlywheelStateFSM.OFF;
    public boolean            limelightTracking   = false;

    // ========================================================================
    // GOAL / FIELD POSITIONS
    // ========================================================================
    public Pose   GoalLocation, ObeliskLocation;
    public String allianceColor;
    public double goalOffsetForLimelight = -2.0;

    // ========================================================================
    // MISC STATE
    // ========================================================================
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper  = false;

    private double lastDistanceToGoal = 0.0;
    private String lastRPMZone        = "UNKNOWN";

    public double targetRPM        = 0;
    public double manualTeleopOffset = 0.0;   // rotations offset for limelight path

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public TurretV2(HardwareMap hardwareMap, String _allianceColor) {
        switch (_allianceColor) {
            case "RED":
                GoalLocation         = FIELD_CONSTANTS.RED_GOAL_POST;
                goalOffsetForLimelight = -2.0;
                break;
            case "BLUE":
                GoalLocation         = FIELD_CONSTANTS.BLUE_GOAL_POST;
                goalOffsetForLimelight = 1.5;
                break;
            default:
                GoalLocation         = FIELD_CONSTANTS.BLUE_GOAL_POST;
                goalOffsetForLimelight = 1.5;
                break;
        }
        ObeliskLocation = FIELD_CONSTANTS.OBELISK_LOCATION;
        this.allianceColor = _allianceColor;

        // --- Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        // --- Turret servo ---
        turretServo = hardwareMap.get(CRServo.class, "s1");
        turretServo.setPower(0);

        // --- Turret encoder (bL port, phantom — read only, exactly like bR for sorter) ---
        turretEncoderPort = hardwareMap.get(DcMotorEx.class, "bL");
        turretEncoderPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoderPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Power is never set on turretEncoderPort. Pedro Pathing controls the
        // actual bL drive motor through its own references; this object only
        // reads the port's encoder counts.

        // --- Flywheel motors (m2 = left-facing, m3 = right-facing) ---
        flywheelMotorA = hardwareMap.get(DcMotorEx.class, "m2");
        flywheelMotorB = hardwareMap.get(DcMotorEx.class, "m3");

        // Both motors must push the ball in the same direction. Because they
        // are physically mirrored, one runs forward and the other reversed.
        // m2 is set FORWARD here as the reference direction; m3 is reversed
        // by default (FLYWHEEL_M3_REVERSED = true). Flip the flag if the ball
        // ejects backwards after assembly.
        flywheelMotorA.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotorB.setDirection(
                FLYWHEEL_M3_REVERSED ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD);

        flywheelMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // m3 has no encoder — use m2 velocity for RPM feedback (both motors
        // are the same model and will spin at the same speed under the same
        // power once balanced).
        flywheelMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- PID timers ---
        flywheelLastTime = System.nanoTime();
        servoLastTime    = System.nanoTime();
        llLastTime       = System.nanoTime();

        flywheelState       = FlywheelStateFSM.OFF;
        currentTrackingMode = TurretTrackingMode.GOAL_TRACKING;
        currentMotif        = TurretMOTIF.UNKNOWN;
    }

    // ========================================================================
    // PUBLIC UPDATE — autonomous (no gamepad)
    // ========================================================================
    public void updateTurret(Follower follower) {
        updateMegaTagRelocalization(follower);
        updateTurretRotation(follower);
        updateFlywheelSpeedBasedOnDistance(follower);
        flywheelState = FlywheelStateFSM.AutomaticDistancing;
    }

    // ========================================================================
    // PUBLIC UPDATE — teleop (with gamepad)
    // ========================================================================
    public void updateTurret(Follower follower, Gamepad gamepad1) {
        boolean leftBumperPressed  = gamepad1.left_bumper;
        boolean rightBumperPressed = gamepad1.right_bumper;

        if (leftBumperPressed && !lastLeftBumper) {
            flywheelState = FlywheelStateFSM.PresetLow1000;
        }
        if (rightBumperPressed && !lastRightBumper) {
            flywheelState = FlywheelStateFSM.AutomaticDistancing;
        }

        updateMegaTagRelocalization(follower);
        updateTurretRotation(follower);
        updateFlywheelSpeedBasedOnDistance(follower);

        lastLeftBumper  = leftBumperPressed;
        lastRightBumper = rightBumperPressed;
    }

    // ========================================================================
    // TURRET ROTATION — servo + throughbore encoder PID
    // ========================================================================
    private void updateTurretRotation(Follower follower) {

        // --- OBELISK tracking (odometry until tag seen) ---
        if (currentTrackingMode == TurretTrackingMode.OBELISK_TRACKING
                && currentMotif == TurretMOTIF.UNKNOWN) {

            double y_dist     = follower.getPose().getY() - ObeliskLocation.getY();
            double x_dist     = follower.getPose().getX() - ObeliskLocation.getX();
            double angle      = Math.atan2(y_dist, x_dist);
            double normalized = normalizeAngle(-angle + follower.getHeading() + Math.PI);
            runServoPID(normalizedAngleToTicks(normalized));

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                    switch (f.getFiducialId()) {
                        case 21: currentMotif = TurretMOTIF.GPP;
                            currentTrackingMode = TurretTrackingMode.GOAL_TRACKING; break;
                        case 22: currentMotif = TurretMOTIF.PGP;
                            currentTrackingMode = TurretTrackingMode.GOAL_TRACKING; break;
                        case 23: currentMotif = TurretMOTIF.PPG;
                            currentTrackingMode = TurretTrackingMode.GOAL_TRACKING; break;
                    }
                }
            }
        }

        this.limelightTracking = false;

        // --- GOAL tracking ---
        if (currentTrackingMode == TurretTrackingMode.GOAL_TRACKING) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    boolean isTargetTag =
                            (fiducial.getFiducialId() == 24 && allianceColor.equals("RED")) ||
                                    (fiducial.getFiducialId() == 20 && allianceColor.equals("BLUE"));

                    if (!isTargetTag) continue;
                    this.limelightTracking = true;

                    // --- MegaTag path ---
                    if (USE_MEGATAG_POSE) {
                        Pose3D botpose = result.getBotpose();
                        if (botpose != null) {
                            double megaTagX   = botpose.getPosition().x * 39.3701;
                            double megaTagY   = botpose.getPosition().y * 39.3701;
                            double megaTagYaw = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

                            double dx = GoalLocation.getX() - megaTagX;
                            double dy = GoalLocation.getY() - megaTagY;

                            double fieldAngleDeg   = Math.toDegrees(Math.atan2(dy, dx));
                            double turretRelativeDeg = fieldAngleDeg - megaTagYaw;

                            while (turretRelativeDeg >  180.0) turretRelativeDeg -= 360.0;
                            while (turretRelativeDeg < -180.0) turretRelativeDeg += 360.0;

                            if (Math.abs(turretRelativeDeg) > llDeadband) {
                                double errorTicks    = degreesToTicks(turretRelativeDeg);
                                double velocityAdj   = calculateDesiredGoalAngleLIMELIGHTOffset(follower);
                                double velAdjTicks   = degreesToTicks(velocityAdj * 360.0);
                                // turretServo.setPower(runLimelightPID(errorTicks + velAdjTicks));
                            } else {
                                resetLimelightPID();
                                // turretServo.setPower(0);
                            }
                            break;
                        }
                        // MegaTag pose unavailable — fall through to bearing-only
                    }

                    // --- Bearing-only path (fallback) ---
                    double llGoalOffset = allianceColor.equals("RED")
                            ? -goalOffsetForLimelight : goalOffsetForLimelight;
                    double bearingDegrees = fiducial.getTargetXDegrees() + llGoalOffset;

                    if (Math.abs(bearingDegrees) > llDeadband) {
                        double errorTicks  = degreesToTicks(bearingDegrees + llGoalOffset);
                        double velocityAdj = calculateDesiredGoalAngleLIMELIGHTOffset(follower);
                        double velAdjTicks = degreesToTicks(velocityAdj * 360.0);
                        // turretServo.setPower(runLimelightPID(errorTicks + velAdjTicks));
                    } else {
                        resetLimelightPID();
                        // turretServo.setPower(0);
                    }
                    break;
                }
            }

            // --- Limelight lost target — fall back to odometry servo PID ---
            if (!this.limelightTracking) {
                resetLimelightPID();

                double y_dist     = follower.getPose().getY() - GoalLocation.getY();
                double x_dist     = follower.getPose().getX() - GoalLocation.getX();
                double angle      = Math.atan2(y_dist, x_dist);
                double normalized = normalizeAngle(-angle + follower.getHeading() + Math.PI);
                runServoPID(normalizedAngleToTicks(normalized));
            }
        }
    }

    // ========================================================================
    // SERVO PID  (replaces the old motor odom PID)
    // ========================================================================
    // error is in raw encoder ticks. The PID output is clamped to [-1, 1] and
    // sent directly to the CRServo.
    private void runServoPID(int targetTicks) {
        int rawTicks   = turretEncoderPort.getCurrentPosition();
        int currentPos = ENCODER_REVERSED ? -rawTicks : rawTicks;
        int error      = targetTicks - currentPos;

        if (Math.abs(error) < servoDeadband) {
            // turretServo.setPower(0);
            resetServoPID();
            return;
        }

        long   currentTime = System.nanoTime();
        double dt          = (currentTime - servoLastTime) / 1e9;
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        servoIntegral += error * dt;
        servoIntegral  = Math.max(-servoIntegralClamp, Math.min(servoIntegralClamp, servoIntegral));

        double derivative = (error - servoLastError) / dt;
        double feedforward = servoKf * Math.signum(error);

        double output = (servoKp * error)
                + (servoKi * servoIntegral)
                + (servoKd * derivative)
                + feedforward;

        output = Math.max(-1.0, Math.min(1.0, output));
        // turretServo.setPower(output);

        servoLastError = error;
        servoLastTime  = currentTime;
    }

    private void resetServoPID() {
        servoIntegral  = 0;
        servoLastError = 0;
        servoLastTime  = System.nanoTime();
    }

    // ========================================================================
    // LIMELIGHT PID  (output goes to turretServo)
    // ========================================================================
    // Error here is in ticks (same unit as the servo PID) so the output
    // scale is consistent regardless of which control path is active.
    private double runLimelightPID(double errorTicks) {
        long   currentTime = System.nanoTime();
        double dt          = (currentTime - llLastTime) / 1e9;
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        llIntegral += errorTicks * dt;
        llIntegral  = Math.max(-llIntegralClamp, Math.min(llIntegralClamp, llIntegral));

        double derivative  = (errorTicks - llLastError) / dt;
        double feedforward = llKf * Math.signum(errorTicks);

        double output = (llKp * errorTicks)
                + (llKi * llIntegral)
                + (llKd * derivative)
                + feedforward;

        output = Math.max(-1.0, Math.min(1.0, output));
        llLastError = errorTicks;
        llLastTime  = currentTime;
        return output;
    }

    private void resetLimelightPID() {
        llIntegral  = 0;
        llLastError = 0;
        llLastTime  = System.nanoTime();
    }

    // ========================================================================
    // ANGLE / TICK CONVERSION HELPERS
    // ========================================================================
    // Convert a normalized angle (radians, [-π, π]) to encoder ticks.
    private int normalizedAngleToTicks(double normalizedRadians) {
        double rotations = normalizedRadians / (2.0 * Math.PI);
        return (int) (rotations * ENCODER_TICKS_PER_REV);
    }

    // Convert degrees to encoder ticks (used for limelight bearing error).
    private double degreesToTicks(double degrees) {
        return (degrees / 360.0) * ENCODER_TICKS_PER_REV;
    }

    // ========================================================================
    // FLYWHEEL — dual motor, RPM feedback from m2 only
    // ========================================================================
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
                targetRPM   = 0;
                lastRPMZone = "OFF";
                break;
            case AutomaticDistancing:
                double[] goalPos      = getMegaTagGoalDistance(follower);
                lastDistanceToGoal    = goalPos[0];
                targetRPM             = getRPMForDistance(lastDistanceToGoal) + RPM_FF;
                break;
            case PresetLow1000:
                targetRPM   = 1000;
                lastRPMZone = "MANUAL";
                break;
            case Preset3100:
                targetRPM   = 3100;
                lastRPMZone = "MANUAL";
                break;
            case Preset3600:
                targetRPM   = 3600;
                lastRPMZone = "MANUAL";
                break;
        }
    }

    private double[] getMegaTagGoalDistance(Follower follower) {
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        if (USE_MEGATAG_POSE) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    robotX = botpose.getPosition().x * 39.3701;
                    robotY = botpose.getPosition().y * 39.3701;
                }
            }
        }

        double dx = robotX - GoalLocation.getX();
        double dy = robotY - GoalLocation.getY();
        return new double[]{ Math.sqrt(dx * dx + dy * dy), robotX, robotY };
    }

    private void updateFlywheelSpeedBasedOnDistance(Follower follower) {
        _calculateDesiredFlywheelRPM(follower);

        // RPM feedback comes from m2 (flywheelMotorA) — the only motor with
        // an encoder. Both motors receive identical power commands.
        double currentVelocity = flywheelMotorA.getVelocity();
        double currentRPM      = (currentVelocity / TICKS_PER_REV_FLYWHEEL) * 60.0;

        long   currentTime = System.nanoTime();
        double dt          = (currentTime - flywheelLastTime) / 1e9;
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        double error      = targetRPM - currentRPM;
        flywheelIntegral += error * dt;
        flywheelIntegral  = Math.max(-10000, Math.min(10000, flywheelIntegral));

        double derivative  = (error - flywheelLastError) / dt;
        double feedforward = flywheelKF * targetRPM;
        double pidOutput   = (flywheelKp * error)
                + (flywheelKi * flywheelIntegral)
                + (flywheelKd * derivative)
                + feedforward;

        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        // Apply to both motors — they share direction setup from the constructor.
        flywheelMotorA.setPower(pidOutput);
        flywheelMotorB.setPower(pidOutput);

        flywheelLastError = error;
        flywheelLastTime  = currentTime;
    }

    // ========================================================================
    // MEGATAG RELOCALIZATION
    // ========================================================================
    private void updateMegaTagRelocalization(Follower follower) {
        if (!USE_MEGATAG_RELOCALIZATION) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return;

        double visionX = botpose.getPosition().x * 39.3701;
        double visionY = botpose.getPosition().y * 39.3701;

        double currentX = follower.getPose().getX();
        double currentY = follower.getPose().getY();
        double currentH = follower.getPose().getHeading();

        double correctedX = currentX + MEGATAG_RELOCALIZE_TRUST * (visionX - currentX);
        double correctedY = currentY + MEGATAG_RELOCALIZE_TRUST * (visionY - currentY);

        follower.setPose(new Pose(correctedX, correctedY, currentH));
    }

    // ========================================================================
    // VELOCITY LEAD COMPENSATION
    // ========================================================================
    private double calculateDesiredGoalAngleLIMELIGHTOffset(Follower follower) {
        double dx               = GoalLocation.getX() - follower.getPose().getX();
        double dy               = GoalLocation.getY() - follower.getPose().getY();
        double distanceToTarget = Math.sqrt(dx * dx + dy * dy);
        double shooting_time    = distanceToTarget / ARTIFACT_SHOOT_EXIT_VELOCITY;
        double x_velocity       = follower.getVelocity().getXComponent();
        double y_velocity       = follower.getVelocity().getYComponent();
        Pose offset = new Pose(x_velocity * shooting_time, y_velocity * shooting_time);
        double angleOffset = Math.atan2(
                offset.getY() - follower.getPose().getY(),
                offset.getX() - follower.getPose().getX()
        );
        return angleOffset / 360.0;
    }

    // ========================================================================
    // ANGLE NORMALIZATION
    // ========================================================================
    private double normalizeAngle(double angle) {
        while (angle >  1.3 * Math.PI) angle -= 2 * Math.PI;
        while (angle < -0.7 * Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ========================================================================
    // PUBLIC API
    // ========================================================================
    public void updateManualOffset(double amount_to_change) {
        manualTeleopOffset += amount_to_change;
    }

    public void setFlywheelRPM(String shootingLocation) {
        if (shootingLocation.equals("FAR"))         targetRPM = RPM_FAR;
        else if (shootingLocation.equals("MEDIUM"))  targetRPM = RPM_MEDIUM;
        else                                         targetRPM = RPM_CLOSE;
    }

    public void setFlywheelRPM(int desiredFlywheelRPM) {
        targetRPM = desiredFlywheelRPM;
    }

    public void stopFlywheel() {
        targetRPM = 0;
        flywheelMotorA.setPower(0);
        flywheelMotorB.setPower(0);
    }

    public boolean flywheelReachedDesiredRPM() {
        double currentRPM = (flywheelMotorA.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
        return Math.abs(targetRPM - currentRPM) < RPM_TOLERANCE;
    }
    public DcMotorEx getFlywheelMotor() {
        return flywheelMotorA;
    }

    public double getFlywheelRPM() {
        return (flywheelMotorA.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
    }

    // Returns the current turret angle in degrees based on encoder ticks.
    public double getTurretAngleDegrees() {
        int ticks = ENCODER_REVERSED
                ? -turretEncoderPort.getCurrentPosition()
                :  turretEncoderPort.getCurrentPosition();
        return (ticks / ENCODER_TICKS_PER_REV) * 360.0;
    }

    // ========================================================================
    // TELEMETRY
    // ========================================================================
    public void postTelemetry(Telemetry telemetry, Follower follower) {
        double currentRPM = getFlywheelRPM();
        double rpmError   = Math.abs(targetRPM - currentRPM);
        boolean rpmReady  = (targetRPM > 0) && (rpmError <= RPM_TOLERANCE);

        double y_dist     = follower.getPose().getY() - GoalLocation.getY();
        double x_dist     = follower.getPose().getX() - GoalLocation.getX();

        double angle      = Math.atan2(y_dist, x_dist);
        double normalized = normalizeAngle(-angle + follower.getHeading() + Math.PI);

        int rawTicks   = turretEncoderPort.getCurrentPosition();
        int currentPos = ENCODER_REVERSED ? -rawTicks : rawTicks;
        int error      = normalizedAngleToTicks(normalized) - currentPos;

        telemetry.addData("Encoder Ticks (raw)",    turretEncoderPort.getCurrentPosition());
        telemetry.addData("Ticks Left to Goal", error);
        telemetry.addData("Turret Angle (deg)",     String.format("%.1f", getTurretAngleDegrees()));
        telemetry.addLine();

        /*
        if (setting.equals("TELEOP")) {
            telemetry.addLine("=== Shooter ===");
            telemetry.addData("Target RPM",             targetRPM);
            telemetry.addData("Actual RPM",             String.format("%.0f", currentRPM));
            telemetry.addData("RPM Zone",               lastRPMZone);
            telemetry.addData("Distance to Goal (in)",  String.format("%.1f", lastDistanceToGoal));
            telemetry.addData("Ready",                  rpmReady ? "YES" : "NO");
            telemetry.addLine("=== Turret ===");
            telemetry.addData("Tracking Mode",          currentTrackingMode);
            telemetry.addData("Limelight Active",       limelightTracking);
            telemetry.addData("MegaTag Enabled",        USE_MEGATAG_POSE);
            telemetry.addData("MegaTag Relocalization", USE_MEGATAG_RELOCALIZATION ? "ON" : "OFF");
            telemetry.addData("Encoder Ticks (raw)",    turretEncoderPort.getCurrentPosition());
            telemetry.addData("Turret Angle (deg)",     String.format("%.1f", getTurretAngleDegrees()));
            telemetry.addLine();
        }
        */
    }

    public void postTelemetryOnlyGoalPose(Telemetry telemetry, Follower follower) {
        double y_goal_distance = follower.getPose().getY() - GoalLocation.getY();
        double x_goal_distance = follower.getPose().getX() - GoalLocation.getX();
        telemetry.addData("Desired Goal X", x_goal_distance);
        telemetry.addData("Desired Goal Y", y_goal_distance);
        telemetry.addLine();
    }
}