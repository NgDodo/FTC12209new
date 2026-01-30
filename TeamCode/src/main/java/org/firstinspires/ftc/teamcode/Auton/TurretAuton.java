package org.firstinspires.ftc.teamcode.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "TurretCloseRed - OLD", group = "Autonomous")
@Configurable
public class TurretAuton extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private ElapsedTime autoTimer;
    private ElapsedTime pathTimer;

    // === Hardware ===
    private DcMotor m1; // Intake motor
    private DcMotorEx bR; // Back right drive motor (sorter encoder)
    private DcMotorEx m0; // Sorter motor
    private DcMotorEx m3; // Flywheel motor
    private DcMotorEx m2; // Turret motor
    private Servo s2; // Shooter servo
    private CRServo s3; // Shooter CRServo
    private RevColorSensorV3 intakeColor;
    private RevColorSensorV3 shooterColor;

    // === Vision & IMU ===
    private Limelight3A limelight;
    private IMU imu;

    // === Limelight Configuration ===
    private static final String LIMELIGHT_NAME = "Webcam 2";
    private static final int APRILTAG_PIPELINE = 1;

    // === Turret Configuration ===
    private static final double TURRET_TICKS_PER_REV = 1393.1;

    // === Paths from Pedro Pathing Visualizer ===
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;
    private PathChain Path9, Path10, Path11, Path12, Path13, Path14, Path15, Path16;

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

    // === Non-blocking sorter movement ===
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

    // === Timed color detection ===
    private long colorStartTime = 0;
    private boolean colorActive = false;
    private static final long DETECT_TIME_MS = 25;

    // === Intake control ===
    private boolean intakeRunning = false;
    private boolean manualSorterMode = false;

    // === Flywheel PID ===
    private static final double TICKS_PER_REV = 28.0;
    private double kP = 0.0012;
    private double kI = 0.00001;
    private double kD = 0.0;
    private double kF = 0.00025;
    private double flywheelIntegral = 0;
    private double flywheelLastError = 0;
    private long flywheelLastTime = 0;
    private double targetRPM = 0;

    // === Shooting parameters ===
    private static final double IDLE_RPM = 2000;
    private static final double SHOOTING_RPM = 2550;
    private static final double SPINUP_TIME = 0.75;
    private static final double SHOOT_DURATION = 0.3;
    private static final double SERVO_RETRACT_DELAY = 0.2;
    private static final double SORTER_WAIT_TIME = 0.15;
    private static final double MODE_TOGGLE_WAIT_TIME = 0.75;
    private int shotsComplete = 0;

    // === Empty chamber detection ===
    private long emptyStartTime = 0;
    private boolean emptyDetectionActive = false;
    private static final long EMPTY_DETECT_TIME_MS = 100;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123.1, 123.1, Math.toRadians(36)));

        buildPaths();

        // === Initialize Hardware ===
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        bR = hardwareMap.get(DcMotorEx.class, "bR");
        m0 = hardwareMap.get(DcMotorEx.class, "m0");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        shooterColor = hardwareMap.get(RevColorSensorV3.class, "shooterColor");

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        s3.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{m1, m0, m3}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        // === Turret Setup ===
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        s2.setPosition(0.68);

        // Initialize chambers as empty
        chamberFull[0] = false;
        chamberFull[1] = false;
        chamberFull[2] = false;

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

        autoTimer = new ElapsedTime();
        pathTimer = new ElapsedTime();
        flywheelLastTime = System.nanoTime();
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized with Turret Tracking");
        panelsTelemetry.debug("Starting Pose", "X: 123.1, Y: 123.1, Heading: 36°");
        panelsTelemetry.update(telemetry);
    }

    private void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(123.100, 123.100), new Pose(90, 82.000)))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(90.000, 82.000), new Pose(95, 82.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(95.000, 82.000), new Pose(104.000, 82.000)))
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(104.000, 82.000), new Pose(103.000, 82.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(103.000, 82.000), new Pose(107, 82.000)))
                .setTangentHeadingInterpolation()
                .build();

        Path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(107.00, 82.000), new Pose(106, 82.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(106, 82.000), new Pose(118, 82.000)))
                .setTangentHeadingInterpolation()
                .build();

        Path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(118, 82.000), new Pose(90.000, 82.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(90.000, 82.000), new Pose(95.000, 58)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(95, 60.000), new Pose(104.000, 60.000)))
                .setTangentHeadingInterpolation()
                .build();

        Path11 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(104.000, 58.000), new Pose(103, 58.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path12 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(103, 58.000), new Pose(107.00, 58.000)))
                .setTangentHeadingInterpolation()
                .build();

        Path13 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(107, 58.000), new Pose(106, 58.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path14 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(106, 58.000), new Pose(118.000, 58.000)))
                .setTangentHeadingInterpolation()
                .build();

        Path15 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(118.000, 58.000), new Pose(90.000, 82.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path16 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(90.000, 82.000), new Pose(108.000, 82.000)))
                .setTangentHeadingInterpolation()
                .build();
    }

    @Override
    public void start() {
        autoTimer.reset();
        pathTimer.reset();
        pathState = 0;
        targetRPM = IDLE_RPM;
        follower.followPath(Path1);
    }

    @Override
    public void loop() {
        follower.update();
        updateSorterMovement();
        updateFlywheelPID();

        if (intakeRunning && !shootingMode && !manualSorterMode) {
            autoIntakeColorCheck();
        }

        if (shootingMode) {
            checkChamberEmpty();
        }

        pathState = autonomousPathUpdate();

        // ====================================================================
        // TURRET TRACKING (RUNS CONTINUOUSLY)
        // ====================================================================
        updateTurretTracking();

        int normPos = normalize(bR.getCurrentPosition());
        double currentRPM = (m3.getVelocity() / TICKS_PER_REV) * 60.0;

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shots Complete", shotsComplete);
        panelsTelemetry.debug("X", String.format("%.2f", follower.getPose().getX()));
        panelsTelemetry.debug("Y", String.format("%.2f", follower.getPose().getY()));
        panelsTelemetry.debug("Heading", String.format("%.1f°", Math.toDegrees(follower.getPose().getHeading())));
        panelsTelemetry.debug("Target RPM", targetRPM);
        panelsTelemetry.debug("Current RPM", String.format("%.0f", currentRPM));
        panelsTelemetry.debug("Sorter Pos", normPos);
        panelsTelemetry.debug("Chamber", currentChamber + 1);
        panelsTelemetry.debug("Chambers", String.format("%s/%s/%s",
                chamberFull[0] ? "●" : "○",
                chamberFull[1] ? "●" : "○",
                chamberFull[2] ? "●" : "○"));
        panelsTelemetry.debug("Intake", intakeRunning ? "ON" : "OFF");
        panelsTelemetry.debug("Shooting Mode", shootingMode);
        panelsTelemetry.debug("Auto Time", String.format("%.2f s", autoTimer.seconds()));
        panelsTelemetry.debug("Turret Pos", String.format("%.2f rot", m2.getCurrentPosition() / TURRET_TICKS_PER_REV));
        panelsTelemetry.update(telemetry);
    }

    // ========================================================================
    // TURRET TRACKING LOGIC (From LimeLightXPositionTracking)
    // ========================================================================

    /**
     * Updates turret tracking using Limelight (if AprilTag visible) or odometry
     * This runs continuously throughout the autonomous
     */
    private void updateTurretTracking() {
        Pose GOAL_POST = new Pose(134, 134, 0);
        boolean limelightTracking = false;

        // Try Limelight tracking first (if AprilTag visible)
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == 20) {
                    limelightTracking = true;
                    double bearing = fiducial.getTargetXDegrees();
                    double turretRotatePower = 0.067 * bearing / 20.0;

                    if (Math.abs(bearing) > 2) {
                        m2.setPower(turretRotatePower);
                    } else {
                        m2.setPower(0);
                    }
                    break; // Found tag 20, stop searching
                }
            }
        }

        // If Limelight not tracking, use odometry-based tracking
        if (!limelightTracking) {
            // 1. Calculate component distances from goal
            double y_goal_distance = follower.getPose().getY() - GOAL_POST.getY();
            double x_goal_distance = follower.getPose().getX() - GOAL_POST.getX();

            // 2. Calculate absolute angle to goal in field coordinates
            double angle_to_goal = Math.atan2(y_goal_distance, x_goal_distance);

            // 3. Calculate turret offset relative to robot heading
            double turretDesiredRelativeOffset = normalizeAngle(-angle_to_goal + follower.getHeading() + Math.PI);

            // 4. Move turret to track the goal
            moveTurretToOffset(m2, turretDesiredRelativeOffset);
        }
    }

    /**
     * Moves turret to desired offset angle (odometry-based tracking)
     */
    private double moveTurretToOffset(DcMotorEx turretMotor, double turretDesiredRelativeOffset) {
        double turretDesiredDegrees = Math.toDegrees(turretDesiredRelativeOffset);
        double turretRotations = turretMotor.getCurrentPosition() / TURRET_TICKS_PER_REV;
        double desiredRotations = turretDesiredDegrees / 360.0;
        double error = desiredRotations - turretRotations;

        if (Math.abs(error) > 0.02) { // 0.02 rotations tolerance
            turretMotor.setPower(error / Math.abs(error) * 0.2);
        } else {
            turretMotor.setPower(0);
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

    // ========================================================================
    // AUTONOMOUS PATH STATE MACHINE
    // ========================================================================

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Path 1
                if (!follower.isBusy()) {
                    pathState = 100;
                    pathTimer.reset();
                }
                break;

            // === FIRST SHOOTING SEQUENCE (after Path 1) ===
            case 100:
                if (pathTimer.seconds() < SPINUP_TIME) {
                    targetRPM = SHOOTING_RPM;
                } else {
                    toggleShootingMode();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 101:
                if (pathTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 102:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 103:
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 104:
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 105:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 106:
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 107:
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 108:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 109:
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 110:
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    toggleShootingMode();
                    targetRPM = IDLE_RPM;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 111:
                if (pathTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    follower.followPath(Path2);
                    pathState = 1;
                }
                break;

            // === INTAKE SEQUENCE 1 (Paths 2-7) ===
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(Path3);
                    pathState++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    startIntake();
                    manualSorterMode = true;
                    currentChamber = nextChamber(currentChamber);
                    int target = getChamberPosition(currentChamber, false);
                    startSorterMove(target);
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 3:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    manualSorterMode = false;
                    follower.followPath(Path4);
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Path5);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    manualSorterMode = true;
                    currentChamber = nextChamber(currentChamber);
                    int target = getChamberPosition(currentChamber, false);
                    startSorterMove(target);
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 6:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    manualSorterMode = false;
                    follower.followPath(Path6);
                    pathState++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(Path7);
                    pathState++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(Path8);
                    pathState++;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    pathState = 200;
                    pathTimer.reset();
                }
                break;

            // === SECOND SHOOTING SEQUENCE (after Path 8) ===
            case 200:
                if (pathTimer.seconds() < SPINUP_TIME) {
                    targetRPM = SHOOTING_RPM;
                } else {
                    toggleShootingMode();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 201:
                if (pathTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 202:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 203:
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 204:
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 205:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 206:
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 207:
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 208:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 209:
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 210:
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    toggleShootingMode();
                    targetRPM = IDLE_RPM;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 211:
                if (pathTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    follower.followPath(Path9);
                    pathState = 10;
                }
                break;

            // === INTAKE SEQUENCE 2 (Paths 9-14) ===
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(Path10);
                    pathState++;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    startIntake();
                    manualSorterMode = true;
                    currentChamber = nextChamber(currentChamber);
                    int target = getChamberPosition(currentChamber, false);
                    startSorterMove(target);
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 12:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    manualSorterMode = false;
                    follower.followPath(Path11);
                    pathState++;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(Path12);
                    pathState++;
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    manualSorterMode = true;
                    currentChamber = nextChamber(currentChamber);
                    int target = getChamberPosition(currentChamber, false);
                    startSorterMove(target);
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 15:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    manualSorterMode = false;
                    follower.followPath(Path13);
                    pathState++;
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(Path14);
                    pathState++;
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(Path15);
                    pathState++;
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    pathState = 300;
                    pathTimer.reset();
                }
                break;

            // === THIRD SHOOTING SEQUENCE (after Path 15) ===
            case 300:
                if (pathTimer.seconds() < SPINUP_TIME) {
                    targetRPM = SHOOTING_RPM;
                } else {
                    toggleShootingMode();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 301:
                if (pathTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 302:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 303:
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 304:
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 305:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 306:
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 307:
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 308:
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 309:
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 310:
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    toggleShootingMode();
                    targetRPM = 0;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 311:
                if (pathTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    follower.followPath(Path16);
                    pathState++;
                }
                break;

            case 312:
                if (!follower.isBusy()) {
                    pathState = 999;
                }
                break;

            case 999:
                break;
        }

        return pathState;
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        m0.setPower(0);
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        s2.setPosition(0.68);
        s3.setPower(0);
    }

    // === Shooting Helper Methods ===

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

    // === Flywheel PID ===

    private void updateFlywheelPID() {
        double currentVelocity = m3.getVelocity();
        double currentRPM = (currentVelocity / TICKS_PER_REV) * 60.0;

        long currentTime = System.nanoTime();
        double dt = (currentTime - flywheelLastTime) / 1e9;

        double error = targetRPM - currentRPM;

        flywheelIntegral += error * dt;
        flywheelIntegral = Math.max(-10000, Math.min(10000, flywheelIntegral));

        double derivative = (error - flywheelLastError) / dt;
        double feedforward = kF * targetRPM;
        double pidOutput = (kP * error) + (kI * flywheelIntegral) + (kD * derivative) + feedforward;

        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        m3.setPower(pidOutput);

        flywheelLastError = error;
        flywheelLastTime = currentTime;
    }

    // === Sorter Helper Methods ===

    private void updateSorterMovement() {
        if (!sorterMoving) return;

        int pos = normalize(bR.getCurrentPosition());
        int error = calculateShortestError(pos, sorterTargetPosition);

        if (sorterTimer.milliseconds() > SORTER_TIMEOUT_MS) {
            m0.setPower(0);
            sorterMoving = false;
            sorterSettling = false;
            return;
        }

        if (Math.abs(error) <= PERFECT_TOL) {
            if (!sorterSettling) {
                sorterSettling = true;
                sorterSettleTimer.reset();
                m0.setPower(0);
            }

            if (sorterSettleTimer.milliseconds() >= SETTLE_MS) {
                m0.setPower(0);
                sorterMoving = false;
                sorterSettling = false;
                return;
            }

            if (Math.abs(error) > FINE_TOL) {
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

    // === Intake Control Methods ===

    private void startIntake() {
        intakeRunning = true;
        m1.setPower(1.0);
    }

    private void stopIntake() {
        intakeRunning = false;
        m1.setPower(0);
    }
}