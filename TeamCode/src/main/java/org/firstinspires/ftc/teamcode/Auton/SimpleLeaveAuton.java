package org.firstinspires.ftc.teamcode.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Simple Leave Auton", group = "Autonomous")
@Configurable
public class SimpleLeaveAuton extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private ElapsedTime autoTimer;
    private ElapsedTime pathTimer;

    // === Hardware ===
    private DcMotor m1; // Intake motor
    private DcMotorEx bR; // Back right drive motor (sorter encoder)
    private DcMotorEx m0; // Sorter motor
    private DcMotorEx m3, m2; // Flywheel motor
    private Servo s2; // Shooter servo
    private CRServo s3; // Shooter CRServo
    private RevColorSensorV3 intakeColor;
    private RevColorSensorV3 shooterColor;

    // === Paths from Pedro Pathing Visualizer ===
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;
    private PathChain Path8, Path9, Path10, Path11, Path12, Path13, Path14;

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
    private static final double SORTER_WAIT_TIME = 0.15; // Time to wait for sorter rotation
    private static final double MODE_TOGGLE_WAIT_TIME = 0.75; // Time to wait for mode toggle
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

        s2.setPosition(0.68);

        // Initialize chambers as empty
        chamberFull[0] = false;
        chamberFull[1] = false;
        chamberFull[2] = false;

        autoTimer = new ElapsedTime();
        pathTimer = new ElapsedTime();
        flywheelLastTime = System.nanoTime();
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Starting Pose", "X: 123.1, Y: 123.1, Heading: 36°");
        panelsTelemetry.update(telemetry);
    }

    private void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(123.100, 123.100), new Pose(85, 123.1)))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(90))
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
        // hold turret state
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setPower(0);


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
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Path 1
                if (!follower.isBusy()) {
                    pathState = 100;
                    pathTimer.reset();
                }
                break;

            // === FIRST SHOOTING SEQUENCE (after Path 1) ===
            case 100: // Spinup flywheel
                if (pathTimer.seconds() < SPINUP_TIME) {
                    targetRPM = SHOOTING_RPM;
                } else {
                    toggleShootingMode();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 101: // FIXED: Wait for mode toggle instead of checking sorterMoving
                if (pathTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 102: // Wait for sorter rotation
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 103: // Wait for shoot duration
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 104: // Wait for servo retract
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 105: // Wait for sorter rotation
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 106: // Shoot 2
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 107: // Wait for servo retract
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 108: // Wait for sorter rotation
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 109: // Shoot 3
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 110: // Wait for servo retract, then back to intake mode
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    toggleShootingMode();
                    targetRPM = IDLE_RPM;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 111: // FIXED: Wait for mode toggle
                if (pathTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    follower.followPath(Path2);
                    pathState = 1;
                }
                break;

            // === INTAKE PATHS 2-7 ===
            case 1: // Path 2
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(Path3);
                    pathState++;
                }
                break;

            case 2: // Path 3 (Intake running continuously)
                if (!follower.isBusy()) {
                    manualSorterMode = true;
                    currentChamber = nextChamber(currentChamber);
                    int target = getChamberPosition(currentChamber, false);
                    startSorterMove(target);
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 3: // Wait for sorter rotation
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    manualSorterMode = false;
                    follower.followPath(Path4);
                    pathState++;
                }
                break;

            case 4: // Path 4 (Intake running continuously)
                if (!follower.isBusy()) {
                    manualSorterMode = true;
                    currentChamber = nextChamber(currentChamber);
                    int target5 = getChamberPosition(currentChamber, false);
                    startSorterMove(target5);
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 5: // Wait for sorter rotation
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    manualSorterMode = false;
                    follower.followPath(Path5);
                    pathState++;
                }
                break;

            case 6: // Path 5 (Intake running continuously)
                if (!follower.isBusy()) {
                    follower.followPath(Path6);
                    pathState++;
                }
                break;

            case 7: // Path 6 (Intake running continuously)
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(Path7);
                    pathState++;
                }
                break;

            case 8: // Path 7
                if (!follower.isBusy()) {
                    pathState = 200;
                    pathTimer.reset();
                }
                break;

            // === SECOND SHOOTING SEQUENCE (after Path 7) ===
            case 200: // Spinup flywheel
                if (pathTimer.seconds() < SPINUP_TIME) {
                    targetRPM = SHOOTING_RPM;
                } else {
                    toggleShootingMode();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 201: // FIXED: Wait for mode toggle
                if (pathTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 202: // Wait for sorter rotation
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

            case 205: // Wait for sorter rotation
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

            case 208: // Wait for sorter rotation
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

            case 210: // Back to intake mode
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    toggleShootingMode();
                    targetRPM = IDLE_RPM;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 211: // FIXED: Wait for mode toggle
                if (pathTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    follower.followPath(Path8);
                    pathState = 9;
                }
                break;

            // === INTAKE PATHS 8-12 ===
            case 9: // Path 8
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(Path9);
                    pathState++;
                }
                break;

            case 10: // Path 9 (Intake running continuously)
                if (!follower.isBusy()) {
                    manualSorterMode = true;
                    currentChamber = nextChamber(currentChamber);
                    int target10 = getChamberPosition(currentChamber, false);
                    startSorterMove(target10);
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 11: // Wait for sorter rotation
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    manualSorterMode = false;
                    follower.followPath(Path10);
                    pathState++;
                }
                break;

            case 12: // Path 10 (Intake running continuously)
                if (!follower.isBusy()) {
                    manualSorterMode = true;
                    currentChamber = nextChamber(currentChamber);
                    int target12 = getChamberPosition(currentChamber, false);
                    startSorterMove(target12);
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 13: // Wait for sorter rotation
                if (pathTimer.seconds() >= SORTER_WAIT_TIME) {
                    manualSorterMode = false;
                    follower.followPath(Path11);
                    pathState++;
                }
                break;

            case 14: // Path 11 (Intake running continuously)
                if (!follower.isBusy()) {
                    follower.followPath(Path12);
                    pathState++;
                }
                break;

            case 15: // Path 12 (Intake running continuously)
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(Path13);
                    pathState++;
                }
                break;

            case 16: // Path 13
                if (!follower.isBusy()) {
                    pathState = 300;
                    pathTimer.reset();
                }
                break;

            // === THIRD SHOOTING SEQUENCE (after Path 13) ===
            case 300: // Spinup flywheel
                if (pathTimer.seconds() < SPINUP_TIME) {
                    targetRPM = SHOOTING_RPM;
                } else {
                    toggleShootingMode();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 301: // FIXED: Wait for mode toggle
                if (pathTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    rotateSorter();
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 302: // Wait for sorter rotation
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

            case 305: // Wait for sorter rotation
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

            case 308: // Wait for sorter rotation
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

            case 310: // Final state
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    toggleShootingMode();
                    targetRPM = 0;
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 311: // FIXED: Wait for mode toggle
                if (pathTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    follower.followPath(Path14);
                    pathState++;
                }
                break;

            case 312: // NEW: Path 14 - Final parking
                if (!follower.isBusy()) {
                    pathState = 999;
                }
                break;

            case 999: // Finished
                break;
        }

        return pathState;
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        m0.setPower(0);
        m1.setPower(0);
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