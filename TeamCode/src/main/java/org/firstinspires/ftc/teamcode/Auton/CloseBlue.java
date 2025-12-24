package org.firstinspires.ftc.teamcode.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Close Blue", group = "Autonomous")
@Configurable
public class CloseBlue extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private ElapsedTime pathTimer;
    private ElapsedTime autoTimer;

    // === Hardware ===
    private DcMotor m1, m2; // m1 = intake, m2 = sorter encoder
    private DcMotorEx m3, m0; // m3 = flywheel, m0 = sorter motor
    private Servo s2; // Shooter servo
    private CRServo s3; // Shooter CRServo
    private RevColorSensorV3 intakeColor;
    private RevColorSensorV3 shooterColor;

    // === Paths ===
    private PathChain Path1; // Move to shooting position
    private PathChain Path2; // Move to parking position

    // === Sorter constants (from TeleOp) ===
    private static final int FULL_ROT = 8192;
    private static final int SLOT = FULL_ROT / 3;
    private static final int OFFSET = (int) (FULL_ROT / 6);
    private static final int CHAMBER_0_POS = 0;
    private static final int CHAMBER_1_POS = SLOT;
    private static final int CHAMBER_2_POS = 2 * SLOT;

    private boolean[] chamberFull = new boolean[3];
    private int currentChamber = 0;
    private boolean shootingMode = false;

    // === Sorter movement ===
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

    private int lastSorterPosition = 0;
    private long lastSorterMoveTime = 0;

    // === Flywheel PID (from TeleOp) ===
    private static final double TICKS_PER_REV = 28.0;
    private double kP = 0.0012;
    private double kI = 0.00001;
    private double kD = 0.0;
    private double kF = 0.00025;
    private double flywheelIntegral = 0;
    private double flywheelLastError = 0;
    private long flywheelLastTime = 0;
    private double targetRPM = 0;

    // === Empty chamber detection ===
    private long emptyStartTime = 0;
    private boolean emptyDetectionActive = false;
    private static final long EMPTY_DETECT_TIME_MS = 100;

    // === TUNABLE PARAMETERS ===
    private static final double SHOOTING_RPM = 2200;
    private static final double SPINUP_TIME = 2.0; // Seconds to wait for flywheel
    private static final double SHOOT_DURATION = 1.0; // How long to keep shooter active
    private static final double SERVO_RETRACT_DELAY = .6; // Wait after stopping shooter

    // State machine
    private enum State {
        IDLE,
        SORT_PRELOADS,
        PREPARE_TO_SHOOT,
        FOLLOW_PATH_1,
        ALIGN_CHAMBER_1,
        SHOOT_1,
        RETRACT_1,
        ALIGN_CHAMBER_2,
        SHOOT_2,
        RETRACT_2,
        ALIGN_CHAMBER_3,
        SHOOT_3,
        RETRACT_3,
        FOLLOW_PATH_2,
        FINISHED
    }

    private State currentState = State.IDLE;
    private int shotsComplete = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(122, 122, Math.toRadians(45)));

        // === Build paths ===
        Path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(122.000, 122.000), new Pose(88.000, 88.000)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(88.000, 88.000), new Pose(76, 101)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        // === Initialize Hardware ===
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m0 = hardwareMap.get(DcMotorEx.class, "m0");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        shooterColor = hardwareMap.get(RevColorSensorV3.class, "shooterColor");

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        s3.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{m1, m2, m3, m0}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        s2.setPosition(0.68);

        // === Assume all chambers start full ===
        chamberFull[0] = true;
        chamberFull[1] = true;
        chamberFull[2] = true;

        pathTimer = new ElapsedTime();
        autoTimer = new ElapsedTime();
        flywheelLastTime = System.nanoTime();

        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        autoTimer.reset();
        pathTimer.reset();
        currentState = State.SORT_PRELOADS;
    }

    @Override
    public void loop() {
        follower.update();
        updateSorterMovement();
        updateFlywheelPID();

        if (shootingMode) {
            checkChamberEmpty();
        }

        pathState = autonomousPathUpdate();

        int normPos = normalize(m2.getCurrentPosition());
        double currentRPM = (m3.getVelocity() / TICKS_PER_REV) * 60.0;

        panelsTelemetry.debug("State", currentState.toString());
        panelsTelemetry.debug("Shots Complete", shotsComplete);
        panelsTelemetry.debug("X", String.format("%.2f", follower.getPose().getX()));
        panelsTelemetry.debug("Y", String.format("%.2f", follower.getPose().getY()));
        panelsTelemetry.debug("Target RPM", targetRPM);
        panelsTelemetry.debug("Current RPM", String.format("%.0f", currentRPM));
        panelsTelemetry.debug("Sorter Pos", normPos);
        panelsTelemetry.debug("Chamber", currentChamber + 1);
        panelsTelemetry.debug("Chambers", String.format("%s/%s/%s",
                chamberFull[0] ? "●" : "○",
                chamberFull[1] ? "●" : "○",
                chamberFull[2] ? "●" : "○"));
        panelsTelemetry.debug("Auto Time", String.format("%.2f s", autoTimer.seconds()));
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (currentState) {
            case IDLE:
                pathState = 0;
                break;

            case SORT_PRELOADS:
                // Sort all preloads to align them
                pathState = 1;
                if (!sorterMoving) {
                    // Cycle through all chambers to ensure sorting
                    if (currentChamber < 2) {
                        currentChamber++;
                        int target = getChamberPosition(currentChamber, false);
                        startSorterMove(target);
                    } else {
                        // Done sorting, prepare to shoot
                        currentChamber = 0; // Reset to chamber 0
                        currentState = State.PREPARE_TO_SHOOT;
                        pathTimer.reset();
                    }
                }
                break;

            case PREPARE_TO_SHOOT:
                // Simulate Y (switch to shooting mode) + Right Bumper (set RPM)
                pathState = 2;
                if (!shootingMode) {
                    simulateY(); // Switch to shooting mode
                }
                targetRPM = SHOOTING_RPM; // Set flywheel RPM

                if (pathTimer.seconds() >= SPINUP_TIME) {
                    // Flywheel spun up, start path 1
                    follower.followPath(Path1);
                    currentState = State.FOLLOW_PATH_1;
                    pathTimer.reset();
                }
                break;

            case FOLLOW_PATH_1:
                pathState = 3;
                if (!follower.isBusy()) {
                    // Arrived at shooting position
                    currentState = State.ALIGN_CHAMBER_1;
                    pathTimer.reset();
                }
                break;

            case ALIGN_CHAMBER_1:
                pathState = 4;
                // Rotate to first chamber (simulate DpadRight)
                if (!sorterMoving) {
                    simulateDpadRight();
                    pathTimer.reset();
                    currentState = State.SHOOT_1;
                }
                break;

            case SHOOT_1:
                pathState = 5;
                // Keep shooter active for entire duration
                simulateA();
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    stopA();
                    shotsComplete++;
                    currentState = State.RETRACT_1;
                    pathTimer.reset();
                }
                break;

            case RETRACT_1:
                pathState = 6;
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    currentState = State.ALIGN_CHAMBER_2;
                    pathTimer.reset();
                }
                break;

            case ALIGN_CHAMBER_2:
                pathState = 7;
                if (!sorterMoving) {
                    simulateDpadRight();
                    pathTimer.reset();
                    currentState = State.SHOOT_2;
                }
                break;

            case SHOOT_2:
                pathState = 8;
                // Keep shooter active for entire duration
                simulateA();
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    stopA();
                    shotsComplete++;
                    currentState = State.RETRACT_2;
                    pathTimer.reset();
                }
                break;

            case RETRACT_2:
                pathState = 9;
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    currentState = State.ALIGN_CHAMBER_3;
                    pathTimer.reset();
                }
                break;

            case ALIGN_CHAMBER_3:
                pathState = 10;
                if (!sorterMoving) {
                    simulateDpadRight();
                    pathTimer.reset();
                    currentState = State.SHOOT_3;
                }
                break;

            case SHOOT_3:
                pathState = 11;
                // Keep shooter active for entire duration
                simulateA();
                if (pathTimer.seconds() >= SHOOT_DURATION) {
                    stopA();
                    targetRPM = 0; // Turn off flywheel
                    shotsComplete++;
                    currentState = State.RETRACT_3;
                    pathTimer.reset();
                }
                break;

            case RETRACT_3:
                pathState = 12;
                if (pathTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    follower.followPath(Path2);
                    currentState = State.FOLLOW_PATH_2;
                    pathTimer.reset();
                }
                break;

            case FOLLOW_PATH_2:
                pathState = 13;
                if (!follower.isBusy()) {
                    currentState = State.FINISHED;
                    simulateY();
                }
                break;

            case FINISHED:
                pathState = 14;
                break;
        }

        return pathState;
    }

    // === Button Simulations ===

    private void simulateY() {
        shootingMode = !shootingMode;
        int target = getChamberPosition(currentChamber, shootingMode);
        startSorterMove(target);
    }

    private void simulateDpadRight() {
        currentChamber = nextChamber(currentChamber);
        int target = getChamberPosition(currentChamber, shootingMode);
        startSorterMove(target);
    }

    private void simulateA() {
        s2.setPosition(0);
        s3.setPower(1.0);
    }

    private void stopA() {
        s2.setPosition(0.68);
        s3.setPower(0.0);
    }

    // === Flywheel PID (from TeleOp) ===

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

    // === Sorter Logic (from TeleOp) ===

    private void updateSorterMovement() {
        if (!sorterMoving) return;

        int pos = normalize(m2.getCurrentPosition());
        int error = calculateShortestError(pos, sorterTargetPosition);

        long currentTime = System.currentTimeMillis();

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
        lastSorterPosition = normalize(m2.getCurrentPosition());
        lastSorterMoveTime = System.currentTimeMillis();
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
}