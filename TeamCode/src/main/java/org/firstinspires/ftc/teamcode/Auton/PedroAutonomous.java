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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime pathTimer;
    private ElapsedTime autoTimer;

    // === Intake & Sorter Hardware ===
    private DcMotor m1, m2; // m1 = intake, m2 = sorter
    private DcMotorEx m0; // Sorter motor
    private RevColorSensorV3 intakeColor;

    // === Sorter constants (from TeleOp) ===
    private static final int FULL_ROT = 8192;
    private static final int SLOT = FULL_ROT / 3;
    private static final int CHAMBER_0_POS = 0;
    private static final int CHAMBER_1_POS = SLOT;
    private static final int CHAMBER_2_POS = 2 * SLOT;

    private boolean[] chamberFull = new boolean[3];
    private int currentChamber = 0;

    // === Sorter movement ===
    private boolean sorterMoving = false;
    private int sorterTargetPosition = 0;
    private ElapsedTime sorterTimer = new ElapsedTime();
    private static final int PERFECT_TOL = 80;
    private static final double MAX_POWER = 0.55;
    private static final double MIN_POWER = 0.08;
    private static final long SORTER_TIMEOUT_MS = 2000;
    private static final int COARSE_TOL = 1000;

    // === Color detection ===
    private long colorStartTime = 0;
    private boolean colorActive = false;
    private static final long DETECT_TIME_MS = 75;

    // === ADJUSTABLE: Intake speeds for paths 2 and 4 ===
    private static final double INTAKE_SPEED = 1.0; // Power for intake motor during paths 2 and 4
    private static final double PATH_2_4_SPEED_MULTIPLIER = 0.2; // Slower speed for paths 2 and 4 (0.0 to 1.0)

    // State machine states
    private enum State {
        IDLE,
        FOLLOW_PATH_1,
        WAIT_AT_PATH_1,
        FOLLOW_PATH_2,
        WAIT_AT_PATH_2,
        FOLLOW_PATH_3,
        WAIT_AT_PATH_3,
        FOLLOW_PATH_4,
        WAIT_AT_PATH_4,
        FOLLOW_PATH_5,
        WAIT_AT_PATH_5,
        FOLLOW_PATH_6,
        WAIT_AT_PATH_6,
        FINISHED
    }

    private State currentState = State.IDLE;

    private static final long WAIT_TIME_MS = 500;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(122, 122, Math.toRadians(45)));

        // === Initialize Intake & Sorter Hardware ===
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m0 = hardwareMap.get(DcMotorEx.class, "m0");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        paths = new Paths(follower);
        pathTimer = new ElapsedTime();
        autoTimer = new ElapsedTime();

        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Total Paths", 6);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        autoTimer.reset();
        currentState = State.FOLLOW_PATH_1;
        follower.followPath(paths.Path1);
        pathTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();

        // Update sorter movement
        updateSorterMovement();

        // Handle intake and color detection during paths 2 and 4
        if (currentState == State.FOLLOW_PATH_2 || currentState == State.FOLLOW_PATH_4) {
            m1.setPower(INTAKE_SPEED);
            m2.setPower(-INTAKE_SPEED);
            autoIntakeColorCheck();
        } else {
            m1.setPower(0);
            m2.setPower(0);
        }

        pathState = autonomousPathUpdate();

        int normPos = normalize(m2.getCurrentPosition());

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Current State", currentState.toString());
        panelsTelemetry.debug("X", String.format("%.2f", follower.getPose().getX()));
        panelsTelemetry.debug("Y", String.format("%.2f", follower.getPose().getY()));
        panelsTelemetry.debug("Heading", String.format("%.2f°", Math.toDegrees(follower.getPose().getHeading())));
        panelsTelemetry.debug("Sorter Pos", normPos);
        panelsTelemetry.debug("Chamber", currentChamber + 1);
        panelsTelemetry.debug("Chambers", String.format("%s/%s/%s",
                chamberFull[0] ? "●" : "○",
                chamberFull[1] ? "●" : "○",
                chamberFull[2] ? "●" : "○"));
        panelsTelemetry.debug("Auto Time", String.format("%.2f s", autoTimer.seconds()));
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(122.000, 122.000), new Pose(88.000, 88.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                    .build();

            // Path 2 - SLOWER for intake
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.000, 88.000),
                                    new Pose(100.000, 84.000),
                                    new Pose(125.000, 83.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setTimeoutConstraint(PATH_2_4_SPEED_MULTIPLIER) // Slower
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.000, 83.000), new Pose(85.000, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                    .build();

            // Path 4 - SLOWER for intake
            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.000, 85.000),
                                    new Pose(99.000, 55.000),
                                    new Pose(125.000, 60.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setTimeoutConstraint(PATH_2_4_SPEED_MULTIPLIER) // Slower
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.000, 60.000), new Pose(85.000, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(85.000, 85.000), new Pose(100.000, 75.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                    .build();
        }
    }

    // === Sorter & Intake Logic (from TeleOp) ===

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

                if (!allChambersFull()) {
                    int nextEmpty = findNextEmptyChamber(currentChamber);
                    if (nextEmpty != -1) {
                        currentChamber = nextEmpty;
                        int target = getChamberPosition(currentChamber);
                        startSorterMove(target);
                    }
                }
            }
            colorActive = false;
            colorStartTime = 0;
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

    private int getChamberPosition(int chamber) {
        switch(chamber) {
            case 0: return CHAMBER_0_POS;
            case 1: return CHAMBER_1_POS;
            case 2: return CHAMBER_2_POS;
            default: return CHAMBER_0_POS;
        }
    }

    private int findNextEmptyChamber(int startChamber) {
        int next = nextChamber(startChamber);
        if (!chamberFull[next]) return next;

        next = nextChamber(next);
        if (!chamberFull[next]) return next;

        return -1;
    }

    private int nextChamber(int c) {
        if (c == 1) return 0;
        if (c == 0) return 2;
        return 1;
    }

    private boolean allChambersFull() {
        return chamberFull[0] && chamberFull[1] && chamberFull[2];
    }

    private void updateSorterMovement() {
        if (!sorterMoving) return;

        int pos = normalize(m2.getCurrentPosition());
        int error = calculateShortestError(pos, sorterTargetPosition);

        if (sorterTimer.milliseconds() > SORTER_TIMEOUT_MS) {
            m0.setPower(0);
            sorterMoving = false;
            return;
        }

        if (Math.abs(error) <= PERFECT_TOL) {
            m0.setPower(0);
            sorterMoving = false;
            return;
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
        sorterTimer.reset();
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

    // === State Machine ===

    public int autonomousPathUpdate() {
        switch (currentState) {
            case IDLE:
                pathState = 0;
                break;

            case FOLLOW_PATH_1:
                pathState = 1;
                if (!follower.isBusy()) {
                    currentState = State.WAIT_AT_PATH_1;
                    pathTimer.reset();
                }
                break;

            case WAIT_AT_PATH_1:
                pathState = 2;
                if (pathTimer.milliseconds() >= WAIT_TIME_MS) {
                    currentState = State.FOLLOW_PATH_2;
                    follower.followPath(paths.Path2);
                    pathTimer.reset();
                }
                break;

            case FOLLOW_PATH_2:
                pathState = 3;
                // Intake is active (handled in loop)
                if (!follower.isBusy()) {
                    currentState = State.WAIT_AT_PATH_2;
                    pathTimer.reset();
                }
                break;

            case WAIT_AT_PATH_2:
                pathState = 4;
                if (pathTimer.milliseconds() >= WAIT_TIME_MS) {
                    currentState = State.FOLLOW_PATH_3;
                    follower.followPath(paths.Path3);
                    pathTimer.reset();
                }
                break;

            case FOLLOW_PATH_3:
                pathState = 5;
                if (!follower.isBusy()) {
                    currentState = State.WAIT_AT_PATH_3;
                    pathTimer.reset();
                }
                break;

            case WAIT_AT_PATH_3:
                pathState = 6;
                if (pathTimer.milliseconds() >= WAIT_TIME_MS) {
                    currentState = State.FOLLOW_PATH_4;
                    follower.followPath(paths.Path4);
                    pathTimer.reset();
                }
                break;

            case FOLLOW_PATH_4:
                pathState = 7;
                // Intake is active (handled in loop)
                if (!follower.isBusy()) {
                    currentState = State.WAIT_AT_PATH_4;
                    pathTimer.reset();
                }
                break;

            case WAIT_AT_PATH_4:
                pathState = 8;
                if (pathTimer.milliseconds() >= WAIT_TIME_MS) {
                    currentState = State.FOLLOW_PATH_5;
                    follower.followPath(paths.Path5);
                    pathTimer.reset();
                }
                break;

            case FOLLOW_PATH_5:
                pathState = 9;
                if (!follower.isBusy()) {
                    currentState = State.WAIT_AT_PATH_5;
                    pathTimer.reset();
                }
                break;

            case WAIT_AT_PATH_5:
                pathState = 10;
                if (pathTimer.milliseconds() >= WAIT_TIME_MS) {
                    currentState = State.FOLLOW_PATH_6;
                    follower.followPath(paths.Path6);
                    pathTimer.reset();
                }
                break;

            case FOLLOW_PATH_6:
                pathState = 11;
                if (!follower.isBusy()) {
                    currentState = State.WAIT_AT_PATH_6;
                    pathTimer.reset();
                }
                break;

            case WAIT_AT_PATH_6:
                pathState = 12;
                if (pathTimer.milliseconds() >= WAIT_TIME_MS) {
                    currentState = State.FINISHED;
                }
                break;

            case FINISHED:
                pathState = 13;
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
    }
}