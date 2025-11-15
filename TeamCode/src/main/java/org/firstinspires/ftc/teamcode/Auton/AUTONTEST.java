package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AUTONTEST", group = "Tests")
public class AUTONTEST extends OpMode {

    private Follower follower;
    private Timer timer = new Timer();
    private int state = 0;

    // ========================================
    // === TUNABLE PARAMETERS ===
    // ========================================

    // RPM Settings
    private static final double SHOOTING_RPM = 3000;  // ← DECREASE to reduce overshoot, INCREASE for more power
    private static final double TICKS_PER_REV = 28.0;
    private static final double RPM_TOLERANCE = 100.0; // Increased tolerance for auto

    // Timing Parameters (all in seconds)
    private static final double SPINUP_TIME = 2.0;    // ← Time to wait for flywheel to reach RPM
    private static final double SHOOT_DURATION = 1.0; // ← How long shooter mechanism runs (s2 down + s3 spinning)
    private static final double SERVO_RETRACT_DELAY = 0.6; // ← Wait after stopping shooter before rotating sorter
    private static final double FINAL_RETRACT_DELAY = 1.0; // ← Wait before switching back to intake

    // Rotation Settings
    private static final double ROTATION_ANGLE = -45.0; // ← Rotation angle in degrees (negative = clockwise, positive = counterclockwise)

    // === Drivetrain test path ===
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pose1 = new Pose(-90, 0, Math.toRadians(0)); // back 45 inches
    private Pose pose2; // Will be set in init() using ROTATION_ANGLE
    private Path backwardPath;
    private com.pedropathing.paths.PathChain rotatePath;

    // === Mechanisms ===
    private DcMotorEx m3; // flywheel
    private DcMotor m1, m2, m0; // intakes + sorter
    private Servo s2;
    private CRServo s3;

    // === Color Sensors & LEDs (from TeleOp) ===
    private RevColorSensorV3 intakeColor;
    private RevColorSensorV3 shooterColor;
    private DigitalChannel LED_Red;
    private DigitalChannel LED_Green;
    private DigitalChannel LED_Blue;

    // === Sorter constants (EXACT from TeleOp) ===
    private static final int FULL_ROT = 8192;
    private static final int SLOT = FULL_ROT / 3;
    private static final int OFFSET = (int) (FULL_ROT / 6);
    private static final int CHAMBER_0_POS = 0;
    private static final int CHAMBER_1_POS = SLOT;
    private static final int CHAMBER_2_POS = 2 * SLOT;

    private boolean[] chamberFull = new boolean[3];
    private int currentChamber = 0;
    private boolean shootingMode = false;

    // === Non-blocking sorter movement with jam detection (EXACT from TeleOp) ===
    private boolean sorterMoving = false;
    private int sorterTargetPosition = 0;
    private ElapsedTime sorterTimer = new ElapsedTime();
    private ElapsedTime sorterSettleTimer = new ElapsedTime();
    private boolean sorterSettling = false;
    private static final int COARSE_TOL = 600;
    private static final int FINE_TOL = 180;
    private static final int PERFECT_TOL = 160;
    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = 0.08;
    private static final long SORTER_TIMEOUT_MS = 2000;
    private static final long SETTLE_MS = 100;

    // === Jam detection (EXACT from TeleOp) ===
    private int lastSorterPosition = 0;
    private long lastSorterMoveTime = 0;
    private static final long JAM_CHECK_INTERVAL_MS = 200;
    private static final int MIN_MOVEMENT_TICKS = 50;
    private boolean sorterJammed = false;

    // === Empty chamber detection (EXACT from TeleOp) ===
    private long emptyStartTime = 0;
    private boolean emptyDetectionActive = false;
    private static final long EMPTY_DETECT_TIME_MS = 500;

    @Override
    public void init() {
        // === Initialize Pedro follower ===
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // === Build paths using tunable constants ===
        pose2 = new Pose(-90, 0, Math.toRadians(ROTATION_ANGLE)); // Use tunable rotation angle

        backwardPath = new Path(new BezierLine(startPose, pose1));
        backwardPath.setConstantHeadingInterpolation(startPose.getHeading());

        // === Build rotation path (uses ROTATION_ANGLE constant) ===
        // For a pure rotation, we keep the same X,Y but change heading
        rotatePath = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();

        // === Init hardware (same as TeleOp) ===
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m0 = hardwareMap.get(DcMotor.class, "m0");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");

        for (DcMotor motor : new DcMotor[]{m1, m2, m0}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // === Flywheel motor setup (EXACT from TeleOp) ===
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setVelocityPIDFCoefficients(8.0, 0.5, 0.0, 12.5);

        s3.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setPosition(1.0);

        // === Color Sensors ===
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        shooterColor = hardwareMap.get(RevColorSensorV3.class, "shooterColor");

        // === LEDs ===
        LED_Red = hardwareMap.get(DigitalChannel.class, "LED1");
        LED_Green = hardwareMap.get(DigitalChannel.class, "LED2");
        LED_Blue = hardwareMap.get(DigitalChannel.class, "LED3");
        LED_Red.setMode(DigitalChannel.Mode.OUTPUT);
        LED_Green.setMode(DigitalChannel.Mode.OUTPUT);
        LED_Blue.setMode(DigitalChannel.Mode.OUTPUT);
        LED_Red.setState(false);
        LED_Green.setState(false);
        LED_Blue.setState(false);

        // === Assume all chambers start full ===
        chamberFull[0] = true;
        chamberFull[1] = true;
        chamberFull[2] = true;

        telemetry.addLine("AutoDriveAndShoot3 (Full TeleOp Logic) Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.resetTimer();
        follower.followPath(backwardPath);
        state = 0;
    }

    @Override
    public void loop() {
        follower.update();

        // === Update sorter using TeleOp logic ===
        updateSorterMovement();

        // === Update LEDs and color detection ===
        if (shootingMode) {
            checkChamberEmpty();
        }
        String shooterColorDetected = detectShooterColor();
        updateColorLEDs(shooterColorDetected);
        updateRPMLED();

        // === Main state machine ===
        switch (state) {
            case 0:
                if (!follower.isBusy()) {
                    // arrived at Pose1 → begin shooting sequence
                    timer.resetTimer();
                    simulateY(); // toggle to shooting
                    state = 1;
                }
                break;

            case 1:
                // Spin up flywheel and wait for it to reach target RPM
                setShooterRPM(SHOOTING_RPM);
                if (timer.getElapsedTimeSeconds() > SPINUP_TIME) {
                    timer.resetTimer();
                    simulateA(); // shoot #1
                    state = 2;
                }
                break;

            case 2:
                // Keep shooting mechanism active
                if (timer.getElapsedTimeSeconds() > SHOOT_DURATION) {
                    stopA();                      // retract servo
                    timer.resetTimer();
                    state = 21;                   // wait for servo reset before rotate
                }
                break;

            case 21:
                // Wait for servo to fully retract before rotating sorter
                if (timer.getElapsedTimeSeconds() > SERVO_RETRACT_DELAY) {
                    simulateDpadRight();                     // rotate sorter
                    timer.resetTimer();
                    state = 3;
                }
                break;

            case 3:
                // Wait for sorter to reach next chamber
                if (!sorterMoving || timer.getElapsedTimeSeconds() > 2.0) {
                    simulateA(); // shoot #2
                    timer.resetTimer();
                    state = 4;
                }
                break;

            case 4:
                // Keep shooting mechanism active
                if (timer.getElapsedTimeSeconds() > SHOOT_DURATION) {
                    stopA();
                    timer.resetTimer();
                    state = 41;   // delay before next sorter move
                }
                break;

            case 41:
                // Wait for servo to fully retract before rotating sorter
                if (timer.getElapsedTimeSeconds() > SERVO_RETRACT_DELAY) {
                    simulateDpadRight();
                    timer.resetTimer();
                    state = 5;
                }
                break;

            case 5:
                // Wait for sorter to reach next chamber
                if (!sorterMoving || timer.getElapsedTimeSeconds() > 2.0) {
                    simulateA(); // shoot #3
                    timer.resetTimer();
                    state = 6;
                }
                break;

            case 6:
                // Keep shooting mechanism active
                if (timer.getElapsedTimeSeconds() > SHOOT_DURATION) {
                    stopA();
                    setShooterRPM(0); // turn off flywheel
                    timer.resetTimer();
                    state = 61;       // delay before switching back to intake
                }
                break;

            case 61:
                // Wait before switching sorter back to intake position
                if (timer.getElapsedTimeSeconds() > FINAL_RETRACT_DELAY) {
                    simulateY();      // back to intake
                    timer.resetTimer();
                    state = 7;
                }
                break;

            case 7:
                // Wait for sorter to finish moving back to intake
                if (!sorterMoving || timer.getElapsedTimeSeconds() > 2.0) {
                    // Directly set target pose for rotation
                    follower.setMaxPower(0.5); // Reduce power for controlled rotation
                    follower.followPath(rotatePath, true); // holdEnd = true to maintain final pose
                    timer.resetTimer();
                    state = 8;
                }
                break;

            case 8:
                // Wait for rotation to complete
                // Check if heading is close to target (within 5 degrees)
                double headingError = Math.abs(Math.toDegrees(follower.getPose().getHeading()) - ROTATION_ANGLE);
                if (headingError < 5.0 || timer.getElapsedTimeSeconds() > 5.0) {
                    follower.setMaxPower(1.0); // Reset power
                    follower.breakFollowing(); // Stop following
                    state = 9; // done
                }
                break;

            case 9:
                // done
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Target RPM", SHOOTING_RPM);
        telemetry.addData("RPM", (m3.getVelocity() / TICKS_PER_REV) * 60.0);
        telemetry.addData("Rotation Angle", ROTATION_ANGLE);
        telemetry.addData("Current Heading", String.format("%.2f°", Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Target Heading (pose2)", pose2 != null ? String.format("%.2f°", Math.toDegrees(pose2.getHeading())) : "null");
        telemetry.addData("Heading Error", state == 8 ? String.format("%.2f°", Math.abs(Math.toDegrees(follower.getPose().getHeading()) - ROTATION_ANGLE)) : "N/A");
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Sorter Target", sorterTargetPosition);
        telemetry.addData("Sorter Pos", normalize(m2.getCurrentPosition()));
        telemetry.addData("Sorter Moving", sorterMoving);
        telemetry.addData("Sorter Jammed", sorterJammed);
        telemetry.addData("Chamber", currentChamber + 1);
        telemetry.addData("Ch1/2/3", String.format("%s/%s/%s",
                chamberFull[0] ? "●" : "○",
                chamberFull[1] ? "●" : "○",
                chamberFull[2] ? "●" : "○"));
        telemetry.addData("Shooter Color", shooterColorDetected);
        telemetry.update();
    }

    // === Simulated button actions ===

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
        s2.setPosition(1.0);
        s3.setPower(0.0);
    }

    private void setShooterRPM(double rpm) {
        double ticksPerSec = (rpm / 60.0) * TICKS_PER_REV;
        m3.setVelocity(ticksPerSec);
    }

    // ====================================================================
    // === EXACT TELEOP LOGIC BELOW (unchanged from UnifiedTeleop) ===
    // ====================================================================

    private void updateSorterMovement() {
        if (!sorterMoving) return;

        int pos = normalize(m2.getCurrentPosition());
        int error = calculateShortestError(pos, sorterTargetPosition);

        long currentTime = System.currentTimeMillis();
        if (currentTime - lastSorterMoveTime >= JAM_CHECK_INTERVAL_MS) {
            int positionChange = Math.abs(pos - lastSorterPosition);
            if (positionChange < MIN_MOVEMENT_TICKS && Math.abs(error) > PERFECT_TOL) {
                sorterJammed = true;
                m0.setPower(0);
                sorterMoving = false;
                sorterSettling = false;
                return;
            }
            lastSorterPosition = pos;
            lastSorterMoveTime = currentTime;
        }

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
        if (sorterJammed) return;
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

    private void updateRPMLED() {
        double currentRPM = (m3.getVelocity() / TICKS_PER_REV) * 60.0;
        double rpmError = Math.abs(SHOOTING_RPM - currentRPM); // Check against our shooting RPM
        LED_Red.setState(rpmError <= RPM_TOLERANCE);
    }

    private void updateColorLEDs(String color) {
        if (color.equals("GREEN")) {
            LED_Green.setState(true);
            LED_Blue.setState(false);
        } else if (color.equals("PURPLE")) {
            LED_Green.setState(false);
            LED_Blue.setState(true);
        } else {
            LED_Green.setState(false);
            LED_Blue.setState(false);
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
        switch (chamber) {
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
        m3.setVelocity(0);
    }
}