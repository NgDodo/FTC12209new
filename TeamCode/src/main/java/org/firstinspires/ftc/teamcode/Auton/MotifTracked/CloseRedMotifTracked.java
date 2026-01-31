package org.firstinspires.ftc.teamcode.Auton.MotifTracked;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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

@Autonomous(name = "Close Red---Motif Tracked", group = "Autonomous")
@Configurable
public class CloseRedMotifTracked extends OpMode {

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

    // === Vision & IMU ===
    private Limelight3A limelight;
    private IMU imu;
    private static final String LIMELIGHT_NAME = "Webcam 2";
    private static final int APRILTAG_PIPELINE = 1;

    // === Turret Configuration ===
    private static final double TICKS_PER_REV = 1393.1;

    // === Tracking Mode ===
    private enum TrackingMode {
        OFF,
        LIMELIGHT_AND_ODOMETRY
    }

    // === Paths ===
    private PathChain Path1; // Move to shooting position
    private PathChain Path2; // Move to parking position

    // === Sorter constants ===
    private static final int FULL_ROT = 8192;
    private static final int SLOT = FULL_ROT / 3;
    private static final int OFFSET = (int) (FULL_ROT / 2);
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
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private double kP = 0.0012;
    private double kI = 0.00001;
    private double kD = 0.0;
    private double kF = 0.00025;
    private double flywheelIntegral = 0;
    private double flywheelLastError = 0;
    private long flywheelLastTime = 0;
    private double targetRPM = 0;

    // === Timed color detection ===
    private long colorStartTime = 0;
    private boolean colorActive = false;
    private static final long DETECT_TIME_MS = 25;

    // === Empty chamber detection ===
    private long emptyStartTime = 0;
    private boolean emptyDetectionActive = false;
    private static final long EMPTY_DETECT_TIME_MS = 100;

    // === Auto Shoot Sequence ===
    private boolean autoShootActive = false;
    private int autoShootState = 0;
    private ElapsedTime autoShootTimer = new ElapsedTime();
    private boolean lastAButton = false;

    // Auto shoot timing constants (from autonomous)
    private static final double SHOOT_DURATION = 0.45;
    private static final double SERVO_RETRACT_DELAY = 0.2;
    private static final double SORTER_WAIT_TIME = 0.15;
    private static final double MODE_TOGGLE_WAIT_TIME = 0.75;
    private static final double SHOOTING_RPM = 2200;
    private static final double SPINUP_TIME = 2.0; // Seconds to wait for flywheel
    private int shotsComplete = 0;

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

    // ========================================================================
    // CHAMBER COLOR TRACKING SYSTEM
    // ========================================================================

    // The sorter has 3 chambers labeled A, B, C (conceptually, moving clockwise)
    //
    // IMPORTANT CONCEPT:
    // - The chamber POSITIONS are fixed: A is always at index 0, B at 1, C at 2
    // - But the CONTENTS rotate when the sorter physically rotates
    // - When rotating clockwise: A→B, B→C, C→A (contents shift, not labels)
    // - chamberColors[0] always represents the "front" chamber (intake or shooter position)
    //
    // INTAKE MODE:
    // - Chamber at position A (index 0) is at the intake
    // - When ball detected, chamberColors[0] is updated with color
    // - Sorter rotates to next empty chamber
    //
    // SHOOTING MODE:
    // - Entire sorter shifts 60 degrees (OFFSET)
    // - Chamber at position A (index 0) is now at the shooter
    // - When ball shoots, chamberColors[0] is cleared to "NONE"
    //
    // COLOR UPDATE:
    // - Only chamberColors[0] is actively updated based on sensor
    // - Other positions maintain their last known color until rotated to position A

    private String[] chamberColors = {"NONE", "NONE", "NONE"};
    private enum MOTIF {
        GPP,
        PGP,
        PPG
    }

    private MOTIF currentMotif;

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
                .addPath(new BezierLine(new Pose(88.000, 88.000), new Pose(100.000, 75.000)))
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

        // === Limelight Setup ===
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        flywheelLastTime = System.nanoTime();

        /// TODO: Write code to initialize current motif in state 1
        currentMotif = MOTIF.GPP;
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
        double currentRPM = (m3.getVelocity() / FLYWHEEL_TICKS_PER_REV) * 60.0;

        // ====================================================================
        // COLOR DETECTION
        // ====================================================================

        // In intake mode: detect when ball enters and fill chamber
        if (!shootingMode) {
            autoIntakeColorCheck();
        }
        // In shooting mode: detect when ball leaves chamber
        else {
            checkChamberEmpty();
        }

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
                if (!autoShootActive) {
                    autoShootActive = true;
                    autoShootState = 0;
                    shotsComplete = 0;
                    autoShootTimer.reset();

                    if (!shootingMode) {
                        shootingMode = true;
                        // Find which chamber (A, B, or C) has the green ball
                        int greenIndex = indexOfColor(chamberColors, "GREEN", true);

                        // Figure out how much extra to turn chamber, based on motif
                        int rotationCompensateForMotif = 0;
                        if (currentMotif.equals(MOTIF.GPP)) {
                            rotationCompensateForMotif = -1;
                        }
                        if (currentMotif.equals(MOTIF.PGP)) {
                            rotationCompensateForMotif = 1;
                        }
                        if (currentMotif.equals(MOTIF.PPG)) {
                            rotationCompensateForMotif = 0;
                        }

                        if (greenIndex != -1) {  // Found a green ball (or next best if no green)
                            // Calculate how many rotations needed to bring that chamber to position A
                            int rotationsNeeded = 0 + rotationCompensateForMotif;

                            if (greenIndex == 0) {
                                // Green is already in A, no rotation needed
                                rotationsNeeded = 0 + rotationCompensateForMotif;
                            } else if (greenIndex == 1) {
                                // Green is in B, need to rotate CCW once to make B→A
                                rotationsNeeded = -1 + rotationCompensateForMotif;  // Negative = counter-clockwise
                            } else if (greenIndex == 2) {
                                // Green is in C, need to rotate CW once to make C→A
                                // OR rotate CCW twice (but CW is shorter)
                                rotationsNeeded = -2 + rotationCompensateForMotif;  // Positive = clockwise
                            }

                            // Apply the rotations to the array
                            for (int i = 0; i < Math.abs(rotationsNeeded); i++) {
                                if (rotationsNeeded > 0) {
                                    rotateChamberColorsClockwise(); // rotates chamberColors[] clockwise
                                    currentChamber = nextChamber(currentChamber); // CW rotation = prev chamber
                                } else if (rotationsNeeded < 0) {
                                    rotateChamberColorsCounterClockwise(); // rotates chamberColors[] counterclockwise
                                    currentChamber = prevChamber(currentChamber); // CCW rotation = next chamber
                                }
                            }

                            // Calculate the new target position in shooting mode
                            int targetPos = getChamberPosition(currentChamber, shootingMode);
                            startSorterMove(targetPos);
                        }
                    }
                }

                // === Auto Shoot State Machine ===
                if (autoShootActive) {
                    updateAutoShootSequence(); // Switch to PATH_2 happens internally
                }
                pathState = 12;
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
        double currentRPM = (currentVelocity / FLYWHEEL_TICKS_PER_REV) * 60.0;

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
    private int prevChamber(int c) {
        if (c == 0) return 1;  // Backwards from 0 is 1
        if (c == 1) return 2;  // Backwards from 1 is 2
        return 0;              // c == 2, backwards is 0
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

    // ========================================================================
    // AUTOMATIC INTAKE COLOR DETECTION
    // ========================================================================

    private String detectIntakeColor() {
        int r = intakeColor.red();
        int g = intakeColor.green();
        int b = intakeColor.blue();

        // Green ball: green channel dominant and in valid range
        if (g > r && g > b && g > 80 && g < 600) return "GREEN";

        // Purple ball: blue channel dominant and in valid range
        if (b > r && b > g && b > 80 && b < 600) return "PURPLE";

        return "NONE";  // No ball or unrecognized color
    }

    /**
     * Continuously checks for ball at intake during intake mode
     * When ball detected for DETECT_TIME_MS, fills current chamber and rotates to next
     * Uses timed detection to avoid false positives from brief color flashes
     */
    private void autoIntakeColorCheck() {
        // Don't check while sorter is moving (wait for it to settle)
        if (sorterMoving) return;

        String detected = detectIntakeColor();

        // No ball detected - reset timer
        if (detected.equals("NONE")) {
            colorActive = false;
            colorStartTime = 0;
            return;
        }

        // Ball detected - start/continue timer
        if (!colorActive) {
            colorActive = true;
            colorStartTime = System.currentTimeMillis();
        }

        // Ball has been detected continuously for required time
        if (System.currentTimeMillis() - colorStartTime >= DETECT_TIME_MS) {
            // Only fill chamber if it's not already full
            if (!chamberFull[currentChamber]) {
                chamberFull[currentChamber] = true;           // Mark chamber as full
                chamberColors[0] = detected;                  // Mark chamber A color (at intake position)
                rotateChamberColorsClockwise();
                currentChamber = nextChamber(currentChamber); // Move to next chamber

                // Rotate sorter to position next empty chamber at intake
                int target = getChamberPosition(currentChamber, false);
                startSorterMove(target);
            }

            // Reset detection timer
            colorActive = false;
            colorStartTime = 0;
        }
    }
    // ========================================================================
    // CHAMBER SORTING OPERATIONS
    // ========================================================================

    /**
     * Helper class for rotating chamber color arrays
     * Currently unused - was intended for tracking ball colors as chambers rotate
     * Kept for potential future implementation
     */
    /**
     * Rotates chamber array clockwise
     * Example: [A, B, C] → [C, A, B]
     */
    private void rotateChamberColorsClockwise () {
        String[] out = {chamberColors[0], chamberColors[1], chamberColors[2]};
        chamberColors[0] = out[2];
        chamberColors[1] = out[0];
        chamberColors[2] = out[1];
    }
    private void rotateChamberColorsCounterClockwise () {
        String[] out = {chamberColors[0], chamberColors[1], chamberColors[2]};
        chamberColors[0] = out[1];
        chamberColors[1] = out[2];
        chamberColors[2] = out[0];
    }

    /**
     * Auto shoot sequence - shoots all 3 balls automatically
     * Based on autonomous shooting sequence
     */
    private void updateAutoShootSequence() {
        switch (autoShootState) {
            case 0: // Wait for mode toggle to complete
                if (autoShootTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    rotateSorter();
                    rotateChamberColorsClockwise();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 1: // Wait for sorter rotation
                if (autoShootTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 2: // Shoot ball 1
                if (autoShootTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    chamberColors[0] = "NONE";
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 3: // Wait for servo retract
                if (autoShootTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
                    rotateChamberColorsClockwise();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 4: // Wait for sorter rotation
                if (autoShootTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 5: // Shoot ball 2
                if (autoShootTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    chamberColors[0] = "NONE";
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 6: // Wait for servo retract
                if (autoShootTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    rotateSorter();
                    rotateChamberColorsClockwise();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 7: // Wait for sorter rotation
                if (autoShootTimer.seconds() >= SORTER_WAIT_TIME) {
                    activateShooter();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 8: // Shoot ball 3
                if (autoShootTimer.seconds() >= SHOOT_DURATION) {
                    deactivateShooter();
                    shotsComplete++;
                    chamberColors[0] = "NONE";
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 9: // Wait for servo retract, then back to intake mode
                if (autoShootTimer.seconds() >= SERVO_RETRACT_DELAY) {
                    toggleShootingMode();
                    autoShootTimer.reset();
                    autoShootState++;
                }
                break;

            case 10: // Wait for mode toggle, then finish
                if (autoShootTimer.seconds() >= MODE_TOGGLE_WAIT_TIME) {
                    autoShootActive = false;
                    autoShootState = 0;
                    shootingMode = false;
                    setAllChambersEmpty();
                    currentState = State.FOLLOW_PATH_2;
                }
                break;
        }
    }
    private void setAllChambersEmpty() {
        chamberFull[0] = false;
        chamberFull[1] = false;
        chamberFull[2] = false;
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
    private int indexOfColor (String[] chamberColors, String desiredColor, boolean returnNextBest) {
        for (int i = 0; i <= 2; i++) {
            if (chamberColors[i].equals(desiredColor)){
                return i;
            }
        }
        if (returnNextBest) {
            for (int i = 0; i<= 2; i++) {
                if (!chamberColors[i].equals("NONE")) {
                    return i;
                }
            }
        }
        return -1;
    }

}