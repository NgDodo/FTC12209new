package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.arcrobotics.ftclib.controller.PDController;

import java.util.List;

@TeleOp(name = "Limelight Assisted Intake", group = "DriveTrainControl")
public class LimelightAssistedIntake extends OpMode {

    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1, m2;
    DcMotorEx m3, m0;
    CRServo s1, s3;
    Servo s2;

    // === Color Sensors & LEDs ===
    private RevColorSensorV3 intakeColor;
    private RevColorSensorV3 shooterColor;
    private DigitalChannel LED_Red;
    private DigitalChannel LED_Green;
    private DigitalChannel LED_Blue;

    // === Vision & AprilTag ===
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private PDController turretPD;
    private IMU imu;

    // === Limelight Ball Tracking (IMU-based like AprilTag) ===
    private Limelight3A limelight;
    private static final double MIN_TARGET_AREA = 1.0;  // Minimum 1% area - filters small detections
    private boolean limelightAssistActive = false;
    private String detectedColor = "NONE";

    // PID constants for ball tracking (same as AprilTag tracking)
    private double ballKp = 0.012;
    private double ballKi = 0.001;
    private double ballKd = 0.003;
    private double ballIntegral = 0.0;
    private double ballLastError = 0.0;
    private double ballLastTime = 0.0;
    private final double BALL_MAX_TURN_POWER = 0.28;
    private final double BALL_MIN_TURN_POWER = 0.02;
    private final double BALL_TOLERANCE_DEG = 0.6;
    private final double BALL_SMOOTHING_ALPHA = 0.2;
    private double smoothedBallBearing = 0.0;
    private boolean ballBearingInitialized = false;

    // === Turret PD constants ===
    private static final double kP = 0.001;
    private static final double kD = 0.0006;
    private static final double acceptableTurretError = .25;
    private boolean turretTrackingEnabled = false;
    private boolean lastTurretToggle = false;

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
    private boolean lastY = false;
    private boolean lastDpadRight = false;

    // === Non-blocking sorter movement ===
    private boolean sorterMoving = false;
    private int sorterTargetPosition = 0;
    private ElapsedTime sorterTimer = new ElapsedTime();
    private ElapsedTime sorterSettleTimer = new ElapsedTime();
    private boolean sorterSettling = false;
    private static final int COARSE_TOL = 600;
    private static final int FINE_TOL = 180;
    private static final int PERFECT_TOL = 160;
    private static final double MAX_POWER = 1;
    private static final double MIN_POWER = 0.08;
    private static final long SORTER_TIMEOUT_MS = 2000;
    private static final long SETTLE_MS = 100;

    // === Jam detection ===
    private int lastSorterPosition = 0;
    private long lastSorterMoveTime = 0;
    private static final long JAM_CHECK_INTERVAL_MS = 200;
    private static final int MIN_MOVEMENT_TICKS = 50;
    private boolean sorterJammed = false;

    // === Timed color detection ===
    private long colorStartTime = 0;
    private boolean colorActive = false;
    private static final long DETECT_TIME_MS = 300;

    // === Empty chamber detection ===
    private long emptyStartTime = 0;
    private boolean emptyDetectionActive = false;
    private static final long EMPTY_DETECT_TIME_MS = 500;

    // === Shooter presets ===
    private final int[] rpmPresets = {3200, 3500, 3800};
    private int presetIndex = -1;
    private double targetRPM = 0;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    private static final double TICKS_PER_REV = 28.0;
    private static final double RPM_TOLERANCE = 50.0;

    @Override
    public void init() {
        // === DriveTrain ===
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "fL");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "bL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "bR");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // === Mechanisms ===
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m0 = hardwareMap.get(DcMotorEx.class, "m0");

        s1 = hardwareMap.get(CRServo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");
        s3.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setPosition(1.0);

        for (DcMotor motor : new DcMotor[]{m1, m2, m3, m0}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // === Flywheel motor setup ===
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setVelocityPIDFCoefficients(8.0, 0.5, 0.0, 12.5);

        // === Color Sensors ===
        intakeColor  = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        shooterColor = hardwareMap.get(RevColorSensorV3.class, "shooterColor");

        // === LEDs ===
        LED_Red   = hardwareMap.get(DigitalChannel.class, "LED1");
        LED_Green = hardwareMap.get(DigitalChannel.class, "LED2");
        LED_Blue  = hardwareMap.get(DigitalChannel.class, "LED3");
        LED_Red.setMode(DigitalChannel.Mode.OUTPUT);
        LED_Green.setMode(DigitalChannel.Mode.OUTPUT);
        LED_Blue.setMode(DigitalChannel.Mode.OUTPUT);
        LED_Red.setState(false);
        LED_Green.setState(false);
        LED_Blue.setState(false);

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
        limelight = hardwareMap.get(Limelight3A.class, "Webcam 2");
        limelight.start();

        // === AprilTag Vision setup ===
        turretPD = new PDController(kP, kD);
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        ballLastTime = getTimeSeconds();

        telemetry.addLine("Limelight Ball Tracking Initialized");
        telemetry.addLine("Right Trigger: Intake with IMU-based tracking");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === Check if Limelight Assist should be active ===
        boolean intakeTriggerPressed = gamepad1.right_trigger > 0.1;
        limelightAssistActive = intakeTriggerPressed && !shootingMode;

        // === Drive Train ===
        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
        double rx = applyDeadzone(gamepad1.right_stick_x);

        // === Ball Tracking (IMU-based like AprilTag) ===
        if (limelightAssistActive) {
            double ballCorrection = getBallTrackingCorrection();
            if (ballCorrection != 0) {
                rx = ballCorrection;  // Override rotation
            }
        } else {
            // Reset tracking state
            ballBearingInitialized = false;
            ballIntegral = 0.0;
            ballLastError = 0.0;
            ballLastTime = getTimeSeconds();
        }

        double fl = y + x + rx, bl = y - x + rx, fr = y - x - rx, br = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));
        frontLeftMotor.setPower(clipLowPower(fl / max));
        backLeftMotor.setPower(clipLowPower(bl / max));
        frontRightMotor.setPower(clipLowPower(fr / max));
        backRightMotor.setPower(clipLowPower(br / max));

        // === Sorter ===
        updateSorterMovement();
        int rawPos = m2.getCurrentPosition();
        int normPos = normalize(rawPos);

        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) {
            shootingMode = !shootingMode;
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            startSorterMove(targetPos);
        }
        lastY = yPressed;

        if (shootingMode) {
            boolean dpadRightPressed = gamepad1.dpad_right;
            if (dpadRightPressed && !lastDpadRight) {
                currentChamber = nextChamber(currentChamber);
                int targetPos = getChamberPosition(currentChamber, true);
                startSorterMove(targetPos);
            }
            lastDpadRight = dpadRightPressed;
        }

        // === Manual jam clear ===
        if (gamepad1.dpad_up && sorterJammed) {
            sorterJammed = false;
        }

        if (!shootingMode) {
            autoIntakeColorCheck();
        } else {
            checkChamberEmpty();
        }

        String shooterColorDetected = detectShooterColor();
        updateColorLEDs(shooterColorDetected);

        // === Intake ===
        if (!shootingMode) {
            double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
            m1.setPower(-triggerPower);
            m2.setPower(triggerPower);
        } else {
            m1.setPower(0);
            m2.setPower(0);
        }

        // === Shooter ===
        if (gamepad1.a) {
            s2.setPosition(0);
            s3.setPower(1.0);
        } else {
            s2.setPosition(1);
            s3.setPower(0.0);
        }

        // === Flywheel RPM ===
        if (gamepad1.right_bumper && !lastRightBumper) {
            presetIndex = (presetIndex + 1) % rpmPresets.length;
            targetRPM = rpmPresets[presetIndex];
        } else if (gamepad1.left_bumper && !lastLeftBumper) {
            targetRPM = 0;
        }
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        double targetTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REV;
        m3.setVelocity(targetTicksPerSec);

        updateRPMLED();

        // === Turret Tracking ===
        if (gamepad1.b && !lastTurretToggle) {
            turretTrackingEnabled = !turretTrackingEnabled;
        }
        lastTurretToggle = gamepad1.b;

        if (turretTrackingEnabled && !limelightAssistActive) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                AprilTagDetection det = detections.get(0);
                double tagX = det.center.x;
                double errorX = tagX - (640.0 / 2.0);

                if (Math.abs(errorX) > acceptableTurretError) {
                    turretPD.setSetPoint(0);
                    double turretPow = turretPD.calculate(errorX);
                    turretPow = com.qualcomm.robotcore.util.Range.clip(turretPow, -.75, .75);
                    s1.setPower(turretPow);
                } else {
                    s1.setPower(0);
                }
            } else {
                s1.setPower(0);
            }
        } else {
            s1.setPower(0);
        }

        updateTelemetry(normPos, shooterColorDetected);
    }

    /**
     * IMU-based ball tracking using PID (same approach as AprilTag tracking)
     */
    private double getBallTrackingCorrection() {
        // Check both pipelines for best target
        limelight.pipelineSwitch(0);
        try { Thread.sleep(10); } catch (InterruptedException e) {}
        LLResult greenResult = limelight.getLatestResult();

        limelight.pipelineSwitch(1);
        try { Thread.sleep(10); } catch (InterruptedException e) {}
        LLResult purpleResult = limelight.getLatestResult();

        LLResultTypes.ColorResult bestTarget = null;
        double bestArea = 0;
        double ballBearing = 0;

        // Check green
        if (greenResult != null && greenResult.isValid()) {
            List<LLResultTypes.ColorResult> greenTargets = greenResult.getColorResults();
            if (greenTargets != null && !greenTargets.isEmpty()) {
                LLResultTypes.ColorResult greenTarget = greenTargets.get(0);
                double greenArea = greenTarget.getTargetArea();
                if (greenArea > MIN_TARGET_AREA) {
                    bestTarget = greenTarget;
                    bestArea = greenArea;
                    detectedColor = "GREEN";
                    ballBearing = greenTarget.getTargetXDegrees();
                }
            }
        }

        // Check purple
        if (purpleResult != null && purpleResult.isValid()) {
            List<LLResultTypes.ColorResult> purpleTargets = purpleResult.getColorResults();
            if (purpleTargets != null && !purpleTargets.isEmpty()) {
                LLResultTypes.ColorResult purpleTarget = purpleTargets.get(0);
                double purpleArea = purpleTarget.getTargetArea();
                if (purpleArea > MIN_TARGET_AREA && purpleArea > bestArea) {
                    bestTarget = purpleTarget;
                    bestArea = purpleArea;
                    detectedColor = "PURPLE";
                    ballBearing = purpleTarget.getTargetXDegrees();
                }
            }
        }

        // If we found a ball, use IMU-based PID tracking
        if (bestTarget != null) {
            // Smooth the bearing (like AprilTag code)
            if (!ballBearingInitialized) {
                smoothedBallBearing = ballBearing;
                ballBearingInitialized = true;
            } else {
                smoothedBallBearing = BALL_SMOOTHING_ALPHA * ballBearing +
                        (1.0 - BALL_SMOOTHING_ALPHA) * smoothedBallBearing;
            }

            // Get current IMU heading
            double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double desiredHeading = wrapAngle(imuHeading + smoothedBallBearing);
            double error = wrapAngle(desiredHeading - imuHeading);

            // PID calculation (same as AprilTag code)
            double now = getTimeSeconds();
            double dt = now - ballLastTime;
            if (dt <= 0) dt = 1e-6;

            ballIntegral += error * dt;
            ballIntegral = clamp(ballIntegral, -100.0, 100.0);

            double derivative = (error - ballLastError) / dt;
            double pidOut = ballKp * error + ballKi * ballIntegral + ballKd * derivative;
            pidOut = clamp(pidOut, -BALL_MAX_TURN_POWER, BALL_MAX_TURN_POWER);

            // Stop if within tolerance
            if (Math.abs(error) < BALL_TOLERANCE_DEG) {
                pidOut = 0.0;
                ballIntegral = 0.0;
            } else if (Math.abs(pidOut) < BALL_MIN_TURN_POWER) {
                pidOut = Math.copySign(BALL_MIN_TURN_POWER, pidOut);
            }

            ballLastError = error;
            ballLastTime = now;

            return pidOut;
        } else {
            // No target found
            detectedColor = "NONE";
            ballBearingInitialized = false;
            ballIntegral = 0.0;
            ballLastError = 0.0;
            ballLastTime = getTimeSeconds();
            return 0;
        }
    }

    private void updateRPMLED() {
        if (targetRPM == 0) {
            LED_Red.setState(false);
            return;
        }
        double currentRPM = (m3.getVelocity() / TICKS_PER_REV) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);
        LED_Red.setState(rpmError <= RPM_TOLERANCE);
    }

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

    private void autoIntakeColorCheck() {
        if (allChambersFull() || sorterMoving) return;
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
        int r = intakeColor.red(), g = intakeColor.green(), b = intakeColor.blue();
        if (g > r && g > b && g > 80 && g < 600) return "GREEN";
        if (b > r && b > g && b > 80 && b < 600) return "PURPLE";
        return "NONE";
    }

    private String detectShooterColor() {
        int r = shooterColor.red(), g = shooterColor.green(), b = shooterColor.blue();
        if (g > r && g > b && g > 80 && g < 600) return "GREEN";
        if (b > r && b > g && b > 80 && b < 600) return "PURPLE";
        return "NONE";
    }

    private boolean allChambersFull() { return chamberFull[0] && chamberFull[1] && chamberFull[2]; }
    private int nextChamber(int c) { if (c == 1) return 0; if (c == 0) return 2; return 1; }
    private int normalize(int ticks) { return ((ticks % FULL_ROT) + FULL_ROT) % FULL_ROT; }
    private int calculateShortestError(int current, int target) {
        int error = target - current;
        if (error > FULL_ROT / 2) error -= FULL_ROT;
        else if (error < -FULL_ROT / 2) error += FULL_ROT;
        return error;
    }

    private void updateTelemetry(int normPos, String shooterColorDetected) {
        double currentRPM = (m3.getVelocity() / TICKS_PER_REV) * 60.0;
        boolean rpmReady = (targetRPM > 0) && (Math.abs(targetRPM - currentRPM) <= RPM_TOLERANCE);

        telemetry.addLine("=== Limelight Ball Tracking ===");
        telemetry.addData("Active", limelightAssistActive ? "YES" : "NO");
        telemetry.addData("Ball Detected", detectedColor);
        if (!detectedColor.equals("NONE")) {
            double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Ball Bearing", String.format("%.2f°", smoothedBallBearing));
            telemetry.addData("IMU Heading", String.format("%.2f°", imuHeading));
            telemetry.addData("Error", String.format("%.2f°", ballLastError));
            telemetry.addData("Aligned", Math.abs(ballLastError) < BALL_TOLERANCE_DEG ? "YES ✓" : "NO");
        }
        telemetry.addLine();

        telemetry.addLine("=== Sorter ===");
        telemetry.addData("Pos", normPos);
        telemetry.addData("Chamber", currentChamber + 1);
        telemetry.addData("Jammed", sorterJammed ? "⚠️ Dpad Up" : "NO");
        telemetry.addData("Ch1/2/3", String.format("%s/%s/%s",
                chamberFull[0] ? "●" : "○", chamberFull[1] ? "●" : "○", chamberFull[2] ? "●" : "○"));
        telemetry.addData("Mode", shootingMode ? "SHOOT" : "INTAKE");
        telemetry.addData("Color", shooterColorDetected);
        telemetry.addLine();

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", String.format("%.0f", currentRPM));
        telemetry.addData("Ready", rpmReady ? "YES ✓" : "NO");
        telemetry.addLine();
        telemetry.addData("Turret", turretTrackingEnabled ? "ON" : "OFF");
        telemetry.update();
    }

    private double getTimeSeconds() { return System.nanoTime() / 1e9; }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double wrapAngle(double angle) {
        angle %= 360.0;
        if (angle <= -180.0) angle += 360.0;
        if (angle > 180.0) angle -= 360.0;
        return angle;
    }
    private double applyDeadzone(double v) { return Math.abs(v) < 0.05 ? 0 : v; }
    private double clipLowPower(double p) { return Math.abs(p) < 0.04 ? 0 : p; }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
        if (limelight != null) limelight.stop();
    }
}