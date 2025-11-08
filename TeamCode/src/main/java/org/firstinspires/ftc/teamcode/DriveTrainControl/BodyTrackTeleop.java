package org.firstinspires.ftc.teamcode.DriveTrainControl;

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

import java.util.List;

@TeleOp(name = "NonBlockTeleop with Body Tracking", group = "DriveTrainControl")
public class BodyTrackTeleop extends OpMode {

    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1, m2;
    DcMotorEx m3, m0;
    CRServo s3;
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
    private IMU imu;

    // === AprilTag Body Alignment PID ===
    private double kp = 0.012;
    private double ki = 0.001;
    private double kd = 0.003;
    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private final double MAX_TURN_POWER = 0.28;
    private final double MIN_TURN_POWER = 0.02;
    private final double TOLERANCE_DEG = 0.6;
    private final double SMOOTHING_ALPHA = 0.2;
    private double smoothedBearing = 0.0;
    private boolean bearingInitialized = false;
    private boolean aprilTagTrackingEnabled = false;
    private boolean lastBButton = false;

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
    private boolean lastDpadLeft = false;

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
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(params);
        imu.resetYaw();

        // === AprilTag Vision setup ===
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        lastTime = getTimeSeconds();

        telemetry.addLine("Initialized — Press START");
        telemetry.addLine("B Button: Toggle AprilTag Body Tracking");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === AprilTag Body Tracking Toggle ===
        boolean bPressed = gamepad1.b;
        if (bPressed && !lastBButton) {
            aprilTagTrackingEnabled = !aprilTagTrackingEnabled;
            if (!aprilTagTrackingEnabled) {
                integral = 0.0;
                lastError = 0.0;
                bearingInitialized = false;
            }
        }
        lastBButton = bPressed;

        // === Drive Train ===
        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
        double rx = applyDeadzone(gamepad1.right_stick_x);

        // === AprilTag Body Alignment ===
        if (aprilTagTrackingEnabled) {
            double bodyCorrection = getAprilTagBodyCorrection();
            if (bodyCorrection != 0) {
               rx = -bodyCorrection;
            }
        } else {
            bearingInitialized = false;
            integral = 0.0;
            lastError = 0.0;
            lastTime = getTimeSeconds();
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
        } else if (gamepad1.dpad_left && !lastDpadLeft) {
            targetRPM = -2000;
        }
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;
        lastDpadLeft = gamepad1.dpad_left;

        double targetTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REV;
        m3.setVelocity(targetTicksPerSec);

        updateRPMLED();
        updateTelemetry(normPos, shooterColorDetected);
    }

    private double getAprilTagBodyCorrection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (!detections.isEmpty() && detections.get(0).ftcPose != null) {
            AprilTagDetection det = detections.get(0);
            double bearing = det.ftcPose.bearing;

            if (!bearingInitialized) {
                smoothedBearing = bearing;
                bearingInitialized = true;
            } else {
                smoothedBearing = SMOOTHING_ALPHA * bearing + (1.0 - SMOOTHING_ALPHA) * smoothedBearing;
            }

            double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double desiredHeading = wrapAngle(imuHeading + smoothedBearing);
            double error = wrapAngle(desiredHeading - imuHeading);

            double now = getTimeSeconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-6;

            integral += error * dt;
            integral = clamp(integral, -100.0, 100.0);

            double derivative = (error - lastError) / dt;
            double pidOut = kp * error + ki * integral + kd * derivative;
            pidOut = clamp(pidOut, -MAX_TURN_POWER, MAX_TURN_POWER);

            if (Math.abs(error) < TOLERANCE_DEG) {
                pidOut = 0.0;
                integral = 0.0;
            } else if (Math.abs(pidOut) < MIN_TURN_POWER) {
                pidOut = Math.copySign(MIN_TURN_POWER, pidOut);
            }

            lastError = error;
            lastTime = now;
            return pidOut;
        } else {
            bearingInitialized = false;
            integral = 0.0;
            lastError = 0.0;
            lastTime = getTimeSeconds();
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
        if (shooting) {
            basePos = normalize(basePos + OFFSET);
        }
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

    private boolean allChambersFull() {
        return chamberFull[0] && chamberFull[1] && chamberFull[2];
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

    private void updateTelemetry(int normPos, String shooterColorDetected) {
        double currentRPM = (m3.getVelocity() / TICKS_PER_REV) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);
        boolean rpmReady = (targetRPM > 0) && (rpmError <= RPM_TOLERANCE);

        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean hasTarget = !detections.isEmpty() && detections.get(0).ftcPose != null;

        telemetry.addLine("=== AprilTag Body Tracking ===");
        telemetry.addData("Active", aprilTagTrackingEnabled ? "YES" : "NO (B to toggle)");
        if (aprilTagTrackingEnabled && hasTarget) {
            AprilTagDetection det = detections.get(0);
            double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Tag ID", det.id);
            telemetry.addData("Bearing", String.format("%.2f°", smoothedBearing));
            telemetry.addData("IMU Heading", String.format("%.2f°", imuHeading));
            telemetry.addData("Error", String.format("%.2f°", lastError));
            telemetry.addData("Aligned", Math.abs(lastError) < TOLERANCE_DEG ? "YES ✓" : "NO");
        }
        telemetry.addLine();

        telemetry.addLine("=== Sorter ===");
        telemetry.addData("Pos", normPos);
        telemetry.addData("Chamber", currentChamber + 1);
        telemetry.addData("Moving", sorterMoving);
        telemetry.addData("Ch1/2/3", String.format("%s/%s/%s",
                chamberFull[0] ? "●" : "○",
                chamberFull[1] ? "●" : "○",
                chamberFull[2] ? "●" : "○"));
        telemetry.addData("Mode", shootingMode ? "SHOOT" : "INTAKE");
        telemetry.addData("Color", shooterColorDetected);
        telemetry.addLine();

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", String.format("%.0f", currentRPM));
        telemetry.addData("Ready", rpmReady ? "YES ✓" : "NO");
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
    }
}