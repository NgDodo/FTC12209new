package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.arcrobotics.ftclib.controller.PDController;
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

    // === Limelight ===
    private Limelight3A limelight;
    private PDController limelightPD;
    private static final double LIMELIGHT_kP = 0.03;
    private static final double LIMELIGHT_kD = 0.005;
    private static final double LIMELIGHT_TOLERANCE = 2.0;  // degrees
    private boolean limelightAssistActive = false;
    private String detectedColor = "NONE";  // Tracks which color ball is being targeted

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

    // ABSOLUTE chamber positions (intake aligned)
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
    private final int[] rpmPresets = {2800,3100, 3400};
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

        // === Flywheel motor setup with custom PIDF ===
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

        // === Limelight Setup ===
        limelight = hardwareMap.get(Limelight3A.class, "Webcam 2");
        limelight.start();

        limelightPD = new PDController(LIMELIGHT_kP, LIMELIGHT_kD);
        limelightPD.setSetPoint(0);  // Target is center (0 degrees)

        // === AprilTag Vision setup ===
        turretPD = new PDController(kP, kD);
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Limelight Assisted Intake Initialized");
        telemetry.addLine("Right Trigger: Intake with auto-centering");
        telemetry.addLine("Targets closest ball (Green or Purple)");
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

        // If limelight assist is active, override rotation with limelight correction
        if (limelightAssistActive) {
            double limelightCorrection = getLimelightCorrection();
            rx = limelightCorrection;  // Override user rotation input
        }

        double fl = y + x + rx, bl = y - x + rx, fr = y - x - rx, br = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));
        frontLeftMotor.setPower(clipLowPower(fl / max));
        backLeftMotor.setPower(clipLowPower(bl / max));
        frontRightMotor.setPower(clipLowPower(fr / max));
        backRightMotor.setPower(clipLowPower(br / max));

        // === Non-blocking sorter movement ===
        updateSorterMovement();

        int rawPos = m2.getCurrentPosition();
        int normPos = normalize(rawPos);

        // ===== Shooting Mode Toggle =====
        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) {
            shootingMode = !shootingMode;
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            startSorterMove(targetPos);
        }
        lastY = yPressed;

        // ===== Manual chamber advance in shooting mode =====
        if (shootingMode) {
            boolean dpadRightPressed = gamepad1.dpad_right;
            if (dpadRightPressed && !lastDpadRight) {
                currentChamber = nextChamber(currentChamber);
                int targetPos = getChamberPosition(currentChamber, true);
                startSorterMove(targetPos);
            }
            lastDpadRight = dpadRightPressed;
        }

        // ===== Auto-rotate only in intake mode =====
        if (!shootingMode) {
            autoIntakeColorCheck();
        } else {
            checkChamberEmpty();
        }

        // ===== Detect shooter color and update LEDs =====
        String shooterColorDetected = detectShooterColor();
        updateColorLEDs(shooterColorDetected);

        // === Intake motors (always responsive) ===
        if (!shootingMode) {
            double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
            m1.setPower(-triggerPower);
            m2.setPower(triggerPower);
        } else {
            m1.setPower(0);
            m2.setPower(0);
        }

        // === Shooter (s2, s3) ===
        if (gamepad1.a) {
            s2.setPosition(0);
            s3.setPower(1.0);
        } else {
            s2.setPosition(1);
            s3.setPower(0.0);
        }

        // === Shooter RPM with velocity control ===
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

        // === Update RPM LED ===
        updateRPMLED();

        // === Turret Tracking Toggle ===
        if (gamepad1.b && !lastTurretToggle) {
            turretTrackingEnabled = !turretTrackingEnabled;
        }
        lastTurretToggle = gamepad1.b;

        // === Turret Tracking (only when limelight assist is not active) ===
        if (turretTrackingEnabled && !limelightAssistActive) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                AprilTagDetection det = detections.get(0);
                double tagX = det.center.x;
                double errorX = tagX - (640.0 / 2.0);

                if (Math.abs(errorX) > acceptableTurretError) {
                    turretPD.setSetPoint(0);
                    double turretPow = turretPD.calculate(errorX);
                    turretPow = Range.clip(turretPow, -.75, .75);
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
     * Get Limelight correction for auto-centering on ball
     * Checks both pipelines and targets the closest/best ball
     * Returns a rotation power value to center the robot on the detected object
     */
    private double getLimelightCorrection() {
        // Check pipeline 0 (GREEN)
        limelight.pipelineSwitch(0);
        LLResult greenResult = limelight.getLatestResult();

        // Check pipeline 1 (PURPLE)
        limelight.pipelineSwitch(1);
        LLResult purpleResult = limelight.getLatestResult();

        // Variables to store best target
        LLResultTypes.ColorResult bestTarget = null;
        double bestArea = 0;
        String bestColor = "NONE";

        // Check green results
        if (greenResult != null && greenResult.isValid()) {
            List<LLResultTypes.ColorResult> greenTargets = greenResult.getColorResults();
            if (greenTargets != null && !greenTargets.isEmpty()) {
                LLResultTypes.ColorResult greenTarget = greenTargets.get(0);
                double greenArea = greenTarget.getTargetArea();
                if (greenArea > bestArea) {
                    bestTarget = greenTarget;
                    bestArea = greenArea;
                    bestColor = "GREEN";
                }
            }
        }

        // Check purple results
        if (purpleResult != null && purpleResult.isValid()) {
            List<LLResultTypes.ColorResult> purpleTargets = purpleResult.getColorResults();
            if (purpleTargets != null && !purpleTargets.isEmpty()) {
                LLResultTypes.ColorResult purpleTarget = purpleTargets.get(0);
                double purpleArea = purpleTarget.getTargetArea();
                if (purpleArea > bestArea) {
                    bestTarget = purpleTarget;
                    bestArea = purpleArea;
                    bestColor = "PURPLE";
                }
            }
        }

        // Update detected color for telemetry
        detectedColor = bestColor;

        // If we found a target, calculate correction
        if (bestTarget != null) {
            double tx = bestTarget.getTargetXDegrees();

            // If we're close to center, don't move
            if (Math.abs(tx) < LIMELIGHT_TOLERANCE) {
                return 0;
            }

            // Calculate PD correction
            double correction = limelightPD.calculate(tx);

            // Clip to safe range
            return Range.clip(correction, -0.6, 0.6);
        }

        // No target found, don't rotate
        detectedColor = "NONE";
        return 0;
    }

    private void updateRPMLED() {
        if (targetRPM == 0) {
            LED_Red.setState(false);
            return;
        }

        double currentRPM = (m3.getVelocity() / TICKS_PER_REV) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);

        if (rpmError <= RPM_TOLERANCE) {
            LED_Red.setState(true);
        } else {
            LED_Red.setState(false);
        }
    }

    private void updateSorterMovement() {
        if (!sorterMoving) {
            return;
        }

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

        // Limelight info
        double llTx = 0;
        boolean llCentered = false;

        telemetry.addLine("=== Limelight Assist ===");
        telemetry.addData("Active", limelightAssistActive ? "YES" : "NO");
        telemetry.addData("Target Detected", detectedColor);

        if (!detectedColor.equals("NONE")) {
            // Get the tx value from whichever pipeline detected the target
            if (detectedColor.equals("GREEN")) {
                limelight.pipelineSwitch(0);
            } else {
                limelight.pipelineSwitch(1);
            }
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && result.getColorResults() != null && !result.getColorResults().isEmpty()) {
                llTx = result.getColorResults().get(0).getTargetXDegrees();
                llCentered = Math.abs(llTx) < LIMELIGHT_TOLERANCE;
            }
            telemetry.addData("Horizontal Offset (tx)", String.format("%.2f°", llTx));
            telemetry.addData("Centered", llCentered ? "YES ✓" : "NO");
        }
        telemetry.addLine();
        telemetry.addLine("=== Sorter ===");
        telemetry.addData("Normalized Pos", normPos);
        telemetry.addData("Current Chamber", currentChamber + 1);
        telemetry.addData("Sorter Moving", sorterMoving);
        telemetry.addLine();
        telemetry.addData("Ch1 Full", chamberFull[0]);
        telemetry.addData("Ch2 Full", chamberFull[1]);
        telemetry.addData("Ch3 Full", chamberFull[2]);
        telemetry.addLine();
        telemetry.addData("Shooting Mode", shootingMode);
        telemetry.addData("Color to Shoot", shooterColorDetected);
        telemetry.addLine();
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", String.format("%.0f", currentRPM));
        telemetry.addData("RPM Ready", rpmReady ? "YES ✓" : "NO");
        telemetry.addLine();
        telemetry.addData("Turret Tracking", turretTrackingEnabled ? "ON" : "OFF");
        telemetry.update();
    }

    private double applyDeadzone(double v) { return Math.abs(v) < 0.05 ? 0 : v; }
    private double clipLowPower(double p) { return Math.abs(p) < 0.04 ? 0 : p; }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
        if (limelight != null) limelight.stop();
    }
}