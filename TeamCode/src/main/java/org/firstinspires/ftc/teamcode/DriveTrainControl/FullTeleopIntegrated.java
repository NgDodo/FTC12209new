package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.arcrobotics.ftclib.controller.PDController;
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

@TeleOp(name = "Full Teleop (Integrated Sorter)", group = "DriveTrainControl")
public class FullTeleopIntegrated extends OpMode {

    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1, m2;
    DcMotorEx m3, m0;
    CRServo s1, s3;
    Servo s2;

    // === Color Sensors & LEDs ===
    private RevColorSensorV3 intakeColor;
    private RevColorSensorV3 shooterColor;
    private DigitalChannel LED_Green;
    private DigitalChannel LED_Blue;

    // === Vision & AprilTag ===
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private PDController turretPD;

    // === Turret PD constants ===
    private static final double kP = 0.001;
    private static final double kD = 0.0006;
    private static final double acceptableTurretError = .25;
    private boolean turretTrackingEnabled = false;
    private boolean lastTurretToggle = false;

    // === Sorter constants ===
    private static final int FULL_ROT = 8200;
    private static final int SLOT = FULL_ROT / 3;
    private static final int OFFSET = (int) (FULL_ROT / 6.3);
    private static final double SORTER_POWER = 1.0;

    // ABSOLUTE chamber positions (intake aligned)
    private static final int CHAMBER_0_POS = 0;
    private static final int CHAMBER_1_POS = SLOT;
    private static final int CHAMBER_2_POS = 2 * SLOT;

    private boolean[] chamberFull = new boolean[3];
    private int currentChamber = 0;

    private boolean shootingMode = false;
    private boolean lastY = false;
    private boolean lastDpadRight = false;

    // === Timed color detection ===
    private long colorStartTime = 0;
    private boolean colorActive = false;
    private static final long DETECT_TIME_MS = 300;

    // === Empty chamber detection ===
    private long emptyStartTime = 0;
    private boolean emptyDetectionActive = false;
    private static final long EMPTY_DETECT_TIME_MS = 500; // 0.5 sec

    // === Shooter presets ===
    private final int[] rpmPresets = {3200, 3500, 3800};
    private int presetIndex = -1;
    private double targetRPM = 0;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    private static final double TICKS_PER_REV = 28.0;

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
        m2 = hardwareMap.get(DcMotor.class, "m2");  // sorter external encoder
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m0 = hardwareMap.get(DcMotorEx.class, "m0"); // sorter motor

        s1 = hardwareMap.get(CRServo.class, "s1"); // turret servo
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");
        s3.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setPosition(1.0);

        for (DcMotor motor : new DcMotor[]{m1, m2, m3, m0}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // === Color Sensors ===
        intakeColor  = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        shooterColor = hardwareMap.get(RevColorSensorV3.class, "shooterColor");

        // === LEDs ===
        LED_Green = hardwareMap.get(DigitalChannel.class, "LED2");
        LED_Blue  = hardwareMap.get(DigitalChannel.class, "LED3");
        LED_Green.setMode(DigitalChannel.Mode.OUTPUT);
        LED_Blue.setMode(DigitalChannel.Mode.OUTPUT);
        LED_Green.setState(false); // Start OFF
        LED_Blue.setState(false);

        // === Vision setup ===
        turretPD = new PDController(kP, kD);
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Initialized — Press START to begin");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === Drive Train ===
        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
        double rx = applyDeadzone(gamepad1.right_stick_x);

        double fl = y + x + rx, bl = y - x + rx, fr = y - x - rx, br = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));
        frontLeftMotor.setPower(clipLowPower(fl / max));
        backLeftMotor.setPower(clipLowPower(bl / max));
        frontRightMotor.setPower(clipLowPower(fr / max));
        backRightMotor.setPower(clipLowPower(br / max));

        int rawPos = m2.getCurrentPosition();
        int normPos = normalize(rawPos);

        // ===== Shooting Mode Toggle =====
        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) {
            shootingMode = !shootingMode;

            // Move to absolute position based on current chamber and mode
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            moveToExternal(targetPos);
        }
        lastY = yPressed;

        // ===== Manual chamber advance in shooting mode =====
        if (shootingMode) {
            boolean dpadRightPressed = gamepad1.dpad_right;
            if (dpadRightPressed && !lastDpadRight) {
                // Advance to next chamber manually
                currentChamber = nextChamber(currentChamber);
                int targetPos = getChamberPosition(currentChamber, true);
                moveToExternal(targetPos);
            }
            lastDpadRight = dpadRightPressed;
        }

        // ===== Auto-rotate only in intake mode =====
        if (!shootingMode) {
            autoIntakeColorCheck();
        } else {
            // Check if current chamber is empty in shooting mode
            checkChamberEmpty();
        }

        // ===== Detect shooter color and update LEDs =====
        String shooterColorDetected = detectShooterColor();
        updateLEDs(shooterColorDetected);

        // === Intake motors (m1/m2) - only run intake in intake mode ===
        if (!shootingMode) {
            double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
            m1.setPower(-triggerPower);
            m2.setPower(triggerPower);  // m2 is also an intake motor (encoder used for sorter)
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

        // === Shooter RPM (m3) ===
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

        // === Turret Tracking Toggle ===
        if (gamepad1.b && !lastTurretToggle) {
            turretTrackingEnabled = !turretTrackingEnabled;
        }
        lastTurretToggle = gamepad1.b;

        // === Turret Tracking ===
        if (turretTrackingEnabled) {
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
     * Update LEDs based on shooter color
     * GREEN ball -> Green LED on, Blue LED off
     * PURPLE ball -> Blue LED on, Green LED off
     * NONE -> Both LEDs off
     */
    private void updateLEDs(String color) {
        if (color.equals("GREEN")) {
            LED_Green.setState(true);   // Green ON
            LED_Blue.setState(false);   // Blue OFF
        } else if (color.equals("PURPLE")) {
            LED_Green.setState(false);  // Green OFF
            LED_Blue.setState(true);    // Blue ON
        } else {
            LED_Green.setState(false);  // Green OFF
            LED_Blue.setState(false);   // Blue OFF
        }
    }

    /**
     * Get the absolute position for a chamber in intake or shooting mode
     * This prevents drift by always using fixed positions
     */
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

    /** Auto chamber switching with continuous color detection - INTAKE MODE ONLY */
    private void autoIntakeColorCheck() {
        if (allChambersFull()) return;

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
                moveToExternal(target);
            }

            colorActive = false;
            colorStartTime = 0;
        }
    }

    /** Check if current chamber is empty by detecting if shooter color sensor sees nothing */
    private void checkChamberEmpty() {
        // Only check in shooting mode when chamber is aligned with shooter
        if (!shootingMode || !chamberFull[currentChamber]) {
            emptyDetectionActive = false;
            emptyStartTime = 0;
            return;
        }

        String detected = detectShooterColor();

        // If we detect a color, reset the empty detection timer
        if (!detected.equals("NONE")) {
            emptyDetectionActive = false;
            emptyStartTime = 0;
            return;
        }

        // Start timing when we first detect no ball
        if (!emptyDetectionActive) {
            emptyDetectionActive = true;
            emptyStartTime = System.currentTimeMillis();
        }

        // If no ball detected continuously for EMPTY_DETECT_TIME_MS, mark chamber as empty
        if (System.currentTimeMillis() - emptyStartTime >= EMPTY_DETECT_TIME_MS) {
            chamberFull[currentChamber] = false;

            // Reset detection
            emptyDetectionActive = false;
            emptyStartTime = 0;
        }
    }

    /** Detect intake color - GREEN / PURPLE / NONE */
    private String detectIntakeColor() {
        int r = intakeColor.red();
        int g = intakeColor.green();
        int b = intakeColor.blue();

        if (g > r && g > b && g > 80 && g < 600) return "GREEN";
        if (b > r && b > g && b > 80 && b < 600) return "PURPLE";
        return "NONE";
    }

    /** Detect shooter color - GREEN / PURPLE / NONE */
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

    /**
     * IMPROVED movement function with better accuracy
     */
    private void moveToExternal(int targetTicks) {
        final int COARSE_TOL = 400;
        final int FINE_TOL = 145;
        final int PERFECT_TOL = 100;
        final double MAX_POWER = 1;
        final double MIN_POWER = 0.08;
        final long TIMEOUT_MS = 2000;
        final long SETTLE_MS = 100;

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        boolean settling = false;

        while (!isStopRequested() && timer.milliseconds() < TIMEOUT_MS) {
            int pos = normalize(m2.getCurrentPosition());
            int error = calculateShortestError(pos, targetTicks);

            if (Math.abs(error) <= PERFECT_TOL) {
                if (!settling) {
                    settling = true;
                    settleTimer.reset();
                    m0.setPower(0);
                }

                if (settleTimer.milliseconds() >= SETTLE_MS) {
                    break;
                }

                if (Math.abs(error) > FINE_TOL) {
                    settling = false;
                } else {
                    continue;
                }
            } else {
                settling = false;
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

            try { Thread.sleep(5); } catch (InterruptedException e) {}
        }

        m0.setPower(0);
        try { Thread.sleep(50); } catch (InterruptedException e) {}

        int finalError = calculateShortestError(
                normalize(m2.getCurrentPosition()),
                targetTicks
        );

        if (Math.abs(finalError) > FINE_TOL && Math.abs(finalError) < COARSE_TOL) {
            double correctionPower = Math.signum(finalError) * MIN_POWER;
            m0.setPower(correctionPower);

            ElapsedTime correctionTimer = new ElapsedTime();
            while (!isStopRequested() && correctionTimer.milliseconds() < 200) {
                int pos = normalize(m2.getCurrentPosition());
                int err = calculateShortestError(pos, targetTicks);

                if (Math.abs(err) <= PERFECT_TOL) {
                    break;
                }

                try { Thread.sleep(5); } catch (InterruptedException e) {}
            }

            m0.setPower(0);
        }

        try { Thread.sleep(30); } catch (InterruptedException e) {}
    }

    private boolean isStopRequested() {
        return false;
    }

    /**
     * Calculate the shortest rotational error (handles wraparound)
     */
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
        telemetry.addLine("=== Sorter ===");
        telemetry.addData("Normalized Pos", normPos);
        telemetry.addData("Current Chamber", currentChamber + 1);
        telemetry.addData("Target Position", getChamberPosition(currentChamber, shootingMode));
        telemetry.addData("Position Error", calculateShortestError(normPos, getChamberPosition(currentChamber, shootingMode)));
        telemetry.addLine();
        telemetry.addData("Ch1 Full", chamberFull[0]);
        telemetry.addData("Ch2 Full", chamberFull[1]);
        telemetry.addData("Ch3 Full", chamberFull[2]);
        telemetry.addLine();
        telemetry.addData("Shooting Mode", shootingMode);
        telemetry.addData("Color to Shoot", shooterColorDetected);
        telemetry.addData("Intake Color Active", colorActive);
        telemetry.addData("Empty Detection Active", emptyDetectionActive);
        telemetry.addLine();
        telemetry.addData("Turret Tracking", turretTrackingEnabled ? "ON" : "OFF");
        telemetry.addData("Target RPM (m3)", targetRPM);
        telemetry.addData("Actual RPM (m3)", ((m3.getVelocity()/TICKS_PER_REV)*60));
        telemetry.update();
    }

    private double applyDeadzone(double v) { return Math.abs(v) < 0.05 ? 0 : v; }
    private double clipLowPower(double p) { return Math.abs(p) < 0.04 ? 0 : p; }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }
}