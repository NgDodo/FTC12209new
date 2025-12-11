package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Turret PID Tuning", group = "Test")
public class TurretPIDTuning extends OpMode {

    // === Hardware ===
    private Servo turretServo;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // === Drive Train (for testing while moving) ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // === Turret Configuration ===
    private static final double SERVO_RANGE_DEG = 300.0; // Total rotation range
    private static final double CENTER_POSITION = 0.5; // Servo center position (0-1)
    private static final double MIN_POSITION = 0.0;
    private static final double MAX_POSITION = 1.0;

    // Convert bearing angle to servo position
    // Bearing is -150 to +150 degrees, servo is 0.0 to 1.0
    private static final double BEARING_TO_SERVO_SCALE = 1.0 / SERVO_RANGE_DEG;

    // === PID Coefficients - TUNE THESE ===
    private double kP = 0.11;
    private double kI = 0.0;
    private double kD = 0.003;

    // === PID variables ===
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;

    // === Tracking Control ===
    private boolean trackingEnabled = false;
    private boolean lastAButton = false;
    private double manualServoPosition = CENTER_POSITION;

    // === Smoothing ===
    private static final double SMOOTHING_ALPHA = 0.3; // Lower = smoother, higher = more responsive
    private double smoothedBearing = 0.0;
    private boolean bearingInitialized = false;

    // === Adjustment increments ===
    private static final double LARGE_INCREMENT = 0.002;
    private static final double SMALL_INCREMENT = 0.0002;
    private static final double MANUAL_SERVO_INCREMENT = 0.01;

    // === Button states ===
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastX = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    // === Current parameter being tuned ===
    private enum TuneParam { KP, KI, KD }
    private TuneParam currentParam = TuneParam.KP;

    @Override
    public void init() {
        // === Turret Servo ===
        turretServo = hardwareMap.get(Servo.class, "turret"); // Change name if needed
        turretServo.setPosition(CENTER_POSITION);

        // === AprilTag Vision setup ===
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // === DriveTrain (optional for testing) ===
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

        lastTime = System.nanoTime();

        telemetry.addLine("Turret PID Tuning");
        telemetry.addLine("===================");
        telemetry.addLine("A: Toggle AprilTag Tracking");
        telemetry.addLine("X/B/Y: Select kP/kI/kD");
        telemetry.addLine("DpadUp/Down: Large adjust");
        telemetry.addLine("DpadLeft/Right: Small adjust");
        telemetry.addLine("LB/RB: Manual turret control");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === Toggle Tracking (A Button) ===
        if (gamepad1.a && !lastAButton) {
            trackingEnabled = !trackingEnabled;
            if (trackingEnabled) {
                bearingInitialized = false;
                integral = 0;
                lastError = 0;
            }
        }
        lastAButton = gamepad1.a;

        // === Parameter selection ===
        if (gamepad1.x && !lastX) currentParam = TuneParam.KP;
        if (gamepad1.b && !lastB) currentParam = TuneParam.KI;
        if (gamepad1.y && !lastY) currentParam = TuneParam.KD;

        lastX = gamepad1.x;
        lastB = gamepad1.b;
        lastY = gamepad1.y;

        // === Adjust current parameter ===
        if (gamepad1.dpad_up && !lastDpadUp) {
            adjustParameter(LARGE_INCREMENT);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            adjustParameter(-LARGE_INCREMENT);
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            adjustParameter(SMALL_INCREMENT);
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            adjustParameter(-SMALL_INCREMENT);
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        // === Manual servo control (when tracking disabled) ===
        if (!trackingEnabled) {
            if (gamepad1.right_bumper && !lastRightBumper) {
                manualServoPosition += MANUAL_SERVO_INCREMENT;
                manualServoPosition = Math.min(MAX_POSITION, manualServoPosition);
            }
            if (gamepad1.left_bumper && !lastLeftBumper) {
                manualServoPosition -= MANUAL_SERVO_INCREMENT;
                manualServoPosition = Math.max(MIN_POSITION, manualServoPosition);
            }
            turretServo.setPosition(manualServoPosition);
        }
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        // === AprilTag Tracking PID Control ===
        if (trackingEnabled) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty() && detections.get(0).ftcPose != null) {
                AprilTagDetection det = detections.get(0);
                double bearing = det.ftcPose.bearing; // Angle to target in degrees

                // Initialize or smooth the bearing
                if (!bearingInitialized) {
                    smoothedBearing = bearing;
                    bearingInitialized = true;
                } else {
                    smoothedBearing = SMOOTHING_ALPHA * bearing +
                            (1.0 - SMOOTHING_ALPHA) * smoothedBearing;
                }

                // PID calculation
                long currentTime = System.nanoTime();
                double dt = (currentTime - lastTime) / 1e9; // Convert to seconds
                if (dt <= 0) dt = 1e-6;

                double error = smoothedBearing; // Error is the bearing itself (want it to be 0)

                integral += error * dt;
                // Anti-windup
                integral = Math.max(-100.0, Math.min(100.0, integral));

                double derivative = (error - lastError) / dt;

                // PID output (in degrees)
                double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);

                // Convert PID output (degrees) to servo position change
                double servoChange = pidOutput * BEARING_TO_SERVO_SCALE;

                // Get current servo position and apply correction
                double currentServoPos = turretServo.getPosition();
                double newServoPos = currentServoPos + servoChange;

                // Clamp to servo limits
                newServoPos = Math.max(MIN_POSITION, Math.min(MAX_POSITION, newServoPos));

                turretServo.setPosition(newServoPos);
                manualServoPosition = newServoPos; // Update manual position for when tracking disabled

                lastError = error;
                lastTime = currentTime;

                // Telemetry for tracking
                telemetry.addLine("=== TRACKING ACTIVE ===");
                telemetry.addData("Tag ID", det.id);
                telemetry.addData("Raw Bearing", String.format("%.2f°", bearing));
                telemetry.addData("Smoothed Bearing", String.format("%.2f°", smoothedBearing));
                telemetry.addData("Error", String.format("%.2f°", error));
                telemetry.addData("Servo Position", String.format("%.3f", newServoPos));
                telemetry.addData("Servo Change", String.format("%.4f", servoChange));
            } else {
                // No tag visible - hold position
                telemetry.addLine("=== NO TAG VISIBLE ===");
                telemetry.addData("Status", "Holding Position");
            }
        } else {
            telemetry.addLine("=== MANUAL MODE ===");
            telemetry.addData("Servo Position", String.format("%.3f", manualServoPosition));
            telemetry.addLine("Use LB/RB to adjust turret");
        }

        // === Drive Train (for testing while moving) ===
        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
        double rx = applyDeadzone(gamepad1.right_stick_x);

        double fl = y + x + rx, bl = y - x + rx, fr = y - x - rx, br = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));
        frontLeftMotor.setPower(fl / max);
        backLeftMotor.setPower(bl / max);
        frontRightMotor.setPower(fr / max);
        backRightMotor.setPower(br / max);

        // === PID Parameters Display ===
        telemetry.addLine();
        telemetry.addLine("=== PID PARAMETERS ===");
        telemetry.addData("kP (X)", String.format("%s%.6f", currentParam == TuneParam.KP ? ">>> " : "    ", kP));
        telemetry.addData("kI (B)", String.format("%s%.6f", currentParam == TuneParam.KI ? ">>> " : "    ", kI));
        telemetry.addData("kD (Y)", String.format("%s%.6f", currentParam == TuneParam.KD ? ">>> " : "    ", kD));
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("Tracking", trackingEnabled ? "ON (A to disable)" : "OFF (A to enable)");
        telemetry.addLine("X/B/Y: Select kP/kI/kD");
        telemetry.addLine("DpadUp/Down: Large adjust (±0.0001)");
        telemetry.addLine("DpadLeft/Right: Small adjust (±0.00001)");
        if (!trackingEnabled) {
            telemetry.addLine("LB/RB: Manual turret control");
        }

        telemetry.update();
    }

    private void adjustParameter(double amount) {
        switch (currentParam) {
            case KP:
                kP += amount;
                kP = Math.max(0, kP);
                break;
            case KI:
                kI += amount;
                kI = Math.max(0, kI);
                break;
            case KD:
                kD += amount;
                kD = Math.max(0, kD);
                break;
        }
    }

    private double applyDeadzone(double v) {
        return Math.abs(v) < 0.05 ? 0 : v;
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
        turretServo.setPosition(CENTER_POSITION);
    }
}