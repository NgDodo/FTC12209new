package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "DriveTrain + Stable Tracking", group = "DriveTrainControl")
public class everyMotorTest extends OpMode {

    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1, m2;
    DcMotorEx m3, m0;
    CRServo s1, s3;
    Servo s2;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Camera PD control (for pulse computation)
    private boolean cameraTrackingMode = false;
    private boolean lastYPressed = false;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private double derivativeFiltered = 0.0;

    // PD-ish constants (tuned for faster response but damped)
    private double kP_base = 0.0012;     // base proportional
    private double kD = 0.008;         // derivative to damp
    private final double MAX_POWER = 0.55;   // allow fast servo bursts
    private final double MIN_POWER = 0.06;   // minimum power to overcome friction
    private final double DEADZONE = 12.0;    // pixel deadband
    private final double DERIV_FILTER = 0.25;
    private final double TICK_INTERVAL = 0.04; // 25 Hz tick rate

    // Pulse state (non-blocking)
    private boolean pulsing = false;
    private double pulseEndTime = 0.0;
    private double pulsePower = 0.0;

    // RPM presets for m3
    private final int[] rpmPresets = {3000, 3500, 3800};
    private int presetIndex = -1;
    private double targetRPM = 0;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    // Encoder + positional control (s5)
    private static final double TICKS_PER_REV = 28.0;
    private int lastEncoderPos = 0;
    private long lastTimeNs = 0;
    private double encoderVelocity = 0;

    private static final double TARGET_FORWARD_POS = 2733.33;
    private static final double MIN_POS = 0;
    private static final double SERVO_MAX_SPEED = 0.45;
    private static final double SERVO_MIN_SPEED = 0.10;
    private static final double SERVO_TOLERANCE = 20;

    private boolean moveToForward = false;
    private boolean moveToHome = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    // safety: timeouts for s5 movement
    private double s5MoveStartTime = 0.0;
    private final double S5_MOVE_TIMEOUT = 2.0; // seconds max to try reaching

    // If CRServo rotates opposite of encoder sign, flip this to -1
    private final int S5_DIRECTION = -1; // set to -1 or +1 to match hardware. adjust if needed.

    @Override
    public void init() {
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

        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m0 = hardwareMap.get(DcMotorEx.class, "m0");

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        s1 = hardwareMap.get(CRServo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");

        s3.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setPosition(1.0);

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        lastEncoderPos = m2.getCurrentPosition();
        lastTimeNs = System.nanoTime();
        lastTime = getTimeSeconds();
    }

    @Override
    public void loop() {
        // === Drive Train ===
        double y = applyDeadzone(gamepad1.left_stick_y);
        double x = applyDeadzone(-gamepad1.left_stick_x);
        double rx = applyDeadzone(gamepad1.right_stick_x);

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeftMotor.setPower(clipLowPower(fl / max));
        backLeftMotor.setPower(clipLowPower(bl / max));
        frontRightMotor.setPower(clipLowPower(fr / max));
        backRightMotor.setPower(clipLowPower(br / max));

        // Motor m0 controls
        if (gamepad1.dpad_right) {
            m0.setPower(.5);
        } else {
            m0.setPower(0);
        }
        if (gamepad1.dpad_left) {
            m0.setPower(-.5);
        } else {
            m0.setPower(0);
        }
        // Motor m0 controls
        if (gamepad1.dpad_up) {
            s1.setPower(.5);
        } else {
            s1.setPower(0);
        }
        if (gamepad1.dpad_down) {
            s1.setPower(-.5);
        } else {
            s1.setPower(0);
        }
        // Servo s2 and s3 controls
        if (gamepad1.a) {
            s2.setPosition(0);
            s3.setPower(1.0);
        } else {
            s2.setPosition(1);
            s3.setPower(0.0);
        }

        // === m1 + m2 trigger control ===
        double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
        m1.setPower(-triggerPower);
        m2.setPower(triggerPower);

        // === m3 preset RPM ===
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

        // === Toggle Camera Tracking ===
        if (gamepad1.y && !lastYPressed) {
            cameraTrackingMode = !cameraTrackingMode;
        }
        lastYPressed = gamepad1.y;

        // === Non-blocking pulse controller for CRServo camera (s1) ===
        //  - pulses the servo for a short time. pulses are non-blocking because we
        //    persist pulse state across loop() calls.
        //  - this prevents runaway rotations and gives camera time to update.

        // if currently pulsing, keep servo on until pulseEndTime, otherwise turn off
        double nowTime = getTimeSeconds();
        if (pulsing) {
            if (nowTime < pulseEndTime) {
                s1.setPower(pulsePower);
            } else {
                s1.setPower(0.0);
                pulsing = false;
            }
        }

        if (cameraTrackingMode && !pulsing) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                AprilTagDetection det = detections.get(0);
                double tagX = det.center.x;
                double frameCenter = 640.0 / 2.0; // adjust if your webcam resolution differs
                double errorX = tagX - frameCenter;

                double dt = nowTime - lastTime;
                if (dt <= 0) dt = 1e-3;

                // derivative
                double derivative = (errorX - lastError) / dt;
                derivativeFiltered = DERIV_FILTER * derivative + (1 - DERIV_FILTER) * derivativeFiltered;

                // adaptive proportional (stronger for larger errors)
                double kP = kP_base * (0.8 + 0.4 * Math.min(Math.abs(errorX) / 300.0, 1.0));

                double raw = (kP * errorX) + (kD * derivativeFiltered);
                // compute pulse amplitude (clamped)
                double amp = clamp(Math.abs(raw), MIN_POWER, MAX_POWER);
                // direction: we want servo to move toward reducing error
                double direction = Math.signum(-errorX); // negative because signed relationship

                // only act if outside deadzone
                if (Math.abs(errorX) > DEADZONE) {
                    // determine pulse duration: larger errors => longer pulse
                    double pulseDur = clamp(0.02 + Math.abs(errorX) / 2500.0, 0.02, 0.10); // 20–100 ms
                    pulsing = true;
                    pulseEndTime = nowTime + pulseDur;
                    pulsePower = direction * amp;
                    s1.setPower(pulsePower); // start pulse immediately; loop() will turn off later
                } else {
                    // centered — ensure servo off
                    s1.setPower(0.0);
                }

                lastError = errorX;
                lastTime = nowTime;

                telemetry.addData("Camera Tracking", "ON");
                telemetry.addData("Tag X", "%.1f", tagX);
                telemetry.addData("ErrX", "%.1f", errorX);
                telemetry.addData("Pulse amp", "%.3f", pulsePower);
                telemetry.addData("PulseDur (s)", "%.3f", (pulseEndTime - nowTime));
            } else {
                // no tag: ensure servo off
                s1.setPower(0.0);
                telemetry.addData("Camera Tracking", "ON (no tag)");
            }
        } else if (!cameraTrackingMode) {
            s1.setPower(0.0);
            telemetry.addData("Camera Tracking", "OFF");
        }

        // === Encoder Velocity Telemetry ===
        int encoderPos = m2.getCurrentPosition();
        long nowNs = System.nanoTime();
        double dtEnc = (nowNs - lastTimeNs) / 1e9;
        if (dtEnc > 0.05) {
            encoderVelocity = (encoderPos - lastEncoderPos) / dtEnc;
            lastTimeNs = nowNs;
            lastEncoderPos = encoderPos;
        }
        telemetry.addData("Target RPM (m3)", targetRPM);
        telemetry.addData("Encoder Velocity", "%.1f ticks/s", encoderVelocity);
        telemetry.update();
    }

    // === helpers ===
    private double applyDeadzone(double value) {
        return (Math.abs(value) < 0.05) ? 0.0 : value;
    }

    private double clipLowPower(double power) {
        return (Math.abs(power) < 0.04) ? 0.0 : power;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private double getTimeSeconds() {
        return System.nanoTime() / 1e9;
    }
}
