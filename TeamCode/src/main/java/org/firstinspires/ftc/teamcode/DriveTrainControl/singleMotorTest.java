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

@TeleOp(name = "Full Teleop", group = "DriveTrainControl")
public class singleMotorTest extends OpMode {

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
    private double kP_base = 0.0012;
    private double kD = 0.008;
    private final double MAX_POWER = 0.55;
    private final double MIN_POWER = 0.06;
    private final double DEADZONE = 12.0;
    private final double DERIV_FILTER = 0.25;

    // Pulse state
    private boolean pulsing = false;
    private double pulseEndTime = 0.0;
    private double pulsePower = 0.0;

    // RPM presets for m3
    private final int[] rpmPresets = {3000, 3500, 3800};
    private int presetIndex = -1;
    private double targetRPM = 0;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    // Encoder telemetry
    private static final double TICKS_PER_REV = 28.0;
    private int lastEncoderPosM2 = 0;
    private long lastTimeNsM2 = 0;
    private double encoderVelocityM2 = 0;

    private int lastEncoderPosM0 = 0;
    private long lastTimeNsM0 = 0;
    private double encoderVelocityM0 = 0;

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
        m0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        lastEncoderPosM2 = m2.getCurrentPosition();
        lastTimeNsM2 = System.nanoTime();

        lastEncoderPosM0 = m0.getCurrentPosition();
        lastTimeNsM0 = System.nanoTime();

        lastTime = getTimeSeconds();
    }

    @Override
    public void loop() {
        // === Drive Train ===
        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
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

        // === m0 manual control ===
        if (gamepad1.dpad_right) m0.setPower(0.5);
        else if (gamepad1.dpad_left) m0.setPower(-0.5);
        else m0.setPower(0.0);

        // === s1 manual control ===
        if (gamepad1.dpad_up) s1.setPower(0.5);
        else if (gamepad1.dpad_down) s1.setPower(-0.5);
        else s1.setPower(0.0);

        // === s2 + s3 shooter control ===
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

        // === m3 RPM control ===
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

        // === Encoder Telemetry for m2 ===
        int encoderPosM2 = m2.getCurrentPosition();
        long nowNs = System.nanoTime();
        double dtEncM2 = (nowNs - lastTimeNsM2) / 1e9;
        if (dtEncM2 > 0.05) {
            encoderVelocityM2 = (encoderPosM2 - lastEncoderPosM2) / dtEncM2;
            lastTimeNsM2 = nowNs;
            lastEncoderPosM2 = encoderPosM2;
        }

        // === Encoder Telemetry for m0 ===
        int encoderPosM0 = m0.getCurrentPosition();
        double dtEncM0 = (nowNs - lastTimeNsM0) / 1e9;
        if (dtEncM0 > 0.05) {
            encoderVelocityM0 = (encoderPosM0 - lastEncoderPosM0) / dtEncM0;
            lastTimeNsM0 = nowNs;
            lastEncoderPosM0 = encoderPosM0;
        }

        telemetry.addLine("=== Motor Telemetry ===");
        telemetry.addData("m0 Pos (ticks)", encoderPosM0);
        telemetry.addData("m0 Vel (ticks/sec)", "%.1f", encoderVelocityM0);
        telemetry.addData("m2 Pos (ticks)", encoderPosM2);
        telemetry.addData("m2 Vel (ticks/sec)", "%.1f", encoderVelocityM2);
        telemetry.addData("Target RPM (m3)", targetRPM);
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
