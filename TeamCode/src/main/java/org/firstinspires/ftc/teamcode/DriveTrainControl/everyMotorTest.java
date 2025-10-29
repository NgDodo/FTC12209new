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

@TeleOp(name = "DriveTrain + m0 Position Move (700 ticks)", group = "DriveTrainControl")
public class everyMotorTest extends OpMode {

    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1, m2;
    DcMotorEx m3, m0;
    CRServo s1, s3;
    Servo s2;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // m0 movement control
    private static final int TICKS_PER_MOVE = 700;
    private static final int POSITION_TOLERANCE = 10;
    private static final double M0_MAX_POWER = 0.5;
    private static final double M0_KP = 0.002;

    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean movingToTarget = false;
    private int targetPositionM0 = 0;

    // Telemetry tracking
    private int lastEncoderPosM0 = 0;
    private long lastTimeNsM0 = 0;
    private double encoderVelocityM0 = 0;

    private int lastEncoderPosM2 = 0;
    private long lastTimeNsM2 = 0;
    private double encoderVelocityM2 = 0;

    private boolean cameraTrackingMode = false;
    private boolean lastYPressed = false;

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

        m0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        lastEncoderPosM0 = m0.getCurrentPosition();
        lastEncoderPosM2 = m2.getCurrentPosition();
        lastTimeNsM0 = System.nanoTime();
        lastTimeNsM2 = System.nanoTime();
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

        // === Replace Dpad Controls: Move m0 exactly 700 ticks ===
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        if (dpadRight && !lastDpadRight) {
            targetPositionM0 = m0.getCurrentPosition() + TICKS_PER_MOVE;
            movingToTarget = true;
        }
        if (dpadLeft && !lastDpadLeft) {
            targetPositionM0 = m0.getCurrentPosition() - TICKS_PER_MOVE;
            movingToTarget = true;
        }

        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;

        if (movingToTarget) {
            int currentPos = m0.getCurrentPosition();
            int error = targetPositionM0 - currentPos;

            if (Math.abs(error) > POSITION_TOLERANCE) {
                double power = clamp(error * M0_KP, -M0_MAX_POWER, M0_MAX_POWER);
                m0.setPower(power);
            } else {
                m0.setPower(0);
                movingToTarget = false;
            }
        } else {
            m0.setPower(0);
        }

        // === Encoder Telemetry for m0 ===
        int encoderPosM0 = m0.getCurrentPosition();
        long nowNs = System.nanoTime();
        double dtEncM0 = (nowNs - lastTimeNsM0) / 1e9;
        if (dtEncM0 > 0.05) {
            encoderVelocityM0 = (encoderPosM0 - lastEncoderPosM0) / dtEncM0;
            lastTimeNsM0 = nowNs;
            lastEncoderPosM0 = encoderPosM0;
        }

        // === Encoder Telemetry for m2 ===
        int encoderPosM2 = m2.getCurrentPosition();
        double dtEncM2 = (nowNs - lastTimeNsM2) / 1e9;
        if (dtEncM2 > 0.05) {
            encoderVelocityM2 = (encoderPosM2 - lastEncoderPosM2) / dtEncM2;
            lastTimeNsM2 = nowNs;
            lastEncoderPosM2 = encoderPosM2;
        }

        // === Telemetry Output ===
        telemetry.addLine("=== m0 Position Control ===");
        telemetry.addData("Target", targetPositionM0);
        telemetry.addData("Current", encoderPosM0);
        telemetry.addData("Moving?", movingToTarget);
        telemetry.addData("Velocity (ticks/sec)", "%.1f", encoderVelocityM0);
        telemetry.addLine("=== m2 Telemetry ===");
        telemetry.addData("Position", encoderPosM2);
        telemetry.addData("Velocity (ticks/sec)", "%.1f", encoderVelocityM2);
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
}
