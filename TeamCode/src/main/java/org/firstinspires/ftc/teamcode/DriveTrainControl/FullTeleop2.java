package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Full Teleop (Sorter + Turret Final, True Offset)", group = "DriveTrainControl")
public class FullTeleop2 extends OpMode {

    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1, m2;
    DcMotorEx m3, m0;
    CRServo s1, s3;
    Servo s2;

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
    private static final int SORTER_STEP_TICKS = 2405;   // per D-pad tap
    private static final int SORTER_OFFSET_TICKS = 1080; // per Y toggle
    private static final double SORTER_POWER = 1.0;

    private boolean sorterOffsetActive = false; // Y toggled offset mode
    private boolean lastYPressed = false;
    private boolean sorterMoving = false;
    private int sorterTargetPosition = 0;

    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    // === Shooter presets ===
    private final int[] rpmPresets = {3200, 3500, 3800};
    private int presetIndex = -1;
    private double targetRPM = 0;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    private static final double TICKS_PER_REV = 28.0;

    // === Encoder tracking ===
    private int lastEncoderPosM2 = 0;
    private long lastTimeNsM2 = 0;
    private double encoderVelocityM2 = 0;

    private int lastEncoderPosM0 = 0;
    private long lastTimeNsM0 = 0;
    private double encoderVelocityM0 = 0;

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
        m2 = hardwareMap.get(DcMotor.class, "m2");  // external encoder
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

        // === Vision setup ===
        turretPD = new PDController(kP, kD);
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        lastEncoderPosM2 = m2.getCurrentPosition();
        lastTimeNsM2 = System.nanoTime();
        lastEncoderPosM0 = m0.getCurrentPosition();
        lastTimeNsM0 = System.nanoTime();

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

        // === Sorter Control ===
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean yPressed = gamepad1.y;

        // --- D-pad Left/Right Rotation (2375 ticks) ---
        if ((dpadRight && !lastDpadRight) || (dpadLeft && !lastDpadLeft)) {
            int direction = dpadRight ? 1 : -1;
            rotateSorterTicks(SORTER_STEP_TICKS * direction);
        }
        lastDpadRight = dpadRight;
        lastDpadLeft = dpadLeft;

        // --- Y Toggle Offset (±1187 ticks physical spin) ---
        if (yPressed && !lastYPressed) {
            sorterOffsetActive = !sorterOffsetActive;
            int direction = sorterOffsetActive ? 1 : -1;
            rotateSorterTicks(SORTER_OFFSET_TICKS * direction);
        }
        lastYPressed = yPressed;

        // === Intake motors (m1/m2) ===
        double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
        m1.setPower(-triggerPower);
        m2.setPower(triggerPower);

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

        updateEncoderTelemetry();
    }

    // === Helper to spin sorter physically by tick distance ===
    private void rotateSorterTicks(int ticksToMove) {
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double direction = Math.signum(ticksToMove);
        m0.setPower(SORTER_POWER * direction);

        while (opModeIsActive() && Math.abs(m2.getCurrentPosition()) < Math.abs(ticksToMove)) {
            telemetry.addData("Sorter Encoder (m2)", m2.getCurrentPosition());
            telemetry.addData("Target", ticksToMove);
            telemetry.update();
        }

        m0.setPower(0);
    }

    private void updateEncoderTelemetry() {
        int posM2 = m2.getCurrentPosition();
        long nowNs = System.nanoTime();
        double dtEncM2 = (nowNs - lastTimeNsM2) / 1e9;
        if (dtEncM2 > 0.05) {
            encoderVelocityM2 = (posM2 - lastEncoderPosM2) / dtEncM2;
            lastTimeNsM2 = nowNs;
            lastEncoderPosM2 = posM2;
        }

        telemetry.addLine("=== Telemetry ===");
        telemetry.addData("Sorter Encoder (m2)", posM2);
        telemetry.addData("Sorter Offset Active", sorterOffsetActive);
        telemetry.addData("Turret Tracking", turretTrackingEnabled ? "ON" : "OFF");
        telemetry.addData("Target RPM (m3)", targetRPM);
        telemetry.addData("Actual RPM (m3)", ((m3.getVelocity()/TICKS_PER_REV)*60));
        telemetry.update();
    }

    private double applyDeadzone(double v) { return Math.abs(v) < 0.05 ? 0 : v; }
    private double clipLowPower(double p) { return Math.abs(p) < 0.04 ? 0 : p; }

    private boolean opModeIsActive() {
        // mimic LinearOpMode check in iterative OpMode
        return !isStopRequested();
    }

    private boolean isStopRequested() {
        return false;
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }
}
