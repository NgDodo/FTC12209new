package org.firstinspires.ftc.teamcode.DriveTrainControl.DeadWheelTracking;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name="LimeLight Metrics", group="DriveTrainControl")
public class LimelightMetrics extends OpMode {
    private Follower follower;
    public static Pose startingPose;

    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1;
    DcMotorEx m2; // Turret motor
    DcMotorEx m3, m0;
    CRServo s3;
    Servo s2;

    private static final double TICKS_PER_REV = 1393.1;

    // === Vision & IMU ===
    private Limelight3A limelight;
    private IMU imu;

    // === Limelight Configuration ===
    private static final String LIMELIGHT_NAME = "Webcam 2";
    private static final int APRILTAG_PIPELINE = 1;

    @Override
    public void init() {
        startingPose = new Pose(72, 72, Math.toRadians(90));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        // === DriveTrain ===
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fL");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "bR");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // === Mechanisms ===
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m0 = hardwareMap.get(DcMotorEx.class, "m0");

        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");
        s3.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setPosition(.68);

        for (DcMotor motor : new DcMotor[]{m1, m3, m0}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

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
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        // === Turret Setup ===
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        follower.update();

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

        // === Turret Rotation Logic ===
        Pose GOAL_POST = new Pose(10, 134, 0);

        boolean LimelightTracking = false;

        // IF apriltag in view, do LimeLight tracking.
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            LLResultTypes.FiducialResult validTarget = null;
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == 20) {
                    telemetry.addData("Found Tag: ", fiducial.getFiducialId());
                    validTarget = fiducial;
                    LimelightTracking = true;
                    double bearing = validTarget.getTargetXDegrees();

                    telemetry.addData("getTargetXDeg/bearing: ", bearing);
                    telemetry.addData("getTargetXDeg no crosshair", validTarget.getTargetXDegreesNoCrosshair());
                    telemetry.addData("------------------: ", "----------------");

                    telemetry.addData("getTargetYDeg", validTarget.getTargetYDegrees());
                    telemetry.addData("getTargetYDeg no crosshair", validTarget.getTargetYDegreesNoCrosshair());
                    telemetry.addData("------------------: ", "----------------");

                    telemetry.addData("get Robot pose target: ", validTarget.getRobotPoseTargetSpace());
                    telemetry.addData("get Robot pose field: ", validTarget.getRobotPoseFieldSpace());

                }
            }
        }

        // === Pose Reset System ===
        if (gamepad1.dpad_down) {
            follower.setPose(new Pose(72, 72, Math.PI / 2));
        }

        // === Telemetry ===
        telemetry.addData("Position (x, y, heading)",
                String.format("%.1f, %.1f, %.1f°",
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        Math.toDegrees(follower.getHeading())));
        telemetry.update();
    }

    private double moveTurretToOffset(DcMotorEx turretMotor, double turretDesiredRelativeOffset) {
        double turretDesiredDegrees = Math.toDegrees(turretDesiredRelativeOffset);
        double turretRotations = turretMotor.getCurrentPosition() / TICKS_PER_REV;

        double desiredRotations = turretDesiredDegrees / 360.0;

        double error = desiredRotations - turretRotations;

        if (Math.abs(error) > 0.02) { // 0.02 rotations is a reasonable tolerance
            // turretMotor.setPower(error / Math.abs(error) * gamepad1.right_trigger);
            turretMotor.setPower(error / Math.abs(error) * 0.2);
        } else {
            turretMotor.setPower(0);
        }
        return error;
    }


    /**
     * Normalizes an angle to the range [-PI, PI]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clipLowPower(double p) {
        return Math.abs(p) < 0.04 ? 0 : p;
    }

    private double applyDeadzone(double v) {
        return Math.abs(v) < 0.05 ? 0 : v;
    }
}