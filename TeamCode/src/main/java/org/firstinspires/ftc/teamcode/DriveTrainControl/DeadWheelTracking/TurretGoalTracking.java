package org.firstinspires.ftc.teamcode.DriveTrainControl.DeadWheelTracking;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp(name="TurretGoalTracking", group="DriveTrainControl")
public class TurretGoalTracking extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1;
    DcMotorEx m2; // Turret motor
    DcMotorEx m3, m0;
    CRServo s3;
    Servo s2;

    // === Color Sensors & RGB LEDs ===
    private RevColorSensorV3 intakeColor;
    private RevColorSensorV3 shooterColor;
    private Servo led1;
    private Servo led2;

    // === Turret Configuration ===
    private static final double TURRET_RANGE_DEG = 330.0;
    private static final double TICKS_PER_REV = 1393.1;
    private static final double OUTPUT_GEAR_TEETH = 44;
    private static final double INPUT_GEAR_TEETH = 18;
    private static final double GEAR_RATIO = INPUT_GEAR_TEETH / OUTPUT_GEAR_TEETH;
    private static final double TURRET_POWER = 0.3; // Adjust as needed for your turret
    private static final double TURRET_TOLERANCE_TICKS = 10; // How close is "close enough"

    @Override
    public void init() {
        startingPose = new Pose(72, 72, Math.toRadians(90));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

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

        // 1. Calculate component distances from goal
        double y_goal_distance = follower.getPose().getY() - GOAL_POST.getY();
        double x_goal_distance = follower.getPose().getX() - GOAL_POST.getX();

        // 2. Calculate absolute angle to goal in field coordinates
        double angle_to_goal = Math.atan2(y_goal_distance, x_goal_distance);

        // 3. Calculate turret offset relative to robot heading
        // Normalize the angle difference to [-PI, PI]
        double turretRelativeOffset = normalizeAngle(angle_to_goal - follower.getHeading());

        // 4. Move turret to track the goal
        moveTurretToOffset(m2, turretRelativeOffset);

        // === Pose Reset System ===
        if (gamepad1.dpad_down) {
            follower.setPose(new Pose(72, 10, Math.PI / 2));
        }

        // === Telemetry ===
        telemetry.addData("Position (x, y, heading)",
                String.format("%.1f, %.1f, %.1f°",
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        Math.toDegrees(follower.getHeading())));
        telemetry.addData("Angle to Goal", "%.1f°", Math.toDegrees(angle_to_goal));
        telemetry.addData("Turret Relative Offset", "%.1f°", Math.toDegrees(turretRelativeOffset));
        telemetry.addData("Turret Position", m2.getCurrentPosition());
        telemetry.addData("Turret Target", m2.getTargetPosition());
        telemetry.addData("Turret Error", m2.getTargetPosition() - m2.getCurrentPosition());
        telemetry.addData("Turret Rotations", "%.2f", m2.getCurrentPosition() / TICKS_PER_REV);
        telemetry.update();
    }

    private void moveTurretToOffset(DcMotorEx turretMotor, double turretRelativeOffset) {

        double turretDegrees = Math.toDegrees(turretRelativeOffset);

        // CORRECT gear relationship
        double motorDegrees = turretDegrees * GEAR_RATIO;

        double motorTicks = (motorDegrees / 360.0) * TICKS_PER_REV;

        int targetPosition = (int) Math.round(motorTicks);

        turretMotor.setTargetPosition(targetPosition);

        int error = Math.abs(targetPosition - turretMotor.getCurrentPosition());

        if (error > TURRET_TOLERANCE_TICKS) {
            //turretMotor.setPower(TURRET_POWER);
        } else {
            //turretMotor.setPower(0);
        }

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