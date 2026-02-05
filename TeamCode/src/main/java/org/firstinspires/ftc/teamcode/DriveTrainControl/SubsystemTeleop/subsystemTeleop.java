package org.firstinspires.ftc.teamcode.DriveTrainControl.SubsystemTeleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Sorter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Subsystem Teleop", group = "!")
public class subsystemTeleop extends OpMode {
    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    Intake intake;
    Sorter sorter;
    Turret turret;

    private Follower follower;
    public static Pose startingPose;

    private boolean lastBButton;

    private ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void init() {
        startingPose = new Pose(72, 72, Math.toRadians(90));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

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

        intake = new Intake(hardwareMap);
        sorter = new Sorter(hardwareMap);
        turret = new Turret(hardwareMap, "BLUE");

        loopTime.reset();
    }

    @Override
    public void loop() {

        // === Drive Train ===
        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
        double rx = applyDeadzone(gamepad1.right_stick_x);

        double fl = y + x + rx, bl = y - x + rx, fr = y - x - rx, br = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeftMotor.setPower(clipLowPower(fl / max));
        backLeftMotor.setPower(clipLowPower(bl / max));
        frontRightMotor.setPower(clipLowPower(fr / max));
        backRightMotor.setPower(clipLowPower(br / max));

        follower.update();

        // === Reset Follower Pose === //
        boolean bPressed = gamepad1.b;
        if (bPressed && !lastBButton) follower.setPose(new Pose(72, 72, Math.PI / 2));
        lastBButton = bPressed;

        // === Update Subsystems === //
        intake.updateIntake(gamepad1);
        sorter.updateSorter(gamepad1);
        turret.updateTurret(follower, gamepad1);

        // === Telemetry === //
        updateTelemetry();
        loopTime.reset();
    }
    private void updateTelemetry() {
        intake.postTelemetry(telemetry);
        sorter.postTelemetry(telemetry);
        turret.postTelemetry(telemetry);

        // === Update Loop Time Tracking ===
        telemetry.addData("Loop Time (Hz)", 1.0 / loopTime.seconds());

        // === Manual Motor Power Read
        telemetry.addData("Intake Power: ", intake.intakeMotor.getPower());
        telemetry.addData("Intake Velocity: ", intake.intakeMotor.getVelocity());

        // === Control Reference ===
        telemetry.addLine("=== Controls ===");
        telemetry.addLine("B: Cycle Track Mode");
        telemetry.addLine("Y: Mode | DpadRight: Chamber");
        telemetry.addLine("A: Shoot");
        telemetry.addLine("RB: RPM | LB: 1500 | DpadDown: Off");

        telemetry.update();
    }
    private double clipLowPower(double p) {
        return Math.abs(p) < 0.04 ? 0 : p;
    }
    private double applyDeadzone(double v) {
        return Math.abs(v) < 0.05 ? 0 : v;
    }
}
