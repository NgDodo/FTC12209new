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
import org.firstinspires.ftc.teamcode.Subsystems.TurretV2;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Subsystem Teleop---BLUE", group = "!")
public class subsystemTeleopBlue extends OpMode {
    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    Intake intake;
    Sorter sorter;
    TurretV2 turret;

    private Follower follower;
    public static Pose startingPose;

    private boolean lastBButton, lastXButton, lastRightBumperPressed, lastLeftBumperPressed, lastDpadLeftPressed, lastDpadRightPressed;

    private ElapsedTime loopTime = new ElapsedTime();

    private boolean runTelemetry = false;
    private ElapsedTime telemetryLimiter = new ElapsedTime();


    @Override
    public void init() {
        startingPose = new Pose(142.500 - 87.000,  120, Math.toRadians(169));
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
        turret = new TurretV2(hardwareMap, "BLUE");

        loopTime.reset();
    }

    public void start() {
        telemetryLimiter.reset();
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
        boolean rightBumperPressed = gamepad2.right_bumper;
        boolean leftBumperPressed = gamepad2.left_bumper;
        boolean dpadRightPressed = gamepad2.dpad_right;
        boolean dpadLeftPressed = gamepad2.dpad_left;

        if (bPressed && !lastBButton) {
            follower.setPose(new Pose(18.4, 123.1, Math.toRadians(144)));
        }
        /// Verbose Telemetry
        /*
        if (bPressed && !lastBButton) {
            if (runTelemetry) {
                runTelemetry = false;
            } else {
                runTelemetry = true;
            }
        }
        */

        /// update sorter manual offset

        if (rightBumperPressed && !lastRightBumperPressed) {
            turret.updateManualOffset(0.05);
        }
        if (leftBumperPressed && !lastLeftBumperPressed) {
            turret.updateManualOffset(-0.05);
        }
        if (dpadRightPressed && !lastDpadRightPressed) {
            sorter.updateManualOffset(200);
        }
        if (dpadLeftPressed && !lastDpadLeftPressed) {
            sorter.updateManualOffset(-200);
        }

        // === Update Subsystems === //
        intake.updateIntake(gamepad1);
        sorter.updateSorter(gamepad1);
        turret.updateTurret(follower, gamepad1);

        // === Telemetry === //
        updateTelemetry();
        loopTime.reset();

        lastBButton = bPressed;
        lastRightBumperPressed = rightBumperPressed;
        lastLeftBumperPressed = leftBumperPressed;
        lastDpadLeftPressed = dpadLeftPressed;
        lastDpadRightPressed = dpadRightPressed;
    }
    private void updateTelemetry() {
        if (runTelemetry) {
            intake.postTelemetry(telemetry);
            sorter.postTelemetry(telemetry);
            turret.postTelemetry(telemetry, follower);

            // === Manual Motor Power Read
            telemetry.addData("Intake Power: ", intake.intakeMotor.getPower());
            telemetry.addData("Intake Velocity: ", intake.intakeMotor.getVelocity());

            // === Control Reference ===
            telemetry.addLine("=== Controls ===");
            telemetry.addLine("B: Cycle Track Mode");
            telemetry.addLine("Y: Mode | DpadRight: Chamber");
            telemetry.addLine("A: Shoot");
            telemetry.addLine("RB: RPM | LB: 1500 | DpadDown: Off");
        }
        if (telemetryLimiter.seconds() > 0.5) {
            turret.postTelemetry(telemetry, follower);

            // === Update Loop Time Tracking ===
            telemetry.addData("Loop Time (Hz)", 1.0 / loopTime.seconds());
            telemetry.addData("Position", follower.getPose());
            telemetry.addData("Sorter State", sorter.sorterState);
            double currentRPM = (turret.getFlywheelMotor().getVelocity() / turret.TICKS_PER_REV_FLYWHEEL) * 60.0;
            double rpmError = Math.abs(turret.targetRPM - currentRPM);

            telemetry.addLine("=== Shooter ===");
            telemetry.addData("Target RPM", turret.targetRPM);
            telemetry.addData("Actual RPM", String.format("%.0f", currentRPM));
            telemetry.addData("Limelight Tracking? ", turret.limelightTracking);
            telemetry.addData("TurretMOTIF", sorter.currentMotif);

            telemetry.update();
            telemetryLimiter.reset();
        }

    }
    private double clipLowPower(double p) {
        return Math.abs(p) < 0.04 ? 0 : p;
    }
    private double applyDeadzone(double v) {
        return Math.abs(v) < 0.05 ? 0 : v;
    }



}
