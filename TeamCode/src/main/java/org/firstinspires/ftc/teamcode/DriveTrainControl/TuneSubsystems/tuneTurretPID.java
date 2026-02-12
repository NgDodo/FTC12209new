package org.firstinspires.ftc.teamcode.DriveTrainControl.TuneSubsystems;

import com.bylazar.configurables.annotations.Configurable;
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

@TeleOp(name = "tuneTurretPID", group = "!")
public class tuneTurretPID extends OpMode {
    // === Drive Train & Mechanisms ===
    Turret turret;

    private Follower follower;
    public static Pose startingPose;

    private boolean lastBButton, lastXButton;

    private ElapsedTime loopTime = new ElapsedTime();

    private boolean runTelemetry = false;
    private ElapsedTime telemetryLimiter = new ElapsedTime();


    @Override
    public void init() {
        startingPose = new Pose(72, 72, Math.toRadians(90));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        turret = new Turret(hardwareMap, "BLUE");

        loopTime.reset();
    }

    public void start() {
        telemetryLimiter.reset();
    }
    @Override
    public void loop() {
        turret.updateTurret(follower, gamepad1);

        // === Telemetry === //
        updateTelemetry();
        loopTime.reset();

    }
    private void updateTelemetry() {

    }
    private double clipLowPower(double p) {
        return Math.abs(p) < 0.04 ? 0 : p;
    }
    private double applyDeadzone(double v) {
        return Math.abs(v) < 0.05 ? 0 : v;
    }



}
