package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Simple Path Test", group = "Tests")
public class SimplePathTest extends OpMode {

    private Follower follower;
    private Timer timer = new Timer();
    private int state = 0;

    // === Drive Train ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // ========================================
    // === TUNABLE PATH PARAMETERS ===
    // ========================================

    // Starting position and heading
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // Path 1: Move forward
    private final Pose pose1 = new Pose(60, 0, Math.toRadians(0)); // Forward 60 inches

    // Path 2: Strafe
    private final Pose pose2 = new Pose(60, 24, Math.toRadians(0)); // Strafe left 24 inches

    // Path 3: Return home (optional)
    private final Pose pose3 = new Pose(0, 0, Math.toRadians(0)); // Back to start

    private Path path1;
    private Path path2;
    private Path path3;

    // Wait time at each position (seconds)
    private static final double WAIT_TIME = 1.0;

    @Override
    public void init() {
        // === Initialize Pedro follower ===
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // === Build paths ===
        path1 = new Path(new BezierLine(startPose, pose1));
        path1.setConstantHeadingInterpolation(startPose.getHeading());

        path2 = new Path(new BezierLine(pose1, pose2));
        path2.setConstantHeadingInterpolation(pose1.getHeading());

        path3 = new Path(new BezierLine(pose2, pose3));
        path3.setConstantHeadingInterpolation(pose2.getHeading());

        // === Initialize drive motors (for manual control if needed) ===
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

        telemetry.addLine("=== Simple Path Test Initialized ===");
        telemetry.addLine();
        telemetry.addLine("Path 1: Forward 60 inches");
        telemetry.addLine("Path 2: Strafe left 24 inches");
        telemetry.addLine("Path 3: Return to start");
        telemetry.addLine();
        telemetry.addData("Start Pose", String.format("(%.1f, %.1f, %.1f°)",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading())));
        telemetry.update();
    }

    @Override
    public void start() {
        timer.resetTimer();
        follower.followPath(path1);
        state = 0;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case 0:
                // Following Path 1 (forward)
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    state = 1;
                }
                break;

            case 1:
                // Wait at position 1
                if (timer.getElapsedTimeSeconds() > WAIT_TIME) {
                    follower.followPath(path2);
                    timer.resetTimer();
                    state = 2;
                }
                break;

            case 2:
                // Following Path 2 (strafe)
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    state = 3;
                }
                break;

            case 3:
                // Wait at position 2
                if (timer.getElapsedTimeSeconds() > WAIT_TIME) {
                    follower.followPath(path3);
                    timer.resetTimer();
                    state = 4;
                }
                break;

            case 4:
                // Following Path 3 (return to start)
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    state = 5;
                }
                break;

            case 5:
                // Finished - hold position
                break;
        }

        // === Telemetry ===
        Pose currentPose = follower.getPose();

        telemetry.addLine("=== Simple Path Test ===");
        telemetry.addLine();

        telemetry.addData("State", getStateName(state));
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.addLine();

        telemetry.addLine("--- Current Position ---");
        telemetry.addData("X", String.format("%.2f inches", currentPose.getX()));
        telemetry.addData("Y", String.format("%.2f inches", currentPose.getY()));
        telemetry.addData("Heading", String.format("%.2f°", Math.toDegrees(currentPose.getHeading())));
        telemetry.addLine();

        telemetry.addLine("--- Target Positions ---");
        switch (state) {
            case 0:
                telemetry.addData("Target", String.format("Path 1: (%.1f, %.1f)", pose1.getX(), pose1.getY()));
                break;
            case 1:
                telemetry.addData("Target", "Waiting at position 1");
                break;
            case 2:
                telemetry.addData("Target", String.format("Path 2: (%.1f, %.1f)", pose2.getX(), pose2.getY()));
                break;
            case 3:
                telemetry.addData("Target", "Waiting at position 2");
                break;
            case 4:
                telemetry.addData("Target", String.format("Path 3: (%.1f, %.1f)", pose3.getX(), pose3.getY()));
                break;
            case 5:
                telemetry.addData("Target", "Complete!");
                break;
        }

        telemetry.update();
    }

    private String getStateName(int state) {
        switch (state) {
            case 0: return "Following Path 1 (Forward)";
            case 1: return "Waiting at Position 1";
            case 2: return "Following Path 2 (Strafe)";
            case 3: return "Waiting at Position 2";
            case 4: return "Following Path 3 (Return)";
            case 5: return "Finished";
            default: return "Unknown";
        }
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}