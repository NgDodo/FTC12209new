package org.firstinspires.ftc.teamcode.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ElapsedTime pathTimer; // Timer for action delays
    private ElapsedTime autoTimer; // Overall autonomous timer

    // State machine states
    private enum State {
        IDLE,
        FOLLOW_PATH_1,
        WAIT_AT_PATH_1,
        FOLLOW_PATH_2,
        WAIT_AT_PATH_2,
        FOLLOW_PATH_3,
        WAIT_AT_PATH_3,
        FOLLOW_PATH_4,
        WAIT_AT_PATH_4,
        FOLLOW_PATH_5,
        WAIT_AT_PATH_5,
        FOLLOW_PATH_6,
        WAIT_AT_PATH_6,
        FINISHED
    }

    private State currentState = State.IDLE;

    // Wait time between paths (milliseconds)
    private static final long WAIT_TIME_MS = 500; // 0.5 seconds - adjust as needed

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        pathTimer = new ElapsedTime();
        autoTimer = new ElapsedTime();

        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Total Paths", 6);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        autoTimer.reset();
        currentState = State.FOLLOW_PATH_1;
        follower.followPath(paths.Path1);
        pathTimer.reset();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Current State", currentState.toString());
        panelsTelemetry.debug("X", String.format("%.2f", follower.getPose().getX()));
        panelsTelemetry.debug("Y", String.format("%.2f", follower.getPose().getY()));
        panelsTelemetry.debug("Heading", String.format("%.2f°", Math.toDegrees(follower.getPose().getHeading())));
        panelsTelemetry.debug("Is Busy", follower.isBusy());
        panelsTelemetry.debug("Auto Time", String.format("%.2f s", autoTimer.seconds()));
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(122.000, 122.000), new Pose(88.000, 88.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.000, 88.000),
                                    new Pose(100.000, 84.000),
                                    new Pose(125.000, 83.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.000, 83.000), new Pose(85.000, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.000, 85.000),
                                    new Pose(99.000, 55.000),
                                    new Pose(125.000, 60.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.000, 60.000), new Pose(85.000, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(85.000, 85.000), new Pose(100.000, 75.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (currentState) {
            case IDLE:
                // Waiting for start
                pathState = 0;
                break;

            case FOLLOW_PATH_1:
                // Following Path 1
                pathState = 1;
                if (!follower.isBusy()) {
                    currentState = State.WAIT_AT_PATH_1;
                    pathTimer.reset();
                }
                break;

            case WAIT_AT_PATH_1:
                // Wait at end of Path 1
                pathState = 2;
                if (pathTimer.milliseconds() >= WAIT_TIME_MS) {
                    currentState = State.FOLLOW_PATH_2;
                    follower.followPath(paths.Path2);
                    pathTimer.reset();
                }
                // TODO: Add mechanism actions here if needed
                break;

            case FOLLOW_PATH_2:
                // Following Path 2
                pathState = 3;
                if (!follower.isBusy()) {
                    currentState = State.WAIT_AT_PATH_2;
                    pathTimer.reset();
                }
                break;

            case WAIT_AT_PATH_2:
                // Wait at end of Path 2
                pathState = 4;
                if (pathTimer.milliseconds() >= WAIT_TIME_MS) {
                    currentState = State.FOLLOW_PATH_3;
                    follower.followPath(paths.Path3);
                    pathTimer.reset();
                }
                // TODO: Add mechanism actions here if needed
                break;

            case FOLLOW_PATH_3:
                // Following Path 3
                pathState = 5;
                if (!follower.isBusy()) {
                    currentState = State.WAIT_AT_PATH_3;
                    pathTimer.reset();
                }
                break;

            case WAIT_AT_PATH_3:
                // Wait at end of Path 3
                pathState = 6;
                if (pathTimer.milliseconds() >= WAIT_TIME_MS) {
                    currentState = State.FOLLOW_PATH_4;
                    follower.followPath(paths.Path4);
                    pathTimer.reset();
                }
                // TODO: Add mechanism actions here if needed
                break;

            case FOLLOW_PATH_4:
                // Following Path 4
                pathState = 7;
                if (!follower.isBusy()) {
                    currentState = State.WAIT_AT_PATH_4;
                    pathTimer.reset();
                }
                break;

            case WAIT_AT_PATH_4:
                // Wait at end of Path 4
                pathState = 8;
                if (pathTimer.milliseconds() >= WAIT_TIME_MS) {
                    currentState = State.FOLLOW_PATH_5;
                    follower.followPath(paths.Path5);
                    pathTimer.reset();
                }
                // TODO: Add mechanism actions here if needed
                break;

            case FOLLOW_PATH_5:
                // Following Path 5
                pathState = 9;
                if (!follower.isBusy()) {
                    currentState = State.WAIT_AT_PATH_5;
                    pathTimer.reset();
                }
                break;

            case WAIT_AT_PATH_5:
                // Wait at end of Path 5
                pathState = 10;
                if (pathTimer.milliseconds() >= WAIT_TIME_MS) {
                    currentState = State.FOLLOW_PATH_6;
                    follower.followPath(paths.Path6);
                    pathTimer.reset();
                }
                // TODO: Add mechanism actions here if needed
                break;

            case FOLLOW_PATH_6:
                // Following Path 6
                pathState = 11;
                if (!follower.isBusy()) {
                    currentState = State.WAIT_AT_PATH_6;
                    pathTimer.reset();
                }
                break;

            case WAIT_AT_PATH_6:
                // Wait at end of Path 6
                pathState = 12;
                if (pathTimer.milliseconds() >= WAIT_TIME_MS) {
                    currentState = State.FINISHED;
                }
                // TODO: Add final mechanism actions here if needed
                break;

            case FINISHED:
                // Autonomous completed
                pathState = 13;
                // Optional: Hold position, stop mechanisms, etc.
                break;
        }

        return pathState;
    }

    @Override
    public void stop() {
        // Clean up when autonomous ends
        follower.breakFollowing();

        // TODO: Stop all mechanisms here
        // Examples:
        // - Set motor powers to 0
        // - Reset servo positions
        // - Turn off LEDs
        // - etc.
    }
}