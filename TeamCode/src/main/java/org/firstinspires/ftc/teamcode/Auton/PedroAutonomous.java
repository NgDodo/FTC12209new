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
        ACTION_AT_PATH_1_END,
        FOLLOW_PATH_2,
        ACTION_AT_PATH_2_END,
        FINISHED
    }

    private State currentState = State.IDLE;

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
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Is Busy", follower.isBusy());
        panelsTelemetry.debug("Auto Time", String.format("%.2f", autoTimer.seconds()));
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;

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
                                    new Pose(110.000, 83.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
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
                    // Path 1 completed, move to action
                    currentState = State.ACTION_AT_PATH_1_END;
                    pathTimer.reset();
                }
                break;

            case ACTION_AT_PATH_1_END:
                // Perform action at end of Path 1
                pathState = 2;

                // Example: Wait for 1 second (simulate scoring, intake, etc.)
                if (pathTimer.milliseconds() >= 1000) {
                    // Action completed, start Path 2
                    currentState = State.FOLLOW_PATH_2;
                    follower.followPath(paths.Path2);
                    pathTimer.reset();
                }

                // TODO: Add your mechanism actions here
                // Examples:
                // - Score specimen
                // - Intake sample
                // - Deploy mechanism
                // - etc.

                break;

            case FOLLOW_PATH_2:
                // Following Path 2
                pathState = 3;

                if (!follower.isBusy()) {
                    // Path 2 completed, move to action
                    currentState = State.ACTION_AT_PATH_2_END;
                    pathTimer.reset();
                }
                break;

            case ACTION_AT_PATH_2_END:
                // Perform action at end of Path 2
                pathState = 4;

                // Example: Wait for 1 second
                if (pathTimer.milliseconds() >= 1000) {
                    // Action completed, finish autonomous
                    currentState = State.FINISHED;
                }

                // TODO: Add your mechanism actions here

                break;

            case FINISHED:
                // Autonomous completed
                pathState = 5;

                // Optional: Stop all mechanisms, hold position, etc.

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