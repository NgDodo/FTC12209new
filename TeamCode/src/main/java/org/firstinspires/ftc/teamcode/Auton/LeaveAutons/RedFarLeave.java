package org.firstinspires.ftc.teamcode.Auton.LeaveAutons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "RedFarLeave", group = "Autonomous")
@Configurable
public class RedFarLeave extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private ElapsedTime autoTimer;
    private ElapsedTime pathTimer;

    // Path from Pedro Pathing Visualizer
    private PathChain Path1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize follower with starting pose
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8.0, Math.toRadians(0)));

        // Build the path
        buildPaths();

        // Initialize timers
        autoTimer = new ElapsedTime();
        pathTimer = new ElapsedTime();
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Starting Pose", "X: 56.0, Y: 8.0, Heading: 180°");
        panelsTelemetry.update(telemetry);
    }

    private void buildPaths() {
        // Path from visualizer: (56, 8, 180°) -> (40, 12, 180°)
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88, 8.0),
                        new Pose(108, 12.0)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    @Override
    public void start() {
        autoTimer.reset();
        pathTimer.reset();
        pathState = 0;
        // Start following the path
        follower.followPath(Path1);
    }

    @Override
    public void loop() {
        // Update follower (required for Pedro Pathing)
        follower.update();

        // Update autonomous state machine
        pathState = autonomousPathUpdate();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", String.format("%.2f", follower.getPose().getX()));
        panelsTelemetry.debug("Y", String.format("%.2f", follower.getPose().getY()));
        panelsTelemetry.debug("Heading", String.format("%.1f°", Math.toDegrees(follower.getPose().getHeading())));
        panelsTelemetry.debug("Auto Time", String.format("%.2f s", autoTimer.seconds()));
        panelsTelemetry.debug("Path Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    /**
     * Autonomous state machine
     * Manages the progression through different autonomous states
     */
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Following Path1
                if (!follower.isBusy()) {
                    // Path1 is complete, move to next state
                    pathTimer.reset();
                    pathState++;
                }
                break;

            case 1: // Path complete - autonomous finished
                // You can add additional logic here if needed
                // For example, stopping motors, holding position, etc.
                break;

            default:
                // Should never reach here
                break;
        }

        return pathState;
    }

    @Override
    public void stop() {
        // Stop following paths
        follower.breakFollowing();
    }
}