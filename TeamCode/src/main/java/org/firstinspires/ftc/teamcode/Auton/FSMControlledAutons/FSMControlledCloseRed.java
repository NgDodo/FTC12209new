package org.firstinspires.ftc.teamcode.Auton.FSMControlledAutons;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Sorter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FSMControlledCloseRed", group = "!Autonomous")
public class FSMControlledCloseRed extends OpMode {
    public Follower follower;
    private int pathState;
    private ElapsedTime autoTimer;
    private ElapsedTime pathTimer;

    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

    private Intake intake;
    private Sorter sorter;
    private Turret turret;

    private ElapsedTime telemetryLimiter = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();

    private boolean lastFollowerBusyState = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123.1, 123.1, Math.toRadians(36)));

        intake = new Intake(hardwareMap);
        sorter = new Sorter(hardwareMap);
        turret = new Turret(hardwareMap, "RED");

        autoTimer = new ElapsedTime();
        pathTimer = new ElapsedTime();

        buildPaths();

    }

    @Override
    public void start() {
        autoTimer.reset();
        pathTimer.reset();
        pathState = 0;

        follower.followPath(Path1);

        // turret.setFlywheelRPM("NEAR"); // rev up flywheel

        sorter.chamberColors[0] = "GREEN";
        sorter.chamberColors[1] = "PURPLE";
        sorter.chamberColors[2] = "PURPLE";

        telemetryLimiter.reset();
        loopTime.reset();
    }

    @Override
    public void loop() {
        follower.update();
        /// Update Subsystem Classes here
        intake.updateIntake();
        sorter.updateSorter(gamepad1);
        turret.updateTurret(follower);
        // pathState = autonomousPathUpdateOnlyPathMovements();
        pathState = autonomousPathUpdateOnlyPathMovements();

        if (telemetryLimiter.seconds() > 0.5) {
            // === Update Loop Time Tracking
            telemetry.addData("Loop Time (Hz)", 1.0 / loopTime.seconds());
            telemetry.addData("Position", follower.getPose());

            telemetry.update();
            telemetryLimiter.reset();
        }

    }

    public int autonomousPathUpdateOnlyPathMovements() {
        switch (pathState) {
            case 0: // Path 1
                if (!follower.isBusy() && pathTimer.seconds() > 3.0) {
                    pathState = 100;
                    follower.followPath(Path2);
                    pathTimer.reset();
                }
                break;
            case 100: // Path 2
                if (!follower.isBusy() && pathTimer.seconds() > 3.0) { // wait until reaches path
                    pathState = 200; // Continue to Path 2
                    follower.followPath(Path3);
                    pathTimer.reset();
                }
            case 200:
                if (!follower.isBusy() && pathTimer.seconds() > 3.0) {
                    pathState = 300; // Continue to Path 2
                    follower.followPath(Path4);
                    pathTimer.reset();
                }
            case 300:
                if (!follower.isBusy() && pathTimer.seconds() > 3.0) {
                    pathState = 400; // Continue to Path 2
                    follower.followPath(Path5);
                    pathTimer.reset();
                }
            case 400:
                if (!follower.isBusy() && pathTimer.seconds() > 3.0) {
                    pathState = 500; // Continue to Path 2
                    follower.followPath(Path6);
                    pathTimer.reset();
                }
            case 500:
                if (!follower.isBusy() && pathTimer.seconds() > 3.0) {
                    pathState = 600; // Continue to Path 2
                    follower.followPath(Path7);
                    pathTimer.reset();
                }
            case 600:
                if (!follower.isBusy() && pathTimer.seconds() > 3.0) {
                    pathState = -1; // Continue to Path 2
                    follower.followPath(Path8);
                    pathTimer.reset();
                }
            case -1:
                //end
                break;
        }
        return pathState;
    }

    public int autonomousPathUpdateFull() {
        boolean followerBusy = follower.isBusy();
        switch (pathState) {
            case 0: // Move to shooting position
                if (!followerBusy && pathTimer.seconds() > 3.0) {
                    sorter.startShootingSequence(); // start shooting
                    pathState = 101; // goes to shooting sequence
                    pathTimer.reset();
                }
                break;

            /// INTAKE + SHOOT PRELOADED ARTIFACTS (1)

            case 101: // while shooting set 1, wait until finished shooting
                if (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING)) {
                    follower.followPath(Path2);
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_IN; // turn on intake
                    pathState = 102; // has finished shooting, line up to artifact row 2
                    pathTimer.reset();
                }
                break;
            case 102: // line up to artifact row 2, wait
                if (!followerBusy && pathTimer.seconds() > 3.0) { // wait until lined up to row 2
                    follower.followPath(Path3);
                    pathState = 200; // Continue to Path 2
                    pathTimer.reset();
                }
                break;
            case 200: // intake over row 2, exit when (path finished + pathtimer exceeds max allowed time)
                        // OR (if all chamber colors are filled with a color)
                if (lastFollowerBusyState && !followerBusy) { // just finished path
                    pathTimer.reset();
                }
                if (!followerBusy && pathTimer.seconds() > 3.0) { // delay of 3 seconds, if balls are not intaken by then
                    follower.followPath(Path4); // give up, go to shooting spot
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_STOP;
                    pathState = 201;
                    pathTimer.reset();
                }
                if (sorter.allChambersFull()) { // successfully intaked all 3 balls
                    follower.followPath(Path4);
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_STOP;
                    pathState = 201;
                    pathTimer.reset();
                }
                break;

            /// INTAKE + SHOOT ROW 2 ARTIFACTS (2)

            case 201: // moving back to shooting spot
                if (!followerBusy) { // once we reach shooting spot
                    // initiate shooting set #2
                    sorter.startShootingSequence(); // start shooting
                    pathState = 202; // go to shooting set 2
                    pathTimer.reset();
                }
                break;
            case 202:
                if (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING)) {
                    follower.followPath(Path5);
                    pathState = 300; // has finished shooting, go to intake from classifier
                    pathTimer.reset();
                }
                break;
            case 300:
                if (!followerBusy) {
                    pathState = 301; // has reached the classifier + has opened it, should intake balls now
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_IN; // turn on intake
                    pathTimer.reset();
                }
                break;
            /// INTAKE + SHOOT CLASSIFIER ARTIFACTS (3)
            case 301:
                if (pathTimer.seconds() > 6.7) { // don't wait more than 6.7 seconds to intake
                    follower.followPath(Path6);
                    pathState = 302;
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_STOP;
                    pathTimer.reset();
                }
                if (sorter.allChambersFull()) { // successfully intaked all 3 balls
                    follower.followPath(Path6);
                    pathState = 302;
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_STOP;
                    pathTimer.reset();
                }
                break;
            case 302:
                if (!followerBusy) { // after reaching shooting spot, shoot
                    // initiate shooting set #3
                    sorter.startShootingSequence(); // start shooting set 3
                    pathState = 303;
                    pathTimer.reset();
                }
                break;
            case 303:
                if (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING)) {
                    follower.followPath(Path7);
                    pathState = 400; // has finished shooting, go intake over artifact row 1
                    pathTimer.reset();
                }
                break;
                /// INTAKE + SHOOT ROW 1 ARTIFACTS (4)
            case 400:
                if (!followerBusy) {
                    if (lastFollowerBusyState && !follower.isBusy()) { // just finished path
                        pathTimer.reset();
                    }
                    if (!followerBusy && pathTimer.seconds() > 3.0) { // delay of 3 seconds, if balls are not intaken by then
                        follower.followPath(Path8); // give up, go to shooting spot
                        intake.intakeState = Intake.intakeStateFSM.INTAKE_STOP;
                        pathState = 401;
                        pathTimer.reset();
                    }
                    if (sorter.allChambersFull()) { // successfully intaked all 3 balls
                        follower.followPath(Path8);
                        intake.intakeState = Intake.intakeStateFSM.INTAKE_STOP;
                        pathState = 401;
                        pathTimer.reset();
                    }
                }
                break;
            case 401:
                if (!followerBusy) { // once we reach shooting spot
                    // initiate shooting set #4
                    sorter.startShootingSequence(); // start shooting
                    pathState = 402; // go to shooting set 4
                    pathTimer.reset();
                }
                break;
            case 402:
                if (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING)) {
                    pathState = -1; // has finished shooting, END AUTON
                    turret.stopFlywheel();
                    pathTimer.reset();
                }
                break;
            case -1:
                //end
                break;
        }
        lastFollowerBusyState = followerBusy;
        return pathState;
    }

    private void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(123.355, 123.014),

                                new Pose(83.810, 84.114)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(83.810, 84.114),

                                new Pose(97.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(97.000, 60.000),
                                new Pose(121.066, 60.400),
                                new Pose(124.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setVelocityConstraint(1.0)
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(124.000, 60.000),
                                new Pose(91.530, 60.687),
                                new Pose(84.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 84.000),

                                new Pose(126.555, 61.005)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(34.95))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(132.313, 60.711),

                                new Pose(84.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 84.000),

                                new Pose(129.204, 83.976)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(129.204, 83.976),

                                new Pose(84.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
    }

}
