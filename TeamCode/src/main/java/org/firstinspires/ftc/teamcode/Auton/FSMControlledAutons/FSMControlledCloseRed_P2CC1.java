package org.firstinspires.ftc.teamcode.Auton.FSMControlledAutons;

import com.bylazar.configurables.annotations.Configurable;
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
import org.firstinspires.ftc.teamcode.Subsystems.TurretV2;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "FSMControlledCloseRed---P2CC1", group = "!Autonomous")
public class FSMControlledCloseRed_P2CC1 extends OpMode {
    public static boolean runSubsystemsAlso = true;

    public Follower follower;
    private int pathState;
    private ElapsedTime autoTimer;
    private ElapsedTime pathTimer;

    public static PathChain Path1, Path2, Path3, Path4, PathStepBackFromClassifier, Path5, Path6, Path7, Path8, Path9;

    private Intake intake;
    private Sorter sorter;
    private TurretV2 turret;

    private ElapsedTime telemetryLimiter = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime globalAutonTime = new ElapsedTime();

    private boolean lastFollowerBusyState = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123.1, 123.1, Math.toRadians(36)));

        intake = new Intake(hardwareMap);
        sorter = new Sorter(hardwareMap);
        turret = new TurretV2(hardwareMap, "RED");

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

        turret.currentTrackingMode = TurretV2.TurretTrackingMode.OBELISK_TRACKING;
        sorter.chamberColors[0] = "GREEN";
        sorter.chamberColors[1] = "PURPLE";
        sorter.chamberColors[2] = "PURPLE";
        sorter.setShooterSpeed(1.0);

        telemetryLimiter.reset();
        loopTime.reset();
        globalAutonTime.reset();
    }

    @Override
    public void loop() {
        follower.update();
        /// Update Subsystem Classes here
        intake.updateIntake();
        sorter.updateSorter();
        turret.updateTurret(follower);

        /// Update Global Motif for Sorter
        if (!turret.currentMotif.equals(TurretV2.TurretMOTIF.UNKNOWN)) {
            switch (turret.currentMotif) {
                case PPG:
                    sorter.currentMotif = Sorter.MOTIF.PPG;
                    break;
                case PGP:
                    sorter.currentMotif = Sorter.MOTIF.PGP;
                    break;
                case GPP:
                    sorter.currentMotif = Sorter.MOTIF.GPP;
                    break;
            }
        }

        if (runSubsystemsAlso) {
            pathState = autonomousPathUpdateFull();
        }
        else {
            if (globalAutonTime.seconds() <= 29.3) {
                pathState = autonomousPathUpdateOnlyPathMovements();
            }
            if (globalAutonTime.seconds() > 29.3 && pathState != -101) {
                sorter.resetSorterAtEndOfAuton(pathState);
                pathState = -101; /// signals everything is shutting down
            }
        }

        if (telemetryLimiter.seconds() > 0.5) {
            // === Update Loop Time Tracking
            telemetry.addData("Loop Time (Hz)", 1.0 / loopTime.seconds());
            telemetry.addData("Position", follower.getPose());
            telemetry.addData("PATH STATE", pathState);
            telemetry.addData("Flywheel RPM", turret.getFlywheelRPM());
            telemetry.addData("Target Flywheel RPM", turret.targetRPM);
            telemetry.update();
            telemetryLimiter.reset();
        }
        loopTime.reset();
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
                    follower.setMaxPower(0.4);
                    follower.followPath(Path3);
                    pathTimer.reset();
                }
            case 200:
                if (!follower.isBusy() && pathTimer.seconds() > 3.0) {
                    pathState = 300; // Continue to Path 2
                    follower.setMaxPower(1.0);
                    follower.followPath(Path4);
                    pathTimer.reset();
                }
            case 300:
                if (!follower.isBusy() && pathTimer.seconds() > 3.0) {
                    pathState = 400; // Continue to Path 2
                    follower.setMaxPower(.3);
                    follower.followPath(Path5);
                    pathTimer.reset();
                }
            case 400:
                if (!follower.isBusy() && pathTimer.seconds() > 3.0) {
                    pathState = 500; // Continue to Path 2
                    follower.setMaxPower(1.0);
                    follower.followPath(Path6);
                    pathTimer.reset();
                }
            case 500:
                if (!follower.isBusy() && pathTimer.seconds() > 3.0) {
                    pathState = 600; // Continue to Path 2
                    follower.setMaxPower(0.4);
                    follower.followPath(Path7);
                    pathTimer.reset();
                }
            case 600:
                if (!follower.isBusy() && pathTimer.seconds() > 3.0) {
                    pathState = -1; // Continue to Path 2
                    follower.setMaxPower(1.0);
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
                if (follower.getPathCompletion() > 0.9
                        && turret.flywheelReachedDesiredRPM()
                        && (!turret.currentMotif.equals(TurretV2.TurretMOTIF.UNKNOWN) || pathTimer.seconds() > 2.0)) {
                    sorter.startShootingSequence(); // start shooting
                    pathState = 101; // goes to shooting sequence
                    pathTimer.reset();
                }
                break;

            /// INTAKE + SHOOT PRELOADED ARTIFACTS (1)

            case 101: // while shooting set 1, wait until finished shooting
                if (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING) && !sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOT_ALIGNING)) {
                    follower.followPath(Path2); // go intake over row 2
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_IN; // turn on intake
                    pathState = 200; // has finished shooting, line up to artifact row 2
                    pathTimer.reset();
                }
                break;
            case 200: // intake over row 2, exit when (path finished + pathtimer exceeds max allowed time)
                        // OR (if all chamber colors are filled with a color)
                if (lastFollowerBusyState && !followerBusy) { // just finished path, reset timer to wait for artifacts
                    pathTimer.reset();
                }
                if (follower.getPathCompletion() > 0.35 && follower.getPathCompletion() < 0.7) {
                    follower.setMaxPower(.7);
                }
                if (sorter.allChambersFull() || (!followerBusy && pathTimer.seconds() > 1.0)) { // successfully intaked all 3 balls
                    follower.setMaxPower(1.0);
                    follower.followPath(Path3); // go to shoot row 2
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_STOP;
                    pathState = 201;
                    pathTimer.reset();
                }
                break;

            /// INTAKE + SHOOT ROW 1 ARTIFACTS (2)

            case 201: // moving back to shooting spot
                if (!followerBusy && lastFollowerBusyState) {
                    pathTimer.reset();
                }
                if (!followerBusy && pathTimer.seconds() > 1.0) { // once we reach shooting spot
                    // initiate shooting set #2
                    sorter.startShootingSequence(); // start shooting
                    pathState = 202;
                    pathTimer.reset();
                }
                break;
            case 202:
                if (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING) && !sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOT_ALIGNING)) {
                    // go intake from classifier
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_IN; // turn on intake
                    follower.followPath(Path4);
                    pathState = 203; // go to intaking row 1
                    pathTimer.reset();
                }
                break;
            case 203:
                if (follower.getPathCompletion() > 0.3) {
                    follower.setMaxPower(.3);
                }
                if (lastFollowerBusyState && !followerBusy) {
                    pathTimer.reset();
                }
                if (!followerBusy) {
                    follower.followPath(PathStepBackFromClassifier);
                }
                pathState = 300;
                break;
            case 300: // intake from classifier, exit when (path finished + pathtimer exceeds max allowed time)
                // OR (if all chamber colors are filled with a color)
                if (lastFollowerBusyState && !followerBusy) { // just finished path, reset timer to wait for artifacts
                    pathTimer.reset();
                }
                if (sorter.allChambersFull() || (!followerBusy && pathTimer.seconds() > 2.0)) { // successfully intaked all 3 balls
                    follower.setMaxPower(1.0);
                    follower.followPath(Path5); // go to shooting spot
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_STOP;
                    pathState = 301;
                    pathTimer.reset();
                }
                break;
            case 301:
                if (!followerBusy) { // once we reach shooting spot
                    // initiate shooting set #2
                    sorter.startShootingSequence(); // start shooting
                    pathState = 302;
                    pathTimer.reset();
                }
                break;
            case 302:
                if (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING) && !sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOT_ALIGNING)) {
                    // go intake from row 3
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_IN; // turn on intake
                    /// TODO: change to set max power after certain time period
                    follower.followPath(Path6);
                    pathState = 30201; // go to intaking row 3
                    pathTimer.reset();
                }
                break;
            case 30201:
                if (lastFollowerBusyState && !followerBusy) { // just finished path, reset timer to wait for artifacts
                    pathTimer.reset();
                }
                if (follower.getPathCompletion() > 0.3) {
                    follower.setMaxPower(.3);
                }
                if (!followerBusy) {
                    follower.followPath(PathStepBackFromClassifier);
                }
                pathState = 303;
                break;
            case 303: // intake from classifier, exit when (path finished + pathtimer exceeds max allowed time)
                // OR (if all chamber colors are filled with a color)
                if (lastFollowerBusyState && !followerBusy) { // just finished path, reset timer to wait for artifacts
                    pathTimer.reset();
                }
                if (sorter.allChambersFull() || (!followerBusy && pathTimer.seconds() > 2.0)) { // successfully intaked all 3 balls
                    follower.setMaxPower(1.0);
                    follower.followPath(Path7); // go to shooting spot
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_STOP;
                    pathState = 304;
                    pathTimer.reset();
                }
                break;
            case 304:
                if (!followerBusy) { // once we reach shooting spot
                    // initiate shooting set #2
                    sorter.startShootingSequence(); // start shooting
                    pathState = 305;
                    pathTimer.reset();
                }
                break;
            case 305:
                if (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING) && !sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOT_ALIGNING)) {
                    // go intake from row 3
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_IN; // turn on intake
                    /// TODO: change to set max power after certain time period
                    follower.followPath(Path8);
                    pathState = 400; // go to intaking row 1
                    pathTimer.reset();
                }
                break;
            /// INTAKE + SHOOT ROW 1 ARTIFACTS (3)
            case 400:
                if (follower.getPathCompletion() > 0.1) {
                    follower.setMaxPower(.7);
                }
                if (!followerBusy) {
                    if (lastFollowerBusyState && !follower.isBusy()) { // just finished path
                        pathTimer.reset();
                    }
                    if (sorter.allChambersFull() || (!followerBusy && pathTimer.seconds() > 1.0)) { // delay of 3 seconds, if balls are not intaken by then
                        follower.setMaxPower(1.0);
                        follower.followPath(Path9); // give up, go to shooting spot
                        intake.intakeState = Intake.intakeStateFSM.INTAKE_STOP;
                        pathState = 401;
                        pathTimer.reset();
                    }
                }
                break;
            case 401:
                if (follower.getPathCompletion() > 0.9 && (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING) && !sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOT_ALIGNING))) {
                    sorter.startShootingSequence(); // start shooting
                }
                if (!followerBusy) { // once we reach shooting spot
                    // initiate shooting set #4
                    pathState = 402; // go to shooting set 4
                    pathTimer.reset();
                }
                break;
            case 402:
                if (!followerBusy && (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING) && !sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOT_ALIGNING))) {
                    pathState = -1; // has finished shooting, END AUTON
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
                                new Pose(123.100, 123.100),

                                new Pose(85.000, 75.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.000, 75.000),
                                new Pose(88.000, 50.000),
                                new Pose(124.000, 56.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.000, 56.000),

                                new Pose(85.000, 75.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.000, 75.000),
                                new Pose(94.000, 56.000),
                                new Pose(124.500, 61.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))

                .build();

        PathStepBackFromClassifier = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(124.500, 61.500),
                            new Pose(124.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(25))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.000, 60.000),

                                new Pose(85.000, 75.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.000, 75.000),
                                new Pose(94.000, 56.000),
                                new Pose(124.500, 61.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.000, 61.000),

                                new Pose(85.000, 75.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.000, 75.000),
                                new Pose(94.200, 89.000),
                                new Pose(122.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-10))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(122.000, 84.000),

                                new Pose(87.000, 115.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))

                .build();


    }
}
