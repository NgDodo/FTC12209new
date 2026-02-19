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
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "FSMControlledFarRed---4Lx3", group = "!Autonomous")
public class FSMControlledFarRed_4Lx3 extends OpMode {
    public static boolean runSubsystemsAlso = true;

    public Follower follower;
    private int pathState;
    private ElapsedTime autoTimer;
    private ElapsedTime pathTimer;

    private PathChain Path1OffsetToShootPreloads, Path2IntakeRow3, Path3ShootRow3, Path4IntakeRow4, Path5ShootRow4, Path6Leave;

    private Intake intake;
    private Sorter sorter;
    private Turret turret;

    private ElapsedTime telemetryLimiter = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime globalAutonTime = new ElapsedTime();

    private boolean lastFollowerBusyState = false;
    public static double shootingTimeForFar = 1.5;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(87.5, 9, Math.toRadians(0)));

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

        follower.followPath(Path1OffsetToShootPreloads);

        turret.setFlywheelRPM("FAR"); // rev up flywheel
        turret.currentTrackingMode = Turret.TurretTrackingMode.OBELISK_TRACKING;

        sorter.chamberColors[0] = "GREEN";
        sorter.chamberColors[1] = "PURPLE";
        sorter.chamberColors[2] = "PURPLE";
        sorter.setShootingConstants(shootingTimeForFar, 0.2, 0.15, 0.0);

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
        if (!turret.currentMotif.equals(Turret.TurretMOTIF.UNKNOWN)) {
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
            if (globalAutonTime.seconds() <= 29.5) {
                pathState = autonomousPathUpdateFull();
            }
            if (globalAutonTime.seconds() > 29.5 && pathState != -101) {
                sorter.resetSorterAtEndOfAuton(pathState);
                pathState = -101; /// signals everything is shutting down
            }
        }

        if (telemetryLimiter.seconds() > 0.5) {
            // === Update Loop Time Tracking
            telemetry.addData("Loop Time (Hz)", 1.0 / loopTime.seconds());
            telemetry.addData("Position", follower.getPose());
            telemetry.addData("PATH STATE", pathState);
            telemetry.addLine("-------------------------");
            telemetry.addData("Flywheel RPM", turret.getFlywheelRPM());
            telemetry.update();
            telemetryLimiter.reset();
        }
        loopTime.reset();
    }

    public int autonomousPathUpdateFull() {
        boolean followerBusy = follower.isBusy();
        switch (pathState) {
            case 0: // Wait until shoot
                if (!followerBusy
                        && turret.flywheelReachedDesiredRPM()
                        && (!turret.currentMotif.equals(Turret.TurretMOTIF.UNKNOWN) || pathTimer.seconds() > 5.0)) {
                    sorter.startShootingSequence(); // start shooting
                    pathState = 100; // goes to shooting sequence
                    pathTimer.reset();
                }
                break;

            /// INTAKE + SHOOT Row 3 Artifacts (1)

            case 100: // while shooting set 1, wait until finished shooting
                if (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING)) {
                    follower.followPath(Path2IntakeRow3); // go intake over row 1
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_IN; // turn on intake
                    pathState = 101; // has finished shooting, line up to artifact row 2
                    pathTimer.reset();
                }
                break;
            case 101:
                if (lastFollowerBusyState && !followerBusy) { // just finished path, reset timer to wait for artifacts
                    pathTimer.reset();
                }
                if (follower.getPathCompletion() > 0.35 && follower.getPathCompletion() < 0.7) {
                    follower.setMaxPower(0.45);
                }
                if (sorter.allChambersFull() || (!followerBusy && pathTimer.seconds() > 1.0)) {
                    follower.setMaxPower(1.0);
                    follower.followPath(Path3ShootRow3); // go to shoot row 3
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_STOP;
                    pathState = 102;
                }
                break;
            case 102:
                if (!followerBusy && turret.flywheelReachedDesiredRPM()) {
                    sorter.startShootingSequence();
                    pathState = 200; // goes to intaking row 4 sequence
                    pathTimer.reset();
                }
                break;
            /// 200s --> Intake + Shoot Row 4
            case 200:
                if (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING)) {
                    follower.followPath(Path4IntakeRow4); // go intake over row 1
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_IN; // turn on intake
                    pathState = 201; // has finished shooting, line up to artifact row 2
                    pathTimer.reset();
                }
                break;
            case 201:
                if (lastFollowerBusyState && !followerBusy) { // just finished path, reset timer to wait for artifacts
                    pathTimer.reset();
                }
                if (follower.getPathCompletion() > 0.35 && follower.getPathCompletion() < 0.7) {
                    follower.setMaxPower(0.45);
                }
                if (sorter.allChambersFull() || (!followerBusy && pathTimer.seconds() > 2.0)) {
                    follower.setMaxPower(1.0);
                    follower.followPath(Path5ShootRow4); // go to shoot row 4
                    intake.intakeState = Intake.intakeStateFSM.INTAKE_STOP;
                    pathState = 202;
                }
                break;
            case 202:
                if (!followerBusy && turret.flywheelReachedDesiredRPM()) {
                    sorter.startShootingSequence();
                    pathState = 203; // goes to intaking row 4 sequence
                    pathTimer.reset();
                }
                break;
            case 203:
                if (!sorter.sorterState.equals(Sorter.sorterStateFSM.SHOOTING)) {
                    follower.followPath(Path6Leave); // go intake over row 1
                    pathState = -1; // has finished shooting, line up to artifact row 2
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
        Path1OffsetToShootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.500, 9.000),

                                new Pose(83.500, 24.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path2IntakeRow3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(83.500, 24.500),
                                new Pose(88.232, 38.517),
                                new Pose(128.000, 35.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path3ShootRow3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.000, 35.000),

                                new Pose(87.500, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path4IntakeRow4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.500, 12.000),

                                new Pose(134.000, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path5ShootRow4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.000, 9.000),

                                new Pose(87.500, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path6Leave = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.500, 12.000),

                                new Pose(105.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
    }
}
