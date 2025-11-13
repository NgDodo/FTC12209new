package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto Drive + Shoot 3", group = "Tests")
public class AutoDriveAndShoot3 extends OpMode {

    private Follower follower;
    private Timer timer = new Timer();
    private int state = 0;

    // --- Drivetrain test path ---
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pose1 = new Pose(-96, 0, Math.toRadians(0)); // back 48 inches
    private Path backwardPath;

    // --- Mechanisms ---
    private DcMotorEx m3; // flywheel
    private DcMotor m1, m2, m0; // intakes + sorter
    private Servo s2;
    private CRServo s3;

    // --- Sorter control constants (from TeleOp) ---
    private static final int FULL_ROT = 8192;
    private static final int SLOT = FULL_ROT / 3;
    private static final int OFFSET = (int) (FULL_ROT / 6);
    private int currentChamber = 0;
    private boolean shootingMode = false;
    private boolean sorterMoving = false;
    private int sorterTargetPosition = 0;
    private long sorterStartTime;

    @Override
    public void init() {
        // === Initialize Pedro follower ===
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // === Build simple backward path ===
        backwardPath = new Path(new BezierLine(startPose, pose1));
        backwardPath.setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading());

        // === Init hardware (subset of UnifiedTeleop) ===
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m0 = hardwareMap.get(DcMotor.class, "m0");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");

        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setVelocityPIDFCoefficients(8.0, 0.5, 0.0, 12.5);

        s3.setDirection(DcMotor.Direction.REVERSE);
        s2.setPosition(1.0);

        telemetry.addLine("AutoDriveAndShoot3 Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.resetTimer();
        follower.followPath(backwardPath);
        state = 0;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case 0:
                if (!follower.isBusy()) {
                    // arrived at Pose1 → begin shooting sequence
                    timer.resetTimer();
                    simulateY(); // toggle to shooting
                    state = 1;
                }
                break;

            case 1:
                // set flywheel to 3200 RPM
                setShooterRPM(3200);
                if (timer.getElapsedTimeSeconds() > 1.0) {
                    timer.resetTimer();
                    simulateA(); // shoot #1
                    state = 2;
                }
                break;

            case 2:
                if (timer.getElapsedTimeSeconds() > 2.0) {
                    stopA();                      // retract servo
                    timer.resetTimer();
                    state = 21;                   // wait for servo reset before rotate
                }
                break;

            case 21:
                if (timer.getElapsedTimeSeconds() > .6) {  // <-- delay (600 ms)
                    simulateDpadRight();                    // rotate sorter
                    timer.resetTimer();
                    state = 3;
                }
                break;

            case 3:
                if (!sorterMoving || System.currentTimeMillis() - sorterStartTime > 1000) {
                    simulateA(); // shoot #2
                    timer.resetTimer();
                    state = 4;
                }
                break;

            case 4:
                if (timer.getElapsedTimeSeconds() > 2.0) {
                    stopA();
                    timer.resetTimer();
                    state = 41;   // delay before next sorter move
                }
                break;

            case 41:
                if (timer.getElapsedTimeSeconds() > 0.6) {
                    simulateDpadRight();
                    timer.resetTimer();
                    state = 5;
                }
                break;

            case 5:
                if (!sorterMoving || System.currentTimeMillis() - sorterStartTime > 1000) {
                    simulateA(); // shoot #3
                    timer.resetTimer();
                    state = 6;
                }
                break;

            case 6:
                if (timer.getElapsedTimeSeconds() > 2.0) {
                    stopA();
                    setShooterRPM(0); // left bumper = off
                    simulateY();      // back to intake
                    state = 7;
                }
                break;

            case 7:
                // done
                break;
        }

        updateSorter();
        telemetry.addData("State", state);
        telemetry.addData("RPM", (m3.getVelocity() / 28.0) * 60.0);
        telemetry.addData("Sorter Target", sorterTargetPosition);
        telemetry.addData("Sorter Pos", normalize(m2.getCurrentPosition()));
        telemetry.update();
    }

    // === Simulated button actions ===

    private void simulateY() {
        shootingMode = !shootingMode;
        int target = getChamberPosition(currentChamber, shootingMode);
        startSorterMove(target);
    }

    private void simulateDpadRight() {
        currentChamber = nextChamber(currentChamber);
        int target = getChamberPosition(currentChamber, shootingMode);
        startSorterMove(target);
    }

    private void simulateA() {
        s2.setPosition(0);
        s3.setPower(1.0);
    }

    private void stopA() {
        s2.setPosition(1.0);
        s3.setPower(0.0);
    }

    private void setShooterRPM(double rpm) {
        double ticksPerSec = (rpm / 60.0) * 28.0;
        m3.setVelocity(ticksPerSec);
    }

    // === Sorter control (copied from TeleOp, simplified) ===

    private void startSorterMove(int target) {
        sorterTargetPosition = target;
        sorterMoving = true;
        sorterStartTime = System.currentTimeMillis();
    }

    private void updateSorter() {
        if (!sorterMoving) return;
        int pos = normalize(m2.getCurrentPosition());
        int error = calculateShortestError(pos, sorterTargetPosition);
        double power = Math.signum(error) * 0.4;
        m0.setPower(power);
        if (Math.abs(error) < 160) {
            m0.setPower(0);
            sorterMoving = false;
        }
    }

    private int getChamberPosition(int chamber, boolean shooting) {
        int base;
        switch (chamber) {
            case 0: base = 0; break;
            case 1: base = SLOT; break;
            case 2: base = 2 * SLOT; break;
            default: base = 0;
        }
        if (shooting) base = normalize(base + OFFSET);
        return base;
    }

    private int nextChamber(int c) {
        if (c == 1) return 0;
        if (c == 0) return 2;
        return 1;
    }

    private int normalize(int ticks) {
        return ((ticks % FULL_ROT) + FULL_ROT) % FULL_ROT;
    }

    private int calculateShortestError(int current, int target) {
        int error = target - current;
        if (error > FULL_ROT / 2) error -= FULL_ROT;
        if (error < -FULL_ROT / 2) error += FULL_ROT;
        return error;
    }
}
