package org.firstinspires.ftc.teamcode.DriveTrainControl.TuneSubsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple PID Tuner for Sorter Mechanism
 *
 * Use FTC Dashboard to adjust PID values in real-time:
 * - Connect to http://192.168.43.1:8080/dash (or your robot's IP)
 * - Adjust kP, kI, kD values in the dashboard
 * - Press A/B/X to test different chamber positions
 * - Press Y to toggle shooting mode
 *
 * CONTROLS:
 * A: Chamber 0 (0°)
 * B: Chamber 1 (120°)
 * X: Chamber 2 (240°)
 * Y: Toggle Shooting Mode (+180° offset)
 */
@Config
@Configurable
@TeleOp(name = "Sorter PID Tuner (Dashboard)", group = "Tuning")
public class tuneSorterPID extends OpMode {

    // === PID GAINS (Editable in FTC Dashboard) ===
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.0001;

    // === Hardware ===
    private DcMotorEx sorterMotor;
    private DcMotorEx backRightMotor;

    // === Sorter Constants ===
    private static final int FULL_ROT = 8192;
    private static final int SLOT = FULL_ROT / 3;
    private static final int OFFSET = FULL_ROT / 2;

    private static final int CHAMBER_0_POS = 0;
    private static final int CHAMBER_1_POS = SLOT;
    private static final int CHAMBER_2_POS = 2 * SLOT;

    // === PID State ===
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0;

    // === Target Control ===
    private int targetPosition = 0;
    private int currentChamber = 0;
    private boolean shootingMode = false;

    // === Button States ===
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    // === Performance Tracking ===
    private ElapsedTime moveTimer = new ElapsedTime();

    @Override
    public void init() {
        // Setup FTC Dashboard telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize hardware
        sorterMotor = hardwareMap.get(DcMotorEx.class, "m0");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "bR");

        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = normalize(backRightMotor.getCurrentPosition());
        lastTime = System.nanoTime();

        telemetry.addLine("Sorter PID Tuner Ready");
        telemetry.addLine("Connect to FTC Dashboard to adjust PID values");
        telemetry.update();
    }

    @Override
    public void start() {
        moveTimer.reset();
        lastTime = System.nanoTime();
    }

    @Override
    public void loop() {
        // Handle target selection
        handleTargetSelection();

        // Get current position
        int currentPosition = normalize(backRightMotor.getCurrentPosition());

        // Calculate and apply PID
        double power = calculatePID(currentPosition, targetPosition);
        sorterMotor.setPower(power);

        // Update telemetry
        updateTelemetry(currentPosition, power);
    }

    /**
     * Handle chamber target selection with buttons
     */
    private void handleTargetSelection() {
        boolean targetChanged = false;

        if (gamepad1.a && !lastA) {
            currentChamber = 0;
            targetChanged = true;
        }
        if (gamepad1.b && !lastB) {
            currentChamber = 1;
            targetChanged = true;
        }
        if (gamepad1.x && !lastX) {
            currentChamber = 2;
            targetChanged = true;
        }
        if (gamepad1.y && !lastY) {
            shootingMode = !shootingMode;
            targetChanged = true;
        }

        if (targetChanged) {
            targetPosition = getChamberPosition(currentChamber, shootingMode);
            integral = 0; // Reset integral on new target
            moveTimer.reset();
        }

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
    }

    /**
     * Calculate PID control output
     */
    private double calculatePID(int current, int target) {
        // Calculate time delta
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }

        // Calculate error (shortest path)
        int error = calculateShortestError(current, target);

        // PID terms
        double pTerm = kP * error;

        integral += error * dt;
        integral = Math.max(-5000, Math.min(5000, integral)); // Anti-windup
        double iTerm = kI * integral;

        double derivative = (error - lastError) / dt;
        double dTerm = kD * derivative;

        lastError = error;

        // Total output
        double output = pTerm + iTerm + dTerm;

        // Clamp output
        output = Math.max(-1.0, Math.min(1.0, output));

        return output;
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry(int currentPosition, double power) {
        int error = calculateShortestError(currentPosition, targetPosition);

        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addLine();

        telemetry.addData("Current Position", "%d (%.1f°)", currentPosition, ticksToDegrees(currentPosition));
        telemetry.addData("Target Position", "%d (%.1f°)", targetPosition, ticksToDegrees(targetPosition));
        telemetry.addData("Error", "%d ticks (%.1f°)", error, ticksToDegrees(error));
        telemetry.addLine();

        telemetry.addData("Power", "%.3f", power);
        telemetry.addData("Integral", "%.1f", integral);
        telemetry.addLine();

        telemetry.addData("Chamber", currentChamber);
        telemetry.addData("Mode", shootingMode ? "SHOOTING" : "INTAKE");
        telemetry.addData("Move Time", "%.2f s", moveTimer.seconds());
        telemetry.addLine();

        telemetry.addLine("Controls: A/B/X = Chamber 0/1/2, Y = Toggle Mode");

        telemetry.update();
    }

    // === Utility Functions ===

    private int getChamberPosition(int chamber, boolean shooting) {
        int basePos;
        switch(chamber) {
            case 0: basePos = CHAMBER_0_POS; break;
            case 1: basePos = CHAMBER_1_POS; break;
            case 2: basePos = CHAMBER_2_POS; break;
            default: basePos = CHAMBER_0_POS;
        }

        if (shooting) {
            basePos = normalize(basePos + OFFSET);
        }

        return basePos;
    }

    private int normalize(int ticks) {
        return ((ticks % FULL_ROT) + FULL_ROT) % FULL_ROT;
    }

    private int calculateShortestError(int current, int target) {
        int error = target - current;

        if (error > FULL_ROT / 2) {
            error -= FULL_ROT;
        } else if (error < -FULL_ROT / 2) {
            error += FULL_ROT;
        }

        return error;
    }

    private double ticksToDegrees(int ticks) {
        return (ticks * 360.0) / FULL_ROT;
    }

    @Override
    public void stop() {
        sorterMotor.setPower(0);
    }
}