package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel PID Tuning", group = "Test")
public class FlywheelPIDTuning extends OpMode {

    private DcMotorEx flywheelMotor;

    // Target RPM
    private double targetRPM = 0;
    private static final double TICKS_PER_REV = 28.0;

    // PID Coefficients - TUNE THESE
    private double kP = 0.001;
    private double kI = 0.00001;
    private double kD = 0.0;
    private double kF = 0.00025; // Feedforward

    // PID variables
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;

    // Adjustment increments
    private static final double LARGE_INCREMENT = 0.00001;
    private static final double SMALL_INCREMENT = 0.000001;
    private static final double RPM_INCREMENT = 2500;

    // Button states
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    // Current parameter being tuned
    private enum TuneParam { KP, KI, KD, KF, TARGET_RPM }
    private TuneParam currentParam = TuneParam.KP;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "m3");
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastTime = System.nanoTime();

        telemetry.addLine("Flywheel PID Tuning");
        telemetry.addLine("===================");
        telemetry.addLine("A/B/X/Y: Select parameter");
        telemetry.addLine("DpadUp/Down: Adjust (large)");
        telemetry.addLine("DpadLeft/Right: Adjust (small)");
        telemetry.addLine("RB/LB: Target RPM ±100");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Parameter selection
        if (gamepad1.a && !lastA) currentParam = TuneParam.KP;
        if (gamepad1.b && !lastB) currentParam = TuneParam.KI;
        if (gamepad1.x && !lastX) currentParam = TuneParam.KD;
        if (gamepad1.y && !lastY) currentParam = TuneParam.KF;

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;

        // Adjust target RPM
        if (gamepad1.right_bumper && !lastRightBumper) {
            targetRPM += RPM_INCREMENT;
        }
        if (gamepad1.left_bumper && !lastLeftBumper) {
            targetRPM -= RPM_INCREMENT;
            if (targetRPM < 0) targetRPM = 0;
        }
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        // Adjust current parameter
        if (gamepad1.dpad_up && !lastDpadUp) {
            adjustParameter(LARGE_INCREMENT);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            adjustParameter(-LARGE_INCREMENT);
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            adjustParameter(SMALL_INCREMENT);
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            adjustParameter(-SMALL_INCREMENT);
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        // Calculate current RPM
        double currentVelocity = flywheelMotor.getVelocity();
        double currentRPM = (currentVelocity / TICKS_PER_REV) * 60.0;

        // PID Control
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9; // Convert to seconds

        double error = targetRPM - currentRPM;

        integral += error * dt;
        // Anti-windup
        integral = Math.max(-10000, Math.min(10000, integral));

        double derivative = (error - lastError) / dt;

        // Feedforward term (estimated power needed for target RPM)
        double feedforward = kF * targetRPM;

        // PID output
        double pidOutput = (kP * error) + (kI * integral) + (kD * derivative) + feedforward;

        // Clamp output to [-1, 1]
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        // Apply power to motor
        flywheelMotor.setPower(pidOutput);

        lastError = error;
        lastTime = currentTime;

        // Telemetry
        telemetry.addLine("=== FLYWHEEL PID TUNING ===");
        telemetry.addLine();
        telemetry.addData("Target RPM", String.format("%.0f (RB/LB ±100)", targetRPM));
        telemetry.addData("Current RPM", String.format("%.0f", currentRPM));
        telemetry.addData("Error", String.format("%.0f RPM", error));
        telemetry.addData("Motor Power", String.format("%.3f", pidOutput));
        telemetry.addLine();

        telemetry.addLine("=== PID PARAMETERS ===");
        telemetry.addData("kP (A)", String.format("%s%.6f", currentParam == TuneParam.KP ? ">>> " : "    ", kP));
        telemetry.addData("kI (B)", String.format("%s%.6f", currentParam == TuneParam.KI ? ">>> " : "    ", kI));
        telemetry.addData("kD (X)", String.format("%s%.6f", currentParam == TuneParam.KD ? ">>> " : "    ", kD));
        telemetry.addData("kF (Y)", String.format("%s%.6f", currentParam == TuneParam.KF ? ">>> " : "    ", kF));
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A/B/X/Y: Select kP/kI/kD/kF");
        telemetry.addLine("DpadUp/Down: Large adjust (±0.00001)");
        telemetry.addLine("DpadLeft/Right: Small adjust (±0.000001)");
        telemetry.addLine("RB/LB: Target RPM");

        telemetry.update();
    }

    private void adjustParameter(double amount) {
        switch (currentParam) {
            case KP:
                kP += amount;
                kP = Math.max(0, kP);
                break;
            case KI:
                kI += amount;
                kI = Math.max(0, kI);
                break;
            case KD:
                kD += amount;
                kD = Math.max(0, kD);
                break;
            case KF:
                kF += amount;
                kF = Math.max(0, kF);
                break;
        }
    }

    @Override
    public void stop() {
        flywheelMotor.setPower(0);
    }
}