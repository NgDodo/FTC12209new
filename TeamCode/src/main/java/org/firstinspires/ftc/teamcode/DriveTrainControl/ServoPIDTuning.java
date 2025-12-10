package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo PID Tuning", group = "Test")
public class ServoPIDTuning extends OpMode {

    private Servo servo;

    // Target position (0.0 to 1.0)
    private double targetPosition = 0.5;

    // PID Coefficients - TUNE THESE
    private double kP = 1.0;
    private double kI = 0.0;
    private double kD = 0.0;

    // PID variables
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;

    // Current position tracking (you'll need an encoder or sensor)
    // For now, we'll assume the servo reaches the position we set
    private double currentPosition = 0.5;
    private double commandedPosition = 0.5;

    // Adjustment increments
    private static final double LARGE_INCREMENT = 0.1;
    private static final double SMALL_INCREMENT = 0.01;
    private static final double POSITION_INCREMENT = 0.05;

    // Button states
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightTrigger = false;
    private boolean lastLeftTrigger = false;

    // Current parameter being tuned
    private enum TuneParam { KP, KI, KD }
    private TuneParam currentParam = TuneParam.KP;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "s1");
        servo.setPosition(0.5);
        commandedPosition = 0.5;
        currentPosition = 0.5;

        lastTime = System.nanoTime();

        telemetry.addLine("Servo PID Tuning");
        telemetry.addLine("===================");
        telemetry.addLine("NOTE: Standard servos don't have position");
        telemetry.addLine("feedback. This simulates PID control.");
        telemetry.addLine();
        telemetry.addLine("A/B/X: Select kP/kI/kD");
        telemetry.addLine("DpadUp/Down: Large adjust");
        telemetry.addLine("DpadLeft/Right: Small adjust");
        telemetry.addLine("RB/LB: Target position ±0.05");
        telemetry.addLine("RT/LT: Preset positions");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Parameter selection
        if (gamepad1.a && !lastA) currentParam = TuneParam.KP;
        if (gamepad1.b && !lastB) currentParam = TuneParam.KI;
        if (gamepad1.x && !lastX) currentParam = TuneParam.KD;

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;

        // Adjust target position
        if (gamepad1.right_bumper && !lastRightBumper) {
            targetPosition += POSITION_INCREMENT;
            targetPosition = Math.min(1.0, targetPosition);
            // Reset PID when target changes
            integral = 0;
            lastError = 0;
        }
        if (gamepad1.left_bumper && !lastLeftBumper) {
            targetPosition -= POSITION_INCREMENT;
            targetPosition = Math.max(0.0, targetPosition);
            // Reset PID when target changes
            integral = 0;
            lastError = 0;
        }
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        // Preset positions
        if (gamepad1.right_trigger > 0.5 && !lastRightTrigger) {
            targetPosition = 1.0;
            integral = 0;
            lastError = 0;
        }
        if (gamepad1.left_trigger > 0.5 && !lastLeftTrigger) {
            targetPosition = 0.0;
            integral = 0;
            lastError = 0;
        }
        lastRightTrigger = gamepad1.right_trigger > 0.5;
        lastLeftTrigger = gamepad1.left_trigger > 0.5;

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

        // Simulate current position (in reality, you'd read from an encoder)
        // For standard servos, we assume they move toward the commanded position
        double positionDiff = commandedPosition - currentPosition;
        currentPosition += positionDiff * 0.1; // Simulated servo movement

        // PID Control
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9; // Convert to seconds

        double error = targetPosition - currentPosition;

        integral += error * dt;
        // Anti-windup
        integral = Math.max(-1.0, Math.min(1.0, integral));

        double derivative = (error - lastError) / dt;

        // PID output
        double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp output to [0, 1] for servo position
        pidOutput = Math.max(0.0, Math.min(1.0, pidOutput));

        // Apply position to servo
        commandedPosition = pidOutput;
        servo.setPosition(commandedPosition);

        lastError = error;
        lastTime = currentTime;

        // Telemetry
        telemetry.addLine("=== SERVO PID TUNING ===");
        telemetry.addLine();
        telemetry.addData("Target Position", String.format("%.3f (RB/LB)", targetPosition));
        telemetry.addData("Current Position", String.format("%.3f (simulated)", currentPosition));
        telemetry.addData("Commanded Position", String.format("%.3f", commandedPosition));
        telemetry.addData("Error", String.format("%.3f", error));
        telemetry.addLine();

        telemetry.addLine("=== PID PARAMETERS ===");
        telemetry.addData("kP (A)", String.format("%s%.3f", currentParam == TuneParam.KP ? ">>> " : "    ", kP));
        telemetry.addData("kI (B)", String.format("%s%.3f", currentParam == TuneParam.KI ? ">>> " : "    ", kI));
        telemetry.addData("kD (X)", String.format("%s%.3f", currentParam == TuneParam.KD ? ">>> " : "    ", kD));
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A/B/X: Select kP/kI/kD");
        telemetry.addLine("DpadUp/Down: Large (±0.1)");
        telemetry.addLine("DpadLeft/Right: Small (±0.01)");
        telemetry.addLine("RB/LB: Target ±0.05");
        telemetry.addLine("RT: Position 1.0 | LT: Position 0.0");

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
        }
    }

    @Override
    public void stop() {
        servo.setPosition(0.5);
    }
}