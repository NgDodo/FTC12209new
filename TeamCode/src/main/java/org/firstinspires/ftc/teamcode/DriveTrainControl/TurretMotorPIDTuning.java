package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@TeleOp(name = "Turret Motor PID Tuning", group = "Test")
public class TurretMotorPIDTuning extends OpMode {

    // === Hardware ===
    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    // === Drive Train ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // === Turret Configuration ===
    private static final double TURRET_RANGE_DEG = 340;
    private static final double TICKS_PER_REV = 1393.1;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;

    private static final int CENTER_POSITION = 0;
    private static final int MAX_POSITION = (int)((TURRET_RANGE_DEG / 2.0) * TICKS_PER_DEGREE);
    private static final int MIN_POSITION = -MAX_POSITION;

    // === Limelight Configuration ===
    private static final String LIMELIGHT_NAME = "Webcam 2";
    private static final int APRILTAG_PIPELINE = 1;

    // === PID Coefficients ===
    private double kP = 0.014;
    private double kI = 0.002;
    private double kD = 0.0005;

    // === Feedforward for robot rotation ===
    private double kFF = 0.4;  // How much to compensate for robot rotation

    // === PID variables ===
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;

    // === Tracking Control ===
    private boolean trackingEnabled = false;
    private boolean lastAButton = false;
    private int manualTargetPosition = CENTER_POSITION;

    // === Smoothing ===
    private static final double SMOOTHING_ALPHA = 0.65;
    private double smoothedBearing = 0.0;
    private boolean bearingInitialized = false;

    // === Prediction ===
    private static final boolean USE_PREDICTION = true;
    private double lastValidBearing = 0.0;
    private double bearingVelocity = 0.0;
    private long lastValidTime = 0;
    private static final long MAX_PREDICTION_MS = 500;

    // === Adjustment increments ===
    private static final double LARGE_INCREMENT = 0.001;
    private static final double SMALL_INCREMENT = 0.0001;
    private static final int MANUAL_POSITION_INCREMENT = 50;
    private static final double FF_INCREMENT = 0.05;

    // === Button states ===
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastX = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftTrigger = false;
    private boolean lastRightTrigger = false;

    // === Current parameter ===
    private enum TuneParam { KP, KI, KD, KFF }
    private TuneParam currentParam = TuneParam.KP;

    // === Power limits ===
    private static final double MAX_POWER = 0.85;
    private static final double MIN_POWER = 0.06;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "m2");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "fL");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "bL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "bR");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lastTime = System.nanoTime();

        telemetry.addLine("Turret Motor PID Tuning");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a && !lastAButton) {
            trackingEnabled = !trackingEnabled;
            if (trackingEnabled) {
                bearingInitialized = false;
                integral = 0;
                lastError = 0;
                bearingVelocity = 0;
            } else {
                turretMotor.setPower(0);
            }
        }
        lastAButton = gamepad1.a;

        // Parameter selection - added left/right trigger for kFF
        if (gamepad1.x && !lastX) currentParam = TuneParam.KP;
        if (gamepad1.b && !lastB) currentParam = TuneParam.KI;
        if (gamepad1.y && !lastY) currentParam = TuneParam.KD;
        if (gamepad1.left_trigger > 0.5 && !lastLeftTrigger) currentParam = TuneParam.KFF;
        lastX = gamepad1.x;
        lastB = gamepad1.b;
        lastY = gamepad1.y;
        lastLeftTrigger = gamepad1.left_trigger > 0.5;

        if (gamepad1.dpad_up && !lastDpadUp) adjustParameter(LARGE_INCREMENT);
        if (gamepad1.dpad_down && !lastDpadDown) adjustParameter(-LARGE_INCREMENT);
        if (gamepad1.dpad_right && !lastDpadRight) adjustParameter(SMALL_INCREMENT);
        if (gamepad1.dpad_left && !lastDpadLeft) adjustParameter(-SMALL_INCREMENT);
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        // Get robot rotation input for feedforward
        double robotRotation = applyDeadzone(gamepad1.right_stick_x);

        if (!trackingEnabled) {
            if (gamepad1.right_bumper && !lastRightBumper) {
                manualTargetPosition += MANUAL_POSITION_INCREMENT;
                manualTargetPosition = Math.min(MAX_POSITION, manualTargetPosition);
            }
            if (gamepad1.left_bumper && !lastLeftBumper) {
                manualTargetPosition -= MANUAL_POSITION_INCREMENT;
                manualTargetPosition = Math.max(MIN_POSITION, manualTargetPosition);
            }

            int currentPosition = turretMotor.getCurrentPosition();
            int error = manualTargetPosition - currentPosition;
            double power = Math.signum(error) * Math.min(Math.abs(error) * 0.01, MAX_POWER);
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

            if (Math.abs(error) < 10) power = 0;
            turretMotor.setPower(power);
        }
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        if (trackingEnabled) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                LLResultTypes.FiducialResult validTarget = null;
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId();
                    if (id == 20 || id == 24) {
                        validTarget = fiducial;
                        break;
                    }
                }

                if (validTarget != null) {
                    double bearing = validTarget.getTargetXDegrees();

                    if (bearingInitialized) {
                        double timeDiff = (System.currentTimeMillis() - lastValidTime) / 1000.0;
                        if (timeDiff > 0 && timeDiff < 0.5) {
                            bearingVelocity = (bearing - lastValidBearing) / timeDiff;
                            bearingVelocity = Math.max(-200, Math.min(200, bearingVelocity));
                        }
                    }

                    lastValidBearing = bearing;
                    lastValidTime = System.currentTimeMillis();

                    if (!bearingInitialized) {
                        smoothedBearing = bearing;
                        bearingInitialized = true;
                    } else {
                        smoothedBearing = SMOOTHING_ALPHA * bearing + (1.0 - SMOOTHING_ALPHA) * smoothedBearing;
                    }

                    long currentTime = System.nanoTime();
                    double dt = (currentTime - lastTime) / 1e9;
                    if (dt <= 0) dt = 1e-6;

                    double error = smoothedBearing;
                    integral += error * dt;
                    integral = Math.max(-1000.0, Math.min(1000.0, integral));

                    double derivative = (error - lastError) / dt;

                    // Calculate PID output
                    double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);

                    // Add feedforward based on robot rotation
                    // Negative because turret rotates opposite to robot to maintain target
                    double feedforward = -robotRotation * kFF;

                    double totalOutput = pidOutput + feedforward;
                    totalOutput = Math.max(-MAX_POWER, Math.min(MAX_POWER, totalOutput));

                    if (Math.abs(totalOutput) > 0 && Math.abs(totalOutput) < MIN_POWER) {
                        totalOutput = Math.copySign(MIN_POWER, totalOutput);
                    }

                    if (Math.abs(error) < 0.8 && Math.abs(robotRotation) < 0.1) {
                        totalOutput = feedforward;  // Keep feedforward active
                        integral = 0;
                    }

                    int currentPosition = turretMotor.getCurrentPosition();
                    if (currentPosition >= MAX_POSITION && totalOutput > 0) {
                        totalOutput = 0;
                    } else if (currentPosition <= MIN_POSITION && totalOutput < 0) {
                        totalOutput = 0;
                    }

                    turretMotor.setPower(totalOutput);
                    lastError = error;
                    lastTime = currentTime;

                    int currentPos = turretMotor.getCurrentPosition();
                    double currentDegrees = currentPos / TICKS_PER_DEGREE;

                    telemetry.addLine("=== TRACKING ACTIVE ===");
                    telemetry.addData("Tag ID", validTarget.getFiducialId());
                    telemetry.addData("Raw Bearing (tx)", String.format("%.2f°", bearing));
                    telemetry.addData("Smoothed Bearing", String.format("%.2f°", smoothedBearing));
                    telemetry.addData("Bearing Velocity", String.format("%.1f°/s", bearingVelocity));
                    telemetry.addData("Error", String.format("%.2f°", error));
                    telemetry.addData("PID Output", String.format("%.3f", pidOutput));
                    telemetry.addData("Feedforward", String.format("%.3f", feedforward));
                    telemetry.addData("Total Power", String.format("%.3f", totalOutput));
                    telemetry.addData("Motor Position", String.format("%d ticks (%.1f°)", currentPos, currentDegrees));
                } else {
                    telemetry.addLine("=== NO VALID TAG (20/24) ===");
                    telemetry.addData("Tags Detected", fiducials.size());
                    if (!fiducials.isEmpty()) {
                        for (LLResultTypes.FiducialResult f : fiducials) {
                            telemetry.addData("  Tag", f.getFiducialId());
                        }
                    }
                }
            } else {
                long timeSinceValid = System.currentTimeMillis() - lastValidTime;

                if (USE_PREDICTION && timeSinceValid < MAX_PREDICTION_MS && bearingInitialized) {
                    double timeSec = timeSinceValid / 1000.0;
                    double predictedBearing = lastValidBearing + (bearingVelocity * timeSec);

                    double decayFactor = 1.0 - (timeSec / (MAX_PREDICTION_MS / 1000.0));
                    decayFactor = Math.max(0.3, decayFactor);

                    double error = predictedBearing * decayFactor;

                    long currentTime = System.nanoTime();
                    double dt = (currentTime - lastTime) / 1e9;
                    if (dt <= 0) dt = 1e-6;

                    double pidOutput = (kP * error * 0.8) + (kD * (error - lastError) / dt * 0.5);

                    // CRITICAL: Keep feedforward active during prediction
                    double feedforward = -robotRotation * kFF;

                    double totalOutput = pidOutput + feedforward;
                    totalOutput = Math.max(-MAX_POWER, Math.min(MAX_POWER, totalOutput));

                    if (Math.abs(totalOutput) > 0 && Math.abs(totalOutput) < MIN_POWER) {
                        totalOutput = Math.copySign(MIN_POWER, totalOutput);
                    }

                    int currentPosition = turretMotor.getCurrentPosition();
                    if (currentPosition >= MAX_POSITION && totalOutput > 0) {
                        totalOutput = 0;
                    } else if (currentPosition <= MIN_POSITION && totalOutput < 0) {
                        totalOutput = 0;
                    }

                    turretMotor.setPower(totalOutput);
                    lastError = error;
                    lastTime = currentTime;

                    telemetry.addLine("=== PREDICTION MODE ===");
                    telemetry.addData("Time Since Valid", String.format("%d ms", timeSinceValid));
                    telemetry.addData("Predicted Bearing", String.format("%.2f°", predictedBearing));
                    telemetry.addData("Decay Factor", String.format("%.2f", decayFactor));
                    telemetry.addData("Feedforward", String.format("%.3f", feedforward));
                    telemetry.addData("Total Power", String.format("%.3f", totalOutput));
                } else {
                    // Even when stopped, apply feedforward if rotating
                    double feedforward = -robotRotation * kFF;

                    int currentPosition = turretMotor.getCurrentPosition();
                    if (currentPosition >= MAX_POSITION && feedforward > 0) {
                        feedforward = 0;
                    } else if (currentPosition <= MIN_POSITION && feedforward < 0) {
                        feedforward = 0;
                    }

                    turretMotor.setPower(feedforward);
                    integral = 0;
                    telemetry.addLine("=== NO TAG VISIBLE ===");
                    telemetry.addData("Feedforward Active", Math.abs(feedforward) > 0.01);
                }
            }
        } else {
            int currentPos = turretMotor.getCurrentPosition();
            double currentDegrees = currentPos / TICKS_PER_DEGREE;

            telemetry.addLine("=== MANUAL MODE ===");
            telemetry.addData("Target", String.format("%d ticks (%.1f°)", manualTargetPosition, manualTargetPosition / TICKS_PER_DEGREE));
            telemetry.addData("Current", String.format("%d ticks (%.1f°)", currentPos, currentDegrees));
        }

        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
        double rx = robotRotation;

        double fl = y + x + rx, bl = y - x + rx, fr = y - x - rx, br = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));
        frontLeftMotor.setPower(fl / max);
        backLeftMotor.setPower(bl / max);
        frontRightMotor.setPower(fr / max);
        backRightMotor.setPower(br / max);

        telemetry.addLine();
        telemetry.addLine("=== PID PARAMETERS ===");
        telemetry.addData("kP (X)", String.format("%s%.6f", currentParam == TuneParam.KP ? ">>> " : "    ", kP));
        telemetry.addData("kI (B)", String.format("%s%.6f", currentParam == TuneParam.KI ? ">>> " : "    ", kI));
        telemetry.addData("kD (Y)", String.format("%s%.6f", currentParam == TuneParam.KD ? ">>> " : "    ", kD));
        telemetry.addData("kFF (LT)", String.format("%s%.3f", currentParam == TuneParam.KFF ? ">>> " : "    ", kFF));
        telemetry.addLine();
        telemetry.addData("Tracking", trackingEnabled ? "ON (A)" : "OFF (A)");
        telemetry.addData("Robot Rotation", String.format("%.2f", robotRotation));
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
            case KFF:
                kFF += (amount * 50);  // Use larger increments for kFF
                kFF = Math.max(0, kFF);
                break;
        }
    }

    private double applyDeadzone(double v) {
        return Math.abs(v) < 0.05 ? 0 : v;
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
        turretMotor.setPower(0);
    }
}