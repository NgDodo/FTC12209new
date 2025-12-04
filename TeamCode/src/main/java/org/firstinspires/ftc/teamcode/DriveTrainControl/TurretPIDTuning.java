package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.arcrobotics.ftclib.controller.PDController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Turret PID Tuning", group = "Tuning")
public class TurretPIDTuning extends OpMode {

    // === Turret Servo ===
    private CRServo s1;

    // === Vision ===
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private PDController turretPD;

    // === PID Tuning Variables ===
    private double kP = 0.001;
    private double kD = 0.0006;
    private double acceptableTurretError = 0.25;
    private boolean turretTrackingEnabled = false;
    private boolean lastToggle = false;

    // === Tuning Controls ===
    private final double KP_STEP = 0.0001;  // Adjust P by 0.0001 per press
    private final double KD_STEP = 0.00001; // Adjust D by 0.00001 per press
    private final double ERROR_STEP = 0.05; // Adjust acceptable error by 0.05 per press

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    // === Debug Info ===
    private double currentError = 0;
    private double turretPower = 0;
    private boolean tagVisible = false;
    private int tagID = -1;

    @Override
    public void init() {
        // === Turret Servo ===
        s1 = hardwareMap.get(CRServo.class, "s1");

        // === PD Controller ===
        turretPD = new PDController(kP, kD);

        // === AprilTag Vision setup ===
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("=== Turret PID Tuning ===");
        telemetry.addLine("A Button: Toggle Tracking ON/OFF");
        telemetry.addLine("Dpad Up/Down: Adjust kP");
        telemetry.addLine("Dpad Left/Right: Adjust kD");
        telemetry.addLine("Left/Right Bumper: Adjust Error Tolerance");
        telemetry.addLine("Manual Control: Left/Right Triggers");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === Toggle Tracking (A Button) ===
        boolean aPressed = gamepad1.a;
        if (aPressed && !lastToggle) {
            turretTrackingEnabled = !turretTrackingEnabled;
        }
        lastToggle = aPressed;

        // === PID Tuning Controls ===
        // Adjust kP with Dpad Up/Down
        if (gamepad1.dpad_up && !lastDpadUp) {
            kP += KP_STEP;
            turretPD = new PDController(kP, kD);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            kP -= KP_STEP;
            if (kP < 0) kP = 0;
            turretPD = new PDController(kP, kD);
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // Adjust kD with Dpad Left/Right
        if (gamepad1.dpad_right && !lastDpadRight) {
            kD += KD_STEP;
            turretPD = new PDController(kP, kD);
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            kD -= KD_STEP;
            if (kD < 0) kD = 0;
            turretPD = new PDController(kP, kD);
        }
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        // Adjust acceptable error with Bumpers
        if (gamepad1.right_bumper && !lastRightBumper) {
            acceptableTurretError += ERROR_STEP;
        }
        if (gamepad1.left_bumper && !lastLeftBumper) {
            acceptableTurretError -= ERROR_STEP;
            if (acceptableTurretError < 0) acceptableTurretError = 0;
        }
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;

        // === Turret Control ===
        if (turretTrackingEnabled) {
            // Automatic AprilTag tracking
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                AprilTagDetection det = detections.get(0);
                tagVisible = true;
                tagID = det.id;

                // Calculate error (target center is 320 for 640px width)
                double tagX = det.center.x;
                double errorX = tagX - (640.0 / 2.0);
                currentError = errorX;

                if (Math.abs(errorX) > acceptableTurretError) {
                    turretPD.setSetPoint(0);
                    double turretPow = turretPD.calculate(errorX);
                    turretPow = com.qualcomm.robotcore.util.Range.clip(turretPow, -0.75, 0.75);
                    turretPower = turretPow;
                    s1.setPower(turretPow);
                } else {
                    turretPower = 0;
                    s1.setPower(0);
                }
            } else {
                tagVisible = false;
                tagID = -1;
                currentError = 0;
                turretPower = 0;
                s1.setPower(0);
            }
        } else {
            // Manual control with triggers
            tagVisible = false;
            tagID = -1;
            currentError = 0;
            double manualPower = gamepad1.right_trigger - gamepad1.left_trigger;
            turretPower = manualPower;
            s1.setPower(manualPower);
        }

        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addLine("=== Turret PID Tuning ===");
        telemetry.addData("Tracking", turretTrackingEnabled ? "ENABLED (A to disable)" : "DISABLED (A to enable)");
        telemetry.addLine();

        telemetry.addLine("=== PID Values ===");
        telemetry.addData("kP", String.format("%.6f (Dpad Up/Down)", kP));
        telemetry.addData("kD", String.format("%.6f (Dpad Left/Right)", kD));
        telemetry.addData("Error Tolerance", String.format("%.2f pixels (Bumpers)", acceptableTurretError));
        telemetry.addLine();

        telemetry.addLine("=== AprilTag Info ===");
        telemetry.addData("Tag Visible", tagVisible ? "YES" : "NO");
        if (tagVisible) {
            telemetry.addData("Tag ID", tagID);
            telemetry.addData("Error (pixels)", String.format("%.2f", currentError));
            telemetry.addData("Aligned", Math.abs(currentError) <= acceptableTurretError ? "YES ✓" : "NO");
        }
        telemetry.addLine();

        telemetry.addLine("=== Turret Status ===");
        telemetry.addData("Power", String.format("%.3f", turretPower));
        telemetry.addData("Manual Control", turretTrackingEnabled ? "Disabled" : "L/R Triggers");
        telemetry.addLine();

        telemetry.addLine("=== Controls ===");
        telemetry.addLine("A: Toggle Tracking");
        telemetry.addLine("Dpad ↑/↓: Adjust kP");
        telemetry.addLine("Dpad ←/→: Adjust kD");
        telemetry.addLine("Bumpers: Adjust Error Tolerance");
        telemetry.addLine("Triggers: Manual Control (when OFF)");

        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
        s1.setPower(0);
    }
}