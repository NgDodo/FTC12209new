package org.firstinspires.ftc.teamcode.DriveTrainControl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Import Marrow Zone classes
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

@TeleOp(name = "Marrow Zone Test", group = "Test")
public class MarrowZoneTest extends OpMode {

    // === Define Field Zones ===
    // Close launch zone (triangle near high basket)
    private final PolygonZone closeLaunchZone = new PolygonZone(
            new Point(144, 144),
            new Point(72, 72),
            new Point(0, 144)
    );

    // Far launch zone (triangle on opposite side)
    private final PolygonZone farLaunchZone = new PolygonZone(
            new Point(48, 0),
            new Point(72, 24),
            new Point(96, 0)
    );

    // Blue base zone (20" x 20" square)
    private final PolygonZone blueBase = new PolygonZone(
            new Point(105.5, 33.5),
            20,
            20
    );

    // Red base zone (20" x 20" square)
    private final PolygonZone redBase = new PolygonZone(
            new Point(38.5, 33.5),
            20,
            20
    );

    // Represent the robot as an 18" by 18" zone
    private final PolygonZone robotZone = new PolygonZone(16.5, 17.5);

    // Pedro Pathing follower for odometry
    private Follower follower;

    // Drive motors
    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // Zone status tracking
    private boolean wasInCloseLaunch = false;
    private boolean wasInFarLaunch = false;
    private boolean wasFullyInRedBase = false;
    private boolean wasFullyInBlueBase = false;

    @Override
    public void init() {
        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 90)); // Start at field center

        // Initialize drive motors
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "fL");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "bL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "bR");

        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behavior to brake
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("=== Marrow Zone Test Initialized ===");
        telemetry.addLine("Drive around to test zone detection!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick: Translate");
        telemetry.addLine("  Right Stick X: Rotate");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Update odometry
        follower.update();

        // Apply teleop drive controls (mecanum drive)
        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
        double rx = applyDeadzone(gamepad1.right_stick_x);

        // Calculate motor powers for mecanum drive
        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        // Normalize powers to [-1, 1]
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeftMotor.setPower(fl / max);
        backLeftMotor.setPower(bl / max);
        frontRightMotor.setPower(fr / max);
        backRightMotor.setPower(br / max);

        // === Sync robot zone with actual robot position and rotation ===
        robotZone.setPosition(follower.getPose().getX(), follower.getPose().getY());
        robotZone.setRotation(follower.getPose().getHeading());

        // === Zone Detection Logic ===

        // Check if robot is in close launch zone
        boolean inCloseLaunch = robotZone.isInside(closeLaunchZone);
        if (inCloseLaunch && !wasInCloseLaunch) {
            // Just entered close launch zone
            gamepad1.rumble(200); // Haptic feedback
        }
        wasInCloseLaunch = inCloseLaunch;

        // Check if robot is in far launch zone
        boolean inFarLaunch = robotZone.isInside(farLaunchZone);
        if (inFarLaunch && !wasInFarLaunch) {
            // Just entered far launch zone
            gamepad1.rumble(200);
        }
        wasInFarLaunch = inFarLaunch;

        // Check if robot is fully parked in red base
        boolean fullyInRedBase = robotZone.isFullyInside(redBase);
        if (fullyInRedBase && !wasFullyInRedBase) {
            // Just fully parked in red base
            gamepad1.rumble(500); // Longer rumble for full parking
        }
        wasFullyInRedBase = fullyInRedBase;

        // Check if robot is fully parked in blue base
        boolean fullyInBlueBase = robotZone.isFullyInside(blueBase);
        if (fullyInBlueBase && !wasFullyInBlueBase) {
            // Just fully parked in blue base
            gamepad1.rumble(500);
        }
        wasFullyInBlueBase = fullyInBlueBase;

        // Calculate distances to bases
        double distanceToRedBase = robotZone.distanceTo(redBase);
        double distanceToBlueBase = robotZone.distanceTo(blueBase);

        // Example: Warn if getting close to a base
        boolean nearRedBase = distanceToRedBase < 10;
        boolean nearBlueBase = distanceToBlueBase < 10;

        // === Telemetry ===
        telemetry.addLine("=== Robot Position ===");
        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine();

        telemetry.addLine("=== Zone Status ===");
        telemetry.addData("In Close Launch", inCloseLaunch ? "YES ✓" : "NO");
        telemetry.addData("In Far Launch", inFarLaunch ? "YES ✓" : "NO");
        telemetry.addLine();

        telemetry.addData("Fully In Red Base", fullyInRedBase ? "YES ✓✓" : "NO");
        telemetry.addData("Fully In Blue Base", fullyInBlueBase ? "YES ✓✓" : "NO");
        telemetry.addLine();

        telemetry.addLine("=== Distance to Bases ===");
        telemetry.addData("Red Base", "%.1f\" %s", distanceToRedBase,
                nearRedBase ? "(NEAR!)" : "");
        telemetry.addData("Blue Base", "%.1f\" %s", distanceToBlueBase,
                nearBlueBase ? "(NEAR!)" : "");
        telemetry.addLine();

        telemetry.addLine("=== Status Messages ===");
        if (fullyInRedBase) {
            telemetry.addLine("✓ PARKED IN RED BASE");
        } else if (fullyInBlueBase) {
            telemetry.addLine("✓ PARKED IN BLUE BASE");
        } else if (inCloseLaunch) {
            telemetry.addLine("→ In Close Launch Zone");
        } else if (inFarLaunch) {
            telemetry.addLine("→ In Far Launch Zone");
        } else if (nearRedBase || nearBlueBase) {
            telemetry.addLine("⚠ Near Base Zone");
        } else {
            telemetry.addLine("• Driving...");
        }

        telemetry.update();
    }

    /**
     * Apply deadzone to controller inputs
     */
    private double applyDeadzone(double value) {
        return Math.abs(value) < 0.05 ? 0 : value;
    }
}