package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SparkFun OTO Test (Inches + Drive)", group = "DriveTrainControl")
public class SparkfunTest extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private SparkFunOTOSCorrected odometry;

    // Conversion constant (sensor gives mm → convert to inches)
    private static final double MM_TO_INCH = 0.0393701;

    @Override
    public void runOpMode() throws InterruptedException {
        // === Drivetrain setup ===
        frontLeft = hardwareMap.get(DcMotorEx.class, "fL");
        backLeft = hardwareMap.get(DcMotorEx.class, "bL");
        frontRight = hardwareMap.get(DcMotorEx.class, "fR");
        backRight = hardwareMap.get(DcMotorEx.class, "bR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // === SparkFun OTOS Sensor Setup ===
        try {
            odometry = hardwareMap.get(SparkFunOTOSCorrected.class, "odometry");
        } catch (Exception e) {
            telemetry.addData("ERROR", "SparkFunOTOSCorrected not found in config");
            telemetry.update();
            sleep(2000);
        }

        telemetry.addLine("SparkFun OTOS Initialized\nPress Play to Start");
        telemetry.update();
        waitForStart();

        double xInch = 0;
        double yInch = 0;
        double headingDeg = 0;

        while (opModeIsActive()) {
            // === Drive with joysticks ===
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                    Math.max(Math.abs(fr), Math.abs(br)))));

            frontLeft.setPower(fl / max);
            backLeft.setPower(bl / max);
            frontRight.setPower(fr / max);
            backRight.setPower(br / max);

            // === Read Sensor ===
            if (odometry != null) {
                //double dxMM = odometry.getDeltaX(); // mm since last loop
                //double dyMM = odometry.getDeltaY();
                //headingDeg = Math.toDegrees(odometry.getHeading());

                //xInch += dxMM * MM_TO_INCH;
                //yInch += dyMM * MM_TO_INCH;

                telemetry.addLine("=== SparkFun OTOS ===");
                //telemetry.addData("dx (in)", "%.3f", dxMM * MM_TO_INCH);
                //telemetry.addData("dy (in)", "%.3f", dyMM * MM_TO_INCH);
                telemetry.addData("Total X (in)", "%.3f", xInch);
                telemetry.addData("Total Y (in)", "%.3f", yInch);
                telemetry.addData("Heading (deg)", "%.2f", headingDeg);
            } else {
                telemetry.addLine("ERROR: No SparkFun OTOS detected");
            }

            telemetry.update();
            sleep(20);
        }
    }
}
