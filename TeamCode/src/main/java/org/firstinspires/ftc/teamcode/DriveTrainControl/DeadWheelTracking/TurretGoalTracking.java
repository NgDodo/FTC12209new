package org.firstinspires.ftc.teamcode.DriveTrainControl.DeadWheelTracking;
//
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DriveTrainControl.IntegratedTurretTracking;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
@TeleOp(name="TurretGoalTracking", group="DriveTrainControl")
public class TurretGoalTracking extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private Supplier<PathChain> pathChain;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;


    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1;
    DcMotorEx m2; // Turret motor
    DcMotorEx m3, m0;
    CRServo s3;
    Servo s2;

    // === Color Sensors & RGB LEDs ===
    private RevColorSensorV3 intakeColor;
    private RevColorSensorV3 shooterColor;
    private Servo led1;
    private Servo led2;

    // === Tracking Mode ===
    private enum TrackingMode {
        OFF,
        TURRET_TRACK
    }
    private TrackingMode currentTrackingMode = TrackingMode.OFF;
    private boolean lastBButton = false;

    // === Turret Configuration ===
    private static final double TURRET_RANGE_DEG = 330.0;
    private static final double TICKS_PER_REV = 1393.1;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    private static final int CENTER_POSITION = 0;
    private static final int MAX_POSITION = (int)((TURRET_RANGE_DEG / 2.0) * TICKS_PER_DEGREE);
    private static final int MIN_POSITION = -MAX_POSITION;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        //Call this once per loop
        follower.update();

        // === DriveTrain ===
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

        // === Mechanisms ===
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m0 = hardwareMap.get(DcMotorEx.class, "m0");

        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");
        s3.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setPosition(.68);

        for (DcMotor motor : new DcMotor[]{m1, m3, m0}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // === Turret Setup ===
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === Drive Train ===
        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
        double rx = applyDeadzone(gamepad1.right_stick_x);

        double fl = y + x + rx, bl = y - x + rx, fr = y - x - rx, br = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));
        frontLeftMotor.setPower(clipLowPower(fl / max));
        backLeftMotor.setPower(clipLowPower(bl / max));
        frontRightMotor.setPower(clipLowPower(fr / max));
        backRightMotor.setPower(clipLowPower(br / max));

        // === Turret Rotation Logic ===
        // the turret only tracks the LEFT SCORING (idk if that's blue or red)
        // it takes the arctan(y distance / x distance), relative to the scoring goal
        // luckily, PedroPathing has (0,144) as the location for the top left, which is nice
            // i'll use (10, 134) as the goal coordinates
        // reference: https://pedropathing.com/docs/pathing/reference/coordinates

        Pose GOAL_POST = new Pose(10, 134, 0);
        // 1. calculate component distances from goal
        double y_goal_distance = follower.getPose().getY() - GOAL_POST.getY();
        double x_goal_distance = follower.getPose().getX() - GOAL_POST.getX();

        // 2. calculate robot+turret relative angle to goal
        // note: the robot's heading must be subtracted to find the desired angle of the turret

        double angle_to_goal = Math.atan(Math.abs(y_goal_distance / x_goal_distance)) + Math.PI/2;
        // angle_to_goal is a number from 90 to 180 deg (ideally), since we took the absolute value + PI/2
            // note: this won't be the case if the robot is on the edges of the field (x < 10, y > 134, etc.)
            // this value corresponds to PedroPathing's heading coordinate system
            // therefore, absolute angle to goal - robot absolute angle = turret offset angle, relative to robot
            // this "turret_offset_angle" will be how much the turret needs to turn

        double turretRelativeOffset = angle_to_goal - follower.getHeading(); // in radians
        // now is fun part: dissecting how gear ratios work to translate the relative offset coordinates to turret motion
        moveTurretToOffset(m2, turretRelativeOffset); // ROTATE TURRET

        // == POSE RESET SYSTEM ==
        // to reset the robot's pose at the beginning of teleop, go to the small white triangle at the base of the field (72, 10)
        // and face the robot to the front of the field

        if (gamepad1.dpad_down) {
            follower.setPose(new Pose(72, 10, Math.PI / 2));
        }

        telemetry.addData("position", follower.getPose());
        telemetry.update();
    }
    private double getTimeSeconds() {
        return System.nanoTime() / 1e9;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private double clipLowPower(double p) {
        return Math.abs(p) < 0.04 ? 0 : p;
    }
    private double applyDeadzone(double v) {
        return Math.abs(v) < 0.05 ? 0 : v;
    }

    private void moveTurretToOffset(DcMotorEx turretMotor, double turretRelativeOffset){
        // this function assumes that 1) the turret started facing the front of the robot at hte beginning of auton with an encoder value of 0
        // and 2) that the encoder value was never reset between auton and teleop/the start of teleop
        // 3) turretRelativeOffset is in radians

        // translate fraction of turret rotation -> output gear teeth -> input gear teeth -> fraction of motor input rotation -> motor ticks
        double OUTPUT_GEAR_TEETH = 10;
        double INPUT_GEAR_TEETH = 5;


        double output_gear_fractional_rotation = Math.toDegrees(turretRelativeOffset) / 360.0;
        double output_gear_teeth_rotation = output_gear_fractional_rotation * OUTPUT_GEAR_TEETH;
        double input_gear_teeth_rotation = output_gear_teeth_rotation * (INPUT_GEAR_TEETH / OUTPUT_GEAR_TEETH);
        double input_gear_fractional_rotation = input_gear_teeth_rotation / INPUT_GEAR_TEETH;
        double ticks_to_rotate = TICKS_PER_REV * input_gear_fractional_rotation;

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setTargetPosition((int)Math.round(ticks_to_rotate));
        turretMotor.setPower(0.2);
    }

}