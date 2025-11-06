package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(5) //change value eventually
            .lateralZeroPowerAcceleration(5) //change value eventually
            .mass(22.25);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(5) //change value eventually
            .yVelocity(5) //change value eventually
            .maxPower(1)
            .rightFrontMotorName("fR")
            .rightRearMotorName("bR")
            .leftRearMotorName("bL")
            .leftFrontMotorName("fL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("m1")
            .strafeEncoder_HardwareMapName("bL")
            .IMU_HardwareMapName("imu")
            .forwardPodY(8.5) //not actually 2, change it later and measure in inches.
            .strafePodX(8)
            //.forwardTicksToInches(1)
            //.forwardEncoderDirection(Encoder.REVERSE)
            //.strafeEncoderDirection(Encoder.REVERSE)
            //.strafeTicksToInches(multiplier)
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )

            );
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .twoWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}