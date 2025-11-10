package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
          //  .forwardZeroPowerAcceleration(1) //change value eventually
          //  .lateralZeroPowerAcceleration(1) //change value eventually
          //  .translationalPIDFCoefficients(new PIDFCoefficients(2,2,2,2))
           // .headingPIDFCoefficients(new PIDFCoefficients(2, 2,2,2))
           // .drivePIDFCoefficients(new FilteredPIDFCoefficients(2, 2, 2, 2, 2))
            .mass(22.25);

    public static MecanumConstants driveConstants = new MecanumConstants()
            //.xVelocity(1) //change value eventually
            //.yVelocity(1) //change value eventually
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
            .forwardPodY(8.5)
            .strafePodX(8)
           // .forwardTicksToInches(.0166)
            //.forwardEncoderDirection(Encoder.REVERSE)
            //.strafeEncoderDirection(Encoder.REVERSE)
           // .strafeTicksToInches(.00127)
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
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