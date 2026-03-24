package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0.00))
            //.headingPIDFCoefficients(new PIDFCoefficients(5, 0,0.75,0.01))
            // .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.6, 0.0, 0.0001, 0.6, 0.025))
            //.useSecondaryTranslationalPIDF(true)
            //.useSecondaryHeadingPIDF(true)
            //.useSecondaryDrivePIDF(true)
            .mass(22.25)
            .forwardZeroPowerAcceleration(-35.45310300521377)
            .lateralZeroPowerAcceleration(-45.59584627903336);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(8)
            .strafePodX(-8.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fR")
            .rightRearMotorName("bR")
            .leftRearMotorName("bL")
            .leftFrontMotorName("fL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(81.12744477039247)
            .yVelocity(59.364026647853095);
    // public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                //.pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}