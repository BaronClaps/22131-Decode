package org.firstinspires.ftc.teamcode.config.pedro;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6)
            .forwardZeroPowerAcceleration(-42.072)
            .lateralZeroPowerAcceleration(-67.58)
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(.2,0,.02,.015))
            //.translationalPIDFCoefficients(new PIDFCoefficients(.1,0,.01,0))
            //.headingPIDFCoefficients(new PIDFCoefficients(3, 0, .02, 0))
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(.2, 0, 0.005, 0))
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .useBrakeModeInTeleOp(true)
            .xVelocity(76.82)
            .yVelocity(61.54)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(1.5)
            .strafePodX(-7.25)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}
