package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.33352)
            .forwardZeroPowerAcceleration(-55.05)
            .lateralZeroPowerAcceleration(-72.04)
//            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.128, 8.125e-4));
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0.01))
            .useSecondaryDrivePIDF(false) // these are both false by default btw
            .useSecondaryHeadingPIDF(false) // these are both false by default btw
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.00001, 0.6, 0.1))
            .centripetalScaling(0.0004);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .leftFrontMotorName("motorFrontLeft")
            .leftRearMotorName("motorRearLeft")
            .rightFrontMotorName("motorFrontRight")
            .rightRearMotorName("motorRearRight")
            .leftFrontMotorDirection(DcMotor.Direction.FORWARD)
            .leftRearMotorDirection(DcMotor.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotor.Direction.REVERSE)
            .rightRearMotorDirection(DcMotor.Direction.REVERSE)
            .xVelocity(42.39)
            .yVelocity(57.07);

    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(-5.465)
//            .strafePodX(1.812)
            // ones below were measured physically, the ones above were obtained from pedro's offset tuner
            .forwardPodY(1.93)
            .strafePodX(-6.05)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1.4);

    static {
//        followerConstants.setCoefficientsSecondaryDrivePIDF(new FilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0.01));
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}