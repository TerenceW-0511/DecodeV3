package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
            .mass(11.43053)
            .forwardZeroPowerAcceleration(-42.88)
            .lateralZeroPowerAcceleration(-65.984)
            .automaticHoldEnd(true)
//            .translationalPIDFCoefficients(new PIDFCoefficients(.1,0.000001,0.01,0))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.02,0))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0,1,0))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1,0.05870021079635862,0.002449644167067265));


    public static PathConstraints pathConstraints = new PathConstraints(0.95, 50, 1.25, 1);
    public static MecanumConstants driveConstants = new MecanumConstants()//
            .xVelocity(80.94)
            .yVelocity(63.1126)
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .useVoltageCompensation(true)
            .maxPower(1)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.83465)
            .strafePodX(-4.22521)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}