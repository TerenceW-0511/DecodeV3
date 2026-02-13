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
            .mass(11.43053)

            .forwardZeroPowerAcceleration(-39.78129302887985) //forward zero power accel tuner -44.619473702954984, - 39.936622122632144, -39.596968154762116, -39.92922602911172, -34.82417513493827
            .lateralZeroPowerAcceleration(-72.39521035844383) //lateral zero power accel tuner -73.51180795502009,-73.64004765072957,-72.70290804677099,-74.22335504722632,-67.89793309247216
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(.25,0,0.03,0))
            .headingPIDFCoefficients(new PIDFCoefficients(2,0,0.1,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.003,0.0001,0.6,0))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,0,0,0,0.6))
            .centripetalScaling(0.0004)
            ;



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 50, 0.75, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)

                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useVoltageCompensation(true)
            .useBrakeModeInTeleOp(true)
            .xVelocity(76.43420333261567) // forward vel tuner //75.62156569112943,75.82549712413879,77.05273821973427,77.28677440252832,76.38444122554749
            .yVelocity(60.284353085014764); //lateral vel tuner 59.233080285740655,58.49578280711737, 61.447703924704726,61.026703121155265,61.218495286355804

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.83465)
            .strafePodX(-4.22521)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


}