package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Map;

public class Constants {
    public final static Map<String, String> Drivetrain_Motors = Map.ofEntries(
            Map.entry("FR", "frontRight"),
            Map.entry("FL", "frontLeft"),
            Map.entry("BR", "backRight"),
            Map.entry("BL", "backLeft")
    );
    //public final static String Drivetrain_FR = "frontRight";
    //public final static String Drivetrain_FL = "frontLeft";
    //public final static String Drivetrain_BR = "backRight";
    //public final static String Drivetrain_BL = "backLeft";

    public final static String OuttakeMotor_Right = "dumb1";
    public final static String OuttakeMotor_Left = "dumb2";
    public final static String Indexer_Motor = "toilet"; //CHANGE THIS
    public final static String Intake_Motor = "intakeMotor";

    public final static String HeadlightName = "head_light";
    public final static String ColorSensorName = "color_sensor";

    public final static String PinPointName = "pinpoint";
    public final static double YawScalarByGobilda = 1.0020024;

    public final static String LimeLightName = "limelight";

    // height difference betwen April Tag (29.5 inch) and Camera914.17 inch)
    public final static double HeightDiffBetweenAprilTagAndCamera = 29.5-14.17;
    public final static double ARTIFACT_DISTANCE_THRESHOLD = 200;


    static final double COUNTS_PER_MOTOR_REV = 537.7; // GoBILDA 5202 Series Yellow Jacket motors
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.75;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    //public static FollowerConstants followerConstants = new FollowerConstants()
    //        .mass(5);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-25.934)
            .lateralZeroPowerAcceleration(-67.343)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.03,
                    0,
                    0,
                    0.015
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.4,
                    0,
                    0.005,
                    0.0006
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0,
                    0.01

            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5,
                    0,
                    0.1,
                    0.0005
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1,
                    0,
                    0.00035,
                    0.6,
                    0.015
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.000005,
                    0.6,
                    0.01
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);


    /*
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(Drivetrain_Motors.get("FR"))
            .rightRearMotorName(Drivetrain_Motors.get("BR"))
            .leftRearMotorName(Drivetrain_Motors.get("BL"))
            .leftFrontMotorName(Drivetrain_Motors.get("FL"))
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(10) //78.262)
            .yVelocity(10); //61.495);
*/

    public static MecanumConstants driveConstants = new MecanumConstants()
            //.maxPower(1)
            .rightFrontMotorName(Drivetrain_Motors.get("FL"))
            .rightRearMotorName(Drivetrain_Motors.get("BL"))
            .leftRearMotorName(Drivetrain_Motors.get("BR"))
            .leftFrontMotorName(Drivetrain_Motors.get("FR"))
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
            //.xVelocity(36.92) //78.262)
            //.yVelocity(26.74); //61.495);

    /*
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(Drivetrain_Motors.get("FL"))
            .rightRearMotorName(Drivetrain_Motors.get("BR"))
            .leftRearMotorName(Drivetrain_Motors.get("BL"))
            .leftFrontMotorName(Drivetrain_Motors.get("FR"))
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(0.5) //78.262)
            .yVelocity(0.5); //61.495);
    */
    /*
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName(Drivetrain_Motors.get("FL"))
            .rightRearMotorName(Drivetrain_Motors.get("BR"))
            .leftRearMotorName(Drivetrain_Motors.get("BL"))
            .leftFrontMotorName(Drivetrain_Motors.get("FR"))
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.001)
            .strafeTicksToInches(0.001)
            .turnTicksToInches(0.001);
    */

    //current encoder odometry
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName(Drivetrain_Motors.get("FL"))
            .rightRearMotorName(Drivetrain_Motors.get("BL"))
            .leftRearMotorName(Drivetrain_Motors.get("BR"))
            .leftFrontMotorName(Drivetrain_Motors.get("FR"))
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .robotLength(17.5)
            .robotWidth(17.3)
            .forwardTicksToInches(0.00541*435/312)
            .strafeTicksToInches(0.00436*435/312)
            //.turnTicksToInches(0.0125);
            .turnTicksToInches(0.0125*435/312);


    public static PinpointConstants localizerConstants_pinpoint = new PinpointConstants()
            .forwardPodY(-2.8125)  // -9 x 5/16
            .strafePodX(7.03125)   // 22.5 x 5/16
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(PinPointName)
            .yawScalar(YawScalarByGobilda)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            //.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            //.strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.25,
            10,
            1);

    public static Follower createFollower(HardwareMap hardwareMap)
    {
        //return null;
        //pathConstraints.setVelocityConstraint(10);
        followerConstants = new FollowerConstants();
        //pathConstraints = new PathConstraints();
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)
                //.pinpointLocalizer(localizerConstants_pinpoint)
                .build();
    }
}
