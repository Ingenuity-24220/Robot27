package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.camera.CameraImpl;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;

import java.util.function.Supplier;

/*IMPORTANT NOTE
        region 7 = forward
        1; right
        left; 5
        back; 3
 */

@TeleOp(name = "robot")
public class TeleOp2025 extends OpMode{


    double[] shootingPos_BlueNear = new double[]{
            44,     99,    137.5, 2,  87, 1,      // shooting position
    };

    // not completed
    double[] shootingPos_BlueFar = new double[]{
            76,    67,        135, 5, 105, 0.7,     // shooting poisition 1
    };

    // not completed
    double[] shootingPos_RedNear = new double[]{
            100,     99,      42.5, 2,  87,   1,     // shooting position
    };

    // not completed
    double[] shootingPos_RedFar = new double[]{
            88,    18,        70, 5, 105, 0.7,     // shooting poisition 1
    };


    Robot robot = null;
    Pose shooting_pos = null;

    Follower follower;
    private Supplier<PathChain> pathChain;
    public static Pose startingPose;
    public static Pose shootingPose;
    //private TelemetryManager telemetryM;
    private boolean automatedDrive;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private int aprilTagId = 20;

    int flick_cnt = 0;

    @Override
    public void init() {

        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        follower = Constants.createFollower(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        if(shooting_pos == null) shooting_pos = new Pose();

        if(startingPose == null) startingPose = new Pose();
        follower.setStartingPose(startingPose);
        follower.update();
        //telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start()
    {
        follower.startTeleOpDrive();
        //follower.setStartingPose(new Pose());
        //follower.update();
        //follower.setTeleOpDrive(
        //        -gamepad1.left_stick_y,
        //        -gamepad1.left_stick_x,
        //        -gamepad1.right_stick_x,
        //        true // Robot Centric
        //);
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, shooting_pos)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, shooting_pos.getHeading(), 0.8))
                .build();
    }
    public void init_loop()
    {
        super.init_loop();
        if(gamepad1.a)
        {
            for(int k=0; k<10; k++)
                telemetry.addData("<font color='red'>Game Selection</font>", "<font color='red'>Red Near</font>");
            shooting_pos = new Pose(shootingPos_RedNear[0], shootingPos_RedNear[1], Math.toRadians(shootingPos_RedNear[2]));
            aprilTagId = 24;
        }
        else if(gamepad1.b) {
            for (int k = 0; k < 10; k++)
                telemetry.addData("<font color='red'>Game Selection</font>", "<font color='red'>Red Far</font>");
            shooting_pos = new Pose(shootingPos_RedFar[0], shootingPos_RedFar[1], Math.toRadians(shootingPos_RedFar[2]));
            aprilTagId = 24;
        }
        else if(gamepad1.x)
        {
            for(int k=0; k<10; k++)
                telemetry.addData("<font color='blue'>Game Selection</font>", "<font color='blue'>Blue Near</font>");
            shooting_pos = new Pose(shootingPos_BlueNear[0], shootingPos_BlueNear[1], Math.toRadians(shootingPos_BlueNear[2]));
            aprilTagId = 20;
        }
        else if(gamepad1.y)
        {
            for(int k=0; k<10; k++)
                telemetry.addData("<font color='blue'>Game Selection</font>", "<font color='blue'>Blue Far</font>");
            shooting_pos = new Pose(shootingPos_BlueFar[0], shootingPos_BlueFar[1], Math.toRadians(shootingPos_BlueFar[2]));
            aprilTagId = 20;
        }
        telemetry.update();
    }

    @Override
    public void loop() {

        //robot.checkArtifactNum();

        robot.intake(); //servo controlling intake

        drive();

        robot.outtake();

        robot.indexerHelper();

        robot.spitOutBalls();

        double[] result = robot.checkLimelight(aprilTagId);

        if(result[0] >= 0) {
            //telemetry.addData("hello hello", result[0]);
            if (Math.abs(result[1]) < 3) {
                //robot.setHeadLightBrightness(-2);
                robot.turnon_headlight();
                telemetry.addData("on on", result[0]);
            }
            else {
                //robot.setHeadLightBrightness(-2);
                robot.turnoff_headlight();
                telemetry.addData("off off", result[0]);
            }
                //flick_cnt++;
        }
        telemetry.addData("distance", result[0]);
        telemetry.addData("aiming angle=", "" + result[1]);

        telemetry.update();
    }

    private void drive()
    {
        follower.update();
        //telemetryM.update();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }
        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, shooting_pos)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, shooting_pos.getHeading(), 0.8))
                    .build();
            follower.followPath(pathChain.get());
            follower.setMaxPower(0.8);
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            follower.setMaxPower(1);
            automatedDrive = false;
        }
        //Slow Mode
        if (gamepad1.leftBumperWasPressed()) {
            //slowMode = !slowMode;
            shooting_pos = follower.getPose();
        }

        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }
        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }
        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }
        telemetry.addData("shooting position", shooting_pos);
        telemetry.addData("current position", follower.getPose());
        //telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);
    }
}