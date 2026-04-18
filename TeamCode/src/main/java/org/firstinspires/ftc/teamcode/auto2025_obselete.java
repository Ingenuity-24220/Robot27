package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

@Autonomous(name="autototoo_obselete")
//@Disabled
public class auto2025_obselete extends OpMode{

    private Follower follower = null;

    Robot robot = null;

    //GoBildaPinpointDriver odo = null;

    public enum PathState
    {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }

    private final Pose tuning_pos1 = new Pose(0, 0, Math.toRadians(0));
    private final Pose tuning_pos2 = new Pose(20, 0, Math.toRadians(0));

    private final Pose startPose = new Pose(22.038, 120.962, Math.toRadians(137.5)); //starting at blue shooting place
    private final Pose shootPose = new Pose(44, 99, Math.toRadians(137.5));

    private final Pose collect1PrevPos = new Pose(48, 83, Math.toRadians(0));
    private final Pose collect1Pos = new Pose(18, 84, Math.toRadians(0));
    private final Pose collect2PrevPos = new Pose(48, 59, Math.toRadians(-1));
    private final Pose collect2Pos = new Pose(2, 60, Math.toRadians(0));
    private final Pose collect2PosSafe = new Pose(20, 60, Math.toRadians(-1));
    private final Pose collect3PrevPos = new Pose(48, 34, Math.toRadians(-1));
    private final Pose collect3Pos = new Pose(2, 35, Math.toRadians(-1));

    private PathChain tuning_path;
    private PathChain driveStartPosShootPos;
    private PathChain driveCollect1Prev;
    private PathChain driveCollect1Pos;
    private PathChain driveCollect1ShootPos;
    private PathChain driveCollect2Prev;
    private PathChain driveCollect2Pos;
    private PathChain driveCollect2PosSafe;
    private PathChain driveCollect2ShootPos;
    private PathChain driveCollect3Prev;
    private PathChain driveCollect3ShootPos;

    private PathChain SHOTOTOTOTOOTOOTOTT;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    GoBildaPinpointDriver  odo;
    double F = 0.005;
    double P = 280;


    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    private DcMotor BR = null;


    private IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

    static final double TURN_POWER = 0.5;
    static final double DEGREES_PER_INCH_OF_WHEEL_TRAVEL = 10; // This must be calculated or tuned for your specific robot base


    @Override
    public void loop()
    {
        follower.update();
        statePathUpdate();
        odo.update();
        telemetry.addData("path state", pathState);
        telemetry.addData("X and Y", "" + String.format("%.2f", odo.getPosX(DistanceUnit.INCH)) + " : " + String.format("%.2f", odo.getPosY(DistanceUnit.INCH)));
        telemetry.addData("X and Y encoder","" + odo.getEncoderX() + " : " + odo.getEncoderY());
        telemetry.addData("Heading", String.format("%.2f", odo.getHeading(AngleUnit.DEGREES)));
        telemetry.update();
    }

    @Override
    public void init(){

        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        //follower.setMaxPower(0.25);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PinPointName);
        odo.resetPosAndIMU();

        buildPaths();
        follower.setStartingPose(startPose);

        //choosisng between tuning and actual path
        pathState = -1;

        telemetry.update();
    }


    public void buildPaths()
    {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveCollect1Prev = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect1PrevPos))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect1PrevPos.getHeading())
                .build();

        driveCollect1Pos = follower.pathBuilder()
                .addPath(new BezierLine(collect1PrevPos, collect1Pos))
                .setLinearHeadingInterpolation(collect1PrevPos.getHeading(), collect1Pos.getHeading())
                .build();

        driveCollect1ShootPos  = follower.pathBuilder()
                .addPath(new BezierLine(collect1Pos, shootPose))
                .setLinearHeadingInterpolation(collect1Pos.getHeading(), shootPose.getHeading())
                .build();

        driveCollect2Prev = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect2PrevPos))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect2PrevPos.getHeading())
                .build();

        driveCollect2Pos = follower.pathBuilder()
                .addPath(new BezierLine(collect2PrevPos, collect2Pos))
                .setLinearHeadingInterpolation(collect2PrevPos.getHeading(), collect2Pos.getHeading())
                .build();

        driveCollect2PosSafe = follower.pathBuilder()
                .addPath(new BezierLine(collect2Pos, collect2PosSafe))
                .setLinearHeadingInterpolation(collect2Pos.getHeading(), collect2PosSafe.getHeading())
                .build();

        driveCollect2ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(collect2PosSafe, shootPose))
                .setLinearHeadingInterpolation(collect2PosSafe.getHeading(), shootPose.getHeading())
                .build();

        driveCollect3Prev = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect3PrevPos))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect3PrevPos.getHeading())
                .build();

        driveCollect3ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(collect3PrevPos, collect3Pos))
                .setLinearHeadingInterpolation(collect3PrevPos.getHeading(), collect3Pos.getHeading())
                .build();

        SHOTOTOTOTOOTOOTOTT = follower.pathBuilder()
                .addPath(new BezierLine(collect3Pos, shootPose))
                .setLinearHeadingInterpolation(collect3Pos.getHeading(), shootPose.getHeading())
                .build();

        tuning_path = follower.pathBuilder()
                .addPath(new BezierLine(tuning_pos1, tuning_pos2))
                .setLinearHeadingInterpolation(tuning_pos1.getHeading(), tuning_pos2.getHeading())
                .build();
    }

    public void statePathUpdate()
    {
        switch(pathState)
        {
            case -1: // for tuning
                follower.setStartingPose(tuning_pos1);
                follower.followPath(tuning_path, 0.25, true);
                pathState = -100;
                break;
            case -100:
                if(!follower.isBusy())
                {
                    telemetry.addLine("Tuning: " + pathTimer.getElapsedTimeSeconds());
                    pathState = -101;
                }
                break;

            case 0:
                follower.followPath(driveStartPosShootPos, true);
                pathState = 1;
                robot.start_flywheel(87);
                pathTimer.resetTimer();
                while(true) {
                    if(pathTimer.getElapsedTime()>1000) break;
                }
                break;
            case 1:
                if(!follower.isBusy())
                {
                    robot.start_intake();
                    robot.start_indexerHelper(1);
                    pathTimer.resetTimer();
                    while(true) {
                        if(pathTimer.getElapsedTime()>2000) break;
                    }

                    robot.stop_flywheel();
                    robot.stop_IndexerHelper();

                    follower.followPath(driveCollect1Prev, true);
                    telemetry.addLine("Done State Machine");
                    pathState = 2;
                }
                break;
            case 2:
                if(!follower.isBusy())
                {
                    follower.followPath(driveCollect1Pos, true);
                    telemetry.addLine("Done wtvvtvttv");
                    pathState = 3;
                    pathTimer.resetTimer();
                    while(true) {
                        if(pathTimer.getElapsedTime()>500) break;
                    }
                }
                break;
            case 3:
                if(!follower.isBusy())
                {
                    follower.followPath(driveCollect1ShootPos, true);
                    pathState = 4;
                    pathTimer.resetTimer();
                    while(true) {
                        if(pathTimer.getElapsedTime()>500) break;
                    }
                    robot.start_flywheel(87);
                }
                break;
            case 4:
                if(!follower.isBusy())
                {

                    robot.start_indexerHelper(1);

                    pathTimer.resetTimer();
                    while(true) {
                        if(pathTimer.getElapsedTime()>2000) break;
                    }

                    robot.stop_IndexerHelper();
                    robot.stop_flywheel();

                    follower.followPath(driveCollect2Prev, true);
                    pathState = 5;

                }
                break;
            case 5:
                if(!follower.isBusy())
                {
                    follower.followPath(driveCollect2Pos, true);
                    pathState = 6;
                    pathTimer.resetTimer();
                    while(true) {
                        if(pathTimer.getElapsedTime()>500) break;
                    }
                }
                break;

            case 6:
                if(!follower.isBusy())
                {
                    follower.followPath(driveCollect2PosSafe, true);
                    pathState = 7;
                }

            case 7:
                if(!follower.isBusy())
                {
                    follower.followPath(driveCollect2ShootPos, true);
                    pathState = 8;
                    pathTimer.resetTimer();
                    while(true) {
                        if(pathTimer.getElapsedTime()>500) break;
                    }

                    robot.start_flywheel(87);

                }
                break;
            case 8:
                if(!follower.isBusy())
                {
                    robot.start_indexerHelper(1);

                    pathTimer.resetTimer();
                    while(true) {
                        if(pathTimer.getElapsedTime()>2000) break;
                    }

                    robot.stop_IndexerHelper();
                    robot.stop_flywheel();

                    follower.followPath(driveCollect3Prev, true);
                    pathState = 9;
                    pathTimer.resetTimer();
                    while(true) {
                        if(pathTimer.getElapsedTime()>500) break;
                    }
                }
                break;
            case 9:
                if(!follower.isBusy())
                {
                    follower.followPath(driveCollect3ShootPos, true);
                    pathState = 10;
                    pathTimer.resetTimer();
                    while(true) {
                        if(pathTimer.getElapsedTime()>500) break;
                    }

                    robot.start_flywheel(87);

                }
                break;
            case 10:
                if(!follower.isBusy())
                {
                    follower.followPath(SHOTOTOTOTOOTOOTOTT, true);
                    pathState = 11;
                }
                break;
            case 11:
                if(!follower.isBusy())
                {
                    robot.start_indexerHelper(1);

                    pathTimer.resetTimer();
                    while(true) {
                        if(pathTimer.getElapsedTime()>2000) break;
                    }

                    robot.stop_IndexerHelper();
                    robot.stop_flywheel();

                    pathState = -100;
                }
                break;
            default:
                telemetry.addLine("No state command");
                break;
        }
    }



    /*public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if(!follower.isBusy()) {
                    /* Score Preload

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position
                if(!follower.isBusy()) {
                    /* Grab Sample

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths
                    setPathState(-1);
                }
                break;
        }
    }*/

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /*
    public void moveStraightInches(double inches, double power)
    {
        final double TICKS_PER_INCH = 100/6;
        int targetPosition = (int)(inches * TICKS_PER_INCH);

        FR.setTargetPosition(FR.getCurrentPosition() + targetPosition);
        FL.setTargetPosition(FR.getCurrentPosition() + targetPosition);
        BR.setTargetPosition(FR.getCurrentPosition() + targetPosition);
        BL.setTargetPosition(FR.getCurrentPosition() + targetPosition);

        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FR.setPower(power);
        FL.setPower(power);
        BR.setPower(power);
        BL.setPower(power);

        while(opModeIsActive() && FR.isBusy() && FL.isBusy() && BR.isBusy() && BL.isBusy())
        {
            telemetry.addData("Status", "moving to position like a BUM");
            telemetry.addData("Target FrontRight", FR.getTargetPosition());
            telemetry.addData("Current FrontRight", FR.getCurrentPosition());
            telemetry.update();
        }

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    */
    /*
    public void turnWithEncoders(double degrees, double power)
    {

        int moveCounts = (int)(degrees * DEGREES_PER_INCH_OF_WHEEL_TRAVEL * COUNTS_PER_INCH);

        int leftTarget = FL.getCurrentPosition() + moveCounts;
        int rightTarget = FR.getCurrentPosition() - moveCounts;

        FL.setTargetPosition(leftTarget);
        BL.setTargetPosition(leftTarget);
        FR.setTargetPosition(rightTarget);
        BR.setTargetPosition(rightTarget);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(Math.abs(power));

        while (opModeIsActive() && (FL.isBusy() || FR.isBusy())) {
            // Optional: Add telemetry to monitor progress
            telemetry.addData("Target Left", leftTarget);
            telemetry.addData("Current Left", FL.getCurrentPosition());
            telemetry.addData("Target Right", rightTarget);
            telemetry.addData("Current Right", FR.getCurrentPosition());
            telemetry.update();
        }

        setMotorPower(0);

        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    */
    /*
    public void setMotorPower(double power)
    {
        FL.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        BR.setPower(power);
    }

    public void setMotorMode(DcMotor.RunMode mode) {
        FL.setMode(mode);
        BL.setMode(mode);
        FR.setMode(mode);
        BR.setMode(mode);
    }
    */

}
