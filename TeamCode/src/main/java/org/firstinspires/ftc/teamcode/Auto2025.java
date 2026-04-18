package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotPath;

@Autonomous(name="autototoo_rev")
public class Auto2025 extends OpMode{

    private Follower follower = null;

    Robot robot = null;

    private Timer pathTimer; //, actionTimer, opmodeTimer;


    /* data format: x, y, heading, shoot time, shooting velocity, helper_power, maxPower_driveTrain */
    double[] pathInfo_BlueNear = new double[]{
            22,      121,    137.5,    0,   0, 0, 0.65,     // starting position
            44,       99,    137.5,    2,  87, 1, 0.65,     // shooting position
            48, /*83*/81, /*357*/0,    0,   0, 0, 0.35,    // pre collection 1 position
            11, /*83*/81, /*357*/0,    0,   0, 0, 0.65,  // collection 1 position
            44,       99,    140,    2,  87, 1, 0.65,  // shooting position
            48, /*59*/54, /*357*/0,    0,   0, 0, 0.35, // pre collection 2 position
            2,  /*59*/54, /*357*/0,    0,   0, 0, 0.65,  // collection 2 position
            20, /*59*/54, /*357*/0,    0,   0, 0, 0.65,  // collection 2 safe
            44,       99,    137.5,    2,  87, 1, 0.65,    // shooting position

            28,       81,    137.5,    0,  0,  0, 0.65 //strafe to the side (remove after whole comp)



            //ADD BACK LATER
            //       48, /*35*/32, /*357*/0, 0,   0, 0, 0.35,  // pre collection 3 position
            //       0, /*35*/32,        /*357*/0, 0,   0, 0, 0.65 // collection 3 position
            //44,     99,    137.5, 2,  87, 1, 0.65   // shooting position (IF the alliance team thing overlaps w this or we run out of time - make the last position 20, 35, 0, 0, 0)



    };

    // not completed
    double[] pathInfo_BlueFar = new double[]{
            //59, 10, 110, 0, 0, 0,     // starting position
            //56, 18, 110, 7, 120, 0.4,     // shooting position


            65, 10, 110, 0, 0, 0, 0.65,     // starting postion
            65, 15, 110, 10, 125, 0.5, 0.65,    // shooting poisition
            65, 16, 110, 15, 0, 0, 0.65,       //waiting time


            38, 16, 110, 0, 0, 0, 0.65, //delete after match done




            //bring back later
            //56, 25, 110, 0, 0, 0, 0.65,        //leave points



            /*55,     30,      0, 0, 0, 0, 0.65,    // collect 2 pre position
            2,     30,      0, 0, 0, 0, 0.65,    // collect 2 shooting poitingos
            70,      30,        0, 0, 0, 0, 0.65,    // pre shoot 2 position
            76,     67,        135, 5, 105, 0.7, 0.65*/    //shoot 2 position
            //at the end add position (38, 5, 90, 0, 0, 0 for extra leave points)
    };

    // not completed try to only shoot three preloaded balls and then shoot two sets of three preplaced balls for both nears
    //double[] pathInfo_RedNear = new double[]{
    //        122,    121,      42.5, 0,   0,   0,  0.65,  // starting position
    //        100,     99,      42.5, 2,  87,   1,  0.65,   // shooting position
    //         96,     81,     180, 0,   0,   0, 0.25,     // pre collection 1 position
    //        128,     81,     180, 0,   0,   0, 0.65,     // collection 1 position
    //        100,     99,      40.0, 2,  87,   1, 0.65,    // shooting position
    //         96,     55,     180, 0,   0,   0, 0.25,     // pre collection 2 position
    //        142,     56,     180, 0,   0,   0, 0.65,     // collection 2 position
    //        124,     56,     180, 0,   0,   0, 0.65,     // collection 2 safe
    //        100,     99,      42.5, 2,  87,   1, 0.65,    // shooting position
    //         96,     33,     180, 0,   0,   0, 0.25,     // pre collection 3 position
    //        144,     34,     180.0, 0,   0,   0, 0.65,     // collection 3 position
    //        100,     99,      42.5, 2,  87,   1, 0.65    // shooting position
    //};

    double[] pathInfo_RedNear;
    double[] pathInfo_RedFar;
    // not completed
    //double[] pathInfo_RedFar = new double[]{
    //        79, 5, 90, 0, 0, 0, 0.65,     // pre collection 1 position
    //        88,    18,        70, 5, 105, 0.7, 0.65,    // shooting poisition 1
    //        //89,     30,      180, 0, 0, 0,     // collect 2 pre position
    //        //142,     30,      180, 0, 0, 0,     // collect 2 shooting poitingos
    //        //74,      30,        180, 0, 0, 0,     // pre shoot 2 position
    //        //68,     67,        45, 5, 105, 0.7,     //shoot 2 position
    //        106, 5, 90, 0, 0, 0, 0.65
    //        //at the end add position (106, 5, 90, 0, 0, 0 for extra leave points)
    //};

    RobotPath[] pathChain_BlueNear;
    RobotPath[] pathChain_BlueFar;

    RobotPath[] pathChain_RedNear;
    RobotPath[] pathChain_RedFar;

    RobotPath[] pathChain_selected;


    GoBildaPinpointDriver odo;

    int path_index = 0;
    boolean isCompleted = true;

    final int NumOfElements = 7;

    @Override
    public void start()
    {
        robot.start_flywheel(87);
    }
    @Override
    public void loop()
    {
        runPaths();
    }

    @Override
    public void init(){

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.65);

        pathInfo_RedNear = mirrorPathInfoArray(pathInfo_BlueNear);
        pathInfo_RedFar  = mirrorPathInfoArray(pathInfo_BlueFar);

        pathChain_RedNear = buildRobotPathChain(pathInfo_RedNear);
        pathChain_RedFar = buildRobotPathChain(pathInfo_RedFar);
        pathChain_BlueNear = buildRobotPathChain(pathInfo_BlueNear);
        pathChain_BlueFar = buildRobotPathChain(pathInfo_BlueFar);

        telemetry.addLine("Please select your game run!!!");
        telemetry.update();
    }

    public void init_loop()
    {
        super.init_loop();
        if(gamepad1.a)
        {
            for(int k=0; k<10; k++)
                telemetry.addData("<font color='red'>Game Selection</font>", "<font color='red'>Red Near</font>");
            pathChain_selected = pathChain_RedNear;
            Pose startPose_t = new Pose(pathInfo_RedNear[0], pathInfo_RedNear[1], Math.toRadians(pathInfo_RedNear[2]));
            follower.setStartingPose(startPose_t);

            robot.checkLimelight(24);
        }
        else if(gamepad1.b)
        {
            for(int k=0; k<10; k++)
                telemetry.addData("<font color='red'>Game Selection</font>", "<font color='red'>Red Far</font>");
            pathChain_selected = pathChain_RedFar;
            Pose startPose_t = new Pose(pathInfo_RedFar[0], pathInfo_RedFar[1], Math.toRadians(pathInfo_RedFar[2]));
            follower.setStartingPose(startPose_t);

            robot.checkLimelight(24);
        }
        else if(gamepad1.x)
        {
            for(int k=0; k<10; k++)
                telemetry.addData("<font color='blue'>Game Selection</font>", "<font color='blue'>Blue Near</font>");
            pathChain_selected = pathChain_BlueNear;
            Pose startPose_t = new Pose(pathInfo_BlueNear[0], pathInfo_BlueNear[1], Math.toRadians(pathInfo_BlueNear[2]));
            follower.setStartingPose(startPose_t);

            robot.checkLimelight(20);
        }
        else if(gamepad1.y)
        {
            for(int k=0; k<10; k++)
                telemetry.addData("<font color='blue'>Game Selection</font>", "<font color='blue'>Blue Far</font>");
            pathChain_selected = pathChain_BlueFar;
            Pose startPose_t = new Pose(pathInfo_BlueFar[0], pathInfo_BlueFar[1], Math.toRadians(pathInfo_BlueFar[2]));
            follower.setStartingPose(startPose_t);

            robot.checkLimelight(20);
        }

        telemetry.update();
    }

    private double[] mirrorPathInfoArray(double[] pathInfo_array)
    {
        double[] pathInfo_t = pathInfo_array.clone();
        for(int k=0; k<pathInfo_t.length/NumOfElements;k++)
        {
            pathInfo_t[k*NumOfElements+0] = 144 - pathInfo_t[k*NumOfElements+0];
            pathInfo_t[k*NumOfElements+2] = 180 - pathInfo_t[k*NumOfElements+2];
        }
        return pathInfo_t;
    }
    public void runPaths()
    {
        follower.update();

        odo.update();
        Pose2D pos_tt = odo.getPosition();
        telemetry.addLine("path index=" + path_index);
        telemetry.addData("odo xxxxx", pos_tt.getX(DistanceUnit.INCH));
        telemetry.addData("odo yyyyy", pos_tt.getY(DistanceUnit.INCH));
        telemetry.addData("odo heading", pos_tt.getHeading(AngleUnit.DEGREES));
        //telemetry.update();
        telemetry.update();

        if(path_index >= pathChain_selected.length) return;

        if(isCompleted)
        {
            follower.followPath(pathChain_selected[path_index].path, pathChain_selected[path_index].maxPower, true);
            isCompleted = false;
            if(pathChain_selected[path_index].shootTime > 0)
            {
                robot.start_flywheel(pathChain_selected[path_index].shootingVelocity);
            }
        }

        if(!follower.isBusy())
        {
            if(pathChain_selected[path_index].shootTime > 0) {
                robot.start_intake();
                robot.start_indexerHelper(pathChain_selected[path_index].indexerHelp_power);
                pathTimer.resetTimer();
                while (true) {
                    if (pathTimer.getElapsedTimeSeconds() > pathChain_selected[path_index].shootTime) break;
                }
                robot.stop_flywheel();
                robot.stop_IndexerHelper();
                if(path_index == pathChain_selected.length-1)
                {
                    robot.stop_intake();
                }
            }
            isCompleted = true;
            path_index++;
        }
    }

    public RobotPath[] buildRobotPathChain(double[] poslist)
    {
        //final int NumOfElements = 7;
        int num_pos = poslist.length/NumOfElements;
        if(num_pos <=1 ) return null;
        RobotPath[] robotPathList = new RobotPath[num_pos-1];
        for(int k = 0; k<num_pos-1; k++)
        {
            //Pose pos1_t = new Pose(poslist[NumOfElements*k+0], poslist[NumOfElements*k+1], Math.toRadians(poslist[NumOfElements*k+2]));
            //Pose pos2_t = new Pose(poslist[NumOfElements*k+6], poslist[NumOfElements*k+7], Math.toRadians(poslist[NumOfElements*k+8]));
            Pose pos1_t = new Pose(poslist[NumOfElements*k+0], poslist[NumOfElements*k+1], Math.toRadians(poslist[NumOfElements*k+2]));
            Pose pos2_t = new Pose(poslist[NumOfElements*(k+1)+0], poslist[NumOfElements*(k+1)+1], Math.toRadians(poslist[NumOfElements*(k+1)+2]));
            double shootTime_t = poslist[NumOfElements*(k+1)+3];
            double shootingVelocity = poslist[NumOfElements*(k+1)+4];
            double indexerHelp_power = poslist[NumOfElements*(k+1)+5];
            double maxPower = poslist[NumOfElements*k+6];
            PathChain pathchain_t = follower.pathBuilder()
                    .addPath(new BezierLine(pos1_t, pos2_t))
                    .setLinearHeadingInterpolation(pos1_t.getHeading(), pos2_t.getHeading())
                    .build();
            robotPathList[k] = new RobotPath(pathchain_t, shootTime_t, shootingVelocity, indexerHelp_power, maxPower);
        }
        return robotPathList;
    }
}
