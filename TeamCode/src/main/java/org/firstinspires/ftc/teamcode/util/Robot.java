package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;


public class Robot
{
    private AngleCalculator_Left angleCalculatorLeft;
    private AngleCalculator_Right angleCalculatorRight;

    Map<String, DcMotorEx> DriveBase_MotorList = new HashMap<>();
    DcMotorEx rightOuttake;
    DcMotorEx leftOuttake;

    DcMotor intakeMotor;

    DcMotor indexerMotor;

    Telemetry t;
    Gamepad gamepad1;
    Gamepad gamepad2;

    IMU imu =  null;

    private Limelight3A limelight;

    final double HIGH_SPEED = 125; //target 850
    final double LOW_SPEED = 87; //target 940

    final double MID_SPEED = 108;
    double cur_FlyWheel_TargetVelocity = 0;
    //final double INIT_SPEED = 100;
    double F = 0.005;
    double P = 280;

    final double POWER_FACTOR = 1; //speed of speed

    double indexerHelperSpeed = 0; //speed of indexer helper

    RevColorSensorV3 color_sensor = null;

    CRServo headlight = null; //CHANGENHCANGEHHEEH HUROTO CRSERVO IN CONFIGURE INNNN THINGY

    boolean isHeadlight = false;
    boolean isColorSensor = false;

    double artifact_dist1 = Constants.ARTIFACT_DISTANCE_THRESHOLD;
    double artifact_dist2 = Constants.ARTIFACT_DISTANCE_THRESHOLD;
    int artifactNum = 0;

    boolean isArtifactFound = false;
    Map<DriveDirection, Map<String, Integer>> DRIVEBASE_PARAMETERS = Map.ofEntries(
            Map.entry(DriveDirection.BACKWARD,     Map.ofEntries(Map.entry("FL",-1), Map.entry("FR", 1), Map.entry("BR", 1), Map.entry("BL",-1))),
            Map.entry(DriveDirection.FORWARD,    Map.ofEntries(Map.entry("FL", 1), Map.entry("FR",-1), Map.entry("BR",-1), Map.entry("BL", 1))),
            Map.entry(DriveDirection.LEFT_SHIFT,  Map.ofEntries(Map.entry("FL", 1), Map.entry("FR", 1), Map.entry("BR",-1), Map.entry("BL",-1))),
            Map.entry(DriveDirection.RIGHT_SHIFT, Map.ofEntries(Map.entry("FL",-1), Map.entry("FR",-1), Map.entry("BR", 1), Map.entry("BL", 1))),
            Map.entry(DriveDirection.LEFT_TURN,   Map.ofEntries(Map.entry("FL", 1), Map.entry("FR", 1), Map.entry("BR", 1), Map.entry("BL", 1))),
            Map.entry(DriveDirection.RIGHT_TURN,  Map.ofEntries(Map.entry("FL",-1), Map.entry("FR",-1), Map.entry("BR",-1), Map.entry("BL",-1)))
    );

    public Robot(HardwareMap hardwareMap, Telemetry t, Gamepad gamepad1, Gamepad gamepad2)
    {
        try {
            color_sensor = hardwareMap.get(RevColorSensorV3.class, Constants.ColorSensorName);
            if(color_sensor != null) {
                isColorSensor = true;
            }
        }
        catch(Exception ex)
        {

        }
        try {
            headlight = hardwareMap.get(CRServo.class, Constants.HeadlightName);
            if(headlight != null) {
                isHeadlight = true;
            }
        }
        catch(Exception ex)
        {

        }


        for(Map.Entry<String, String> entry: Constants.Drivetrain_Motors.entrySet())
        {
            String motor_flag = entry.getKey();
            String motor_name = entry.getValue();
            DcMotorEx motor_t = hardwareMap.get(DcMotorEx.class, motor_name);
            DriveBase_MotorList.put(motor_flag, motor_t);
            //motor_t.setDirection(DcMotorSimple.Direction.FORWARD);
        }


        this.rightOuttake = hardwareMap.get(DcMotorEx.class, Constants.OuttakeMotor_Right); //expansion hub motor port 1
        this.leftOuttake = hardwareMap.get(DcMotorEx.class, Constants.OuttakeMotor_Left); //expansion hub motor port 2

        this.indexerMotor = hardwareMap.get(DcMotor.class, Constants.Indexer_Motor); //expansion hub motor port 0
        this.intakeMotor = hardwareMap.get(DcMotor.class, Constants.Intake_Motor); //expansion hub motor port 3

        this.limelight = hardwareMap.get(Limelight3A.class, Constants.LimeLightName);
        limelight.pipelineSwitch(0);
        limelight.start();

        this.imu = hardwareMap.get(IMU.class, "imu");
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.t = t;

        rightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightOuttake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        rightOuttake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftOuttake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        t.addLine("A robot is instantialized!!!w");

        t.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        angleCalculatorLeft = new AngleCalculator_Left();
        angleCalculatorRight = new AngleCalculator_Right();
    }

    public Map<String, Integer> getDrivebasePosition()
    {
        HashMap<String, Integer> pos_info = new HashMap<>();
        for(Map.Entry<String, DcMotorEx> entry: DriveBase_MotorList.entrySet())
        {
            String motor_name = entry.getKey();
            DcMotorEx motor_t = entry.getValue();
            pos_info.put(motor_name, motor_t.getCurrentPosition());
        }
        return pos_info;
    }

    public double[] checkLimelight(int tagId)
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw()); //tells limelight what current yaw is
        LLResult llResult = limelight.getLatestResult();

        double aimingAngle = -1000;
        double distance = -1;
        boolean isOn = false;
        if(llResult != null && llResult.isValid())
        {
            Pose3D botPose = llResult.getBotpose_MT2();
            //t.addData("Tx", llResult.getTx());
            //t.addData("Ty", llResult.getTy());
            //t.addLine("--------------------------------------");
            //t.addData("TxNc", llResult.getTxNC());
            //t.addData("TyNc", llResult.getTyNC());
            //t.addLine("--------------------------------------");
            //t.addData("Ta", llResult.getTa()); //basically how much of the camera's view is being used by the april tag
            //t.addLine("--------------------------------------");
            t.addData("Botpose", llResult.getBotpose());

            //telemetry.addData("Pipeline", llResult.getPipelineIndex());

            String tgIdList = "";
            for(LLResultTypes.FiducialResult  t: llResult.getFiducialResults())
            {
                int tagId_t = t.getFiducialId();

                tgIdList = tgIdList + ":" + tagId_t;

                if(tagId_t == tagId)
                {
                    distance = Constants.HeightDiffBetweenAprilTagAndCamera*Math.tan(Math.toRadians(90-llResult.getTy()));
                    aimingAngle = llResult.getTx();
                }
            }
            t.addData("Tag IDs", tgIdList);
            t.addData("Tx", llResult.getTx());
            if(llResult.getTx() >=-9 && llResult.getTx() <= 3)
            {
                //turnon_headlight();
                isOn = true;
            }
            else
            {
                //turnoff_headlight();
            }
        }
        else
        {
            t.addLine("Nothing found!!! ");
        }
        if(isOn)
        {
            turnon_headlight();
        }
        else
        {
            turnoff_headlight();
        }
        return new double[] { distance, aimingAngle };
    }
    public void checkArtifactNum()
    {
        if(indexerMotor.isBusy())
        {
            turnoff_headlight();
            artifactNum = 0;
            artifact_dist1 = Constants.ARTIFACT_DISTANCE_THRESHOLD;
            artifact_dist2 = Constants.ARTIFACT_DISTANCE_THRESHOLD;
        }

        //t.addLine("hihihihihihihihi");
        t.update();
        if(!isColorSensor) return;
        if(!isHeadlight) return;
        double dist = color_sensor.getDistance(DistanceUnit.MM);

        if(dist > 60) return;

        artifact_dist2 = dist;

        if(java.lang.Math.abs(artifact_dist1 - Constants.ARTIFACT_DISTANCE_THRESHOLD)<0.01)
        {
            artifact_dist1 = artifact_dist2;
        }

        if(artifact_dist2 <= artifact_dist1)
        {
            artifact_dist1 = artifact_dist2;
            isArtifactFound = false;
            return;
        }
        else
        {
            if(artifact_dist2 - artifact_dist1 > 3)
            {
                if(!isArtifactFound) {
                    isArtifactFound = true;
                    //artifact_dist2 = Constants.ARTIFACT_DISTANCE_THRESHOLD;
                    //artifact_dist1 = Constants.ARTIFACT_DISTANCE_THRESHOLD;
                    artifactNum++;
                }
            }
        }
        t.addData("counting articficats", artifactNum);
        t.addData("disntance woow 1111", artifact_dist1);
        t.addData("dristsnace wowowwoww 2222", artifact_dist2);

        t.addData("hahahahahaha",dist);
        t.update();

        if(artifactNum >= 3)
        {
            turnon_headlight();
        }

    }
    public void drive() //gamepad 1 left stick and right stick
    {
        int region_left = angleCalculatorLeft.findRegion(gamepad1.left_stick_x, gamepad1.left_stick_y);
        int region_right = angleCalculatorRight.findRegion(gamepad1.right_stick_x, gamepad1.right_stick_y);

        double powerLeft = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)) * POWER_FACTOR;
        double powerRight = Math.sqrt(Math.pow(gamepad1.right_stick_x,2) + Math.pow(gamepad1.right_stick_y, 2)) * POWER_FACTOR;

        if (region_left == 1) {
            start_drivebase(DriveDirection.RIGHT_SHIFT, powerLeft);
        }

        if (region_left == 2) {
            start_drivebase(DriveDirection.BACKWARD, powerLeft);
        }

        if (region_left == 3) {
            start_drivebase(DriveDirection.LEFT_SHIFT, powerLeft);
        }

        if (region_left == 4) {
            start_drivebase(DriveDirection.FORWARD, powerLeft);

        }

        if (region_right == 5) {
            start_drivebase(DriveDirection.RIGHT_TURN, powerRight * 0.75);
        }

        if (region_right == 6) {
            start_drivebase(DriveDirection.LEFT_TURN, powerRight * 0.75);
        }
    }

    public void start_drivebase(DriveDirection direction, double power)
    {
        Map<String, Integer> DRIVE_PARAMETERS = DRIVEBASE_PARAMETERS.get(direction);
        String power_info = "";
        for(Map.Entry<String, DcMotorEx> entry: DriveBase_MotorList.entrySet())
        {
            DcMotorEx motor_t = entry.getValue();
            int direction_sign = 1;
            if(motor_t.getDirection() == DcMotorSimple.Direction.REVERSE) direction_sign=-1;
            double power_t = direction_sign*power*DRIVE_PARAMETERS.get(entry.getKey());
            motor_t.setPower(power_t);
            power_info = power_info + entry.getKey() + ":" + String.format("%.2f", power_t) + "; ";
        }
        t.addData("Motor Powers: ", power_info);
        for(Map.Entry<String, DcMotorEx> entry: DriveBase_MotorList.entrySet())
        {
            DcMotorEx motor_t = entry.getValue();
            motor_t.setPower(0);
            t.addData("encoder", motor_t.getCurrentPosition());
        }
    }

    public void stop_drivebase()
    {
        for(Map.Entry<String, DcMotorEx> entry: DriveBase_MotorList.entrySet())
        {
            DcMotorEx motor_t = entry.getValue();
            motor_t.setPower(0);
        }

    }

    public void turn_drivebase(double turn_angle, DriveDirection direction, double power, double tolerance)
    {
        if(direction != DriveDirection.LEFT_TURN && direction != DriveDirection.RIGHT_TURN) return;

        double start_angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double target_angle = start_angle + turn_angle;

        while(true)
        {
            start_drivebase(direction, power);
            double cur_angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            t.addData("Angle:", "" + target_angle + ":" + cur_angle);
            t.update();

            if(java.lang.Math.abs(cur_angle-target_angle)<tolerance) {
                start_drivebase(direction, 0);
                break;
            }
        }
    }
    public void travel(double distance_inches, double power, DriveDirection direction)
    {
        if(direction != DriveDirection.RIGHT_SHIFT && direction != DriveDirection.LEFT_SHIFT && direction != DriveDirection.FORWARD && direction != DriveDirection.BACKWARD) return;

        final double TICKS_PER_INCH = 100.0/6.0;
        int targetPosition = (int)(distance_inches * TICKS_PER_INCH);

        Map<String, Integer> DRIVE_PARAMETERS = DRIVEBASE_PARAMETERS.get(direction);

        Map<String, DcMotor.RunMode> curRunModeList = new HashMap<>();
        for(Map.Entry<String, DcMotorEx> entry: DriveBase_MotorList.entrySet())
        {
            DcMotorEx motor_t = entry.getValue();
            String motor_name_t = entry.getKey();
            curRunModeList.put(motor_name_t, motor_t.getMode());
        }

        for(Map.Entry<String, DcMotorEx> entry: DriveBase_MotorList.entrySet())
        {
            DcMotorEx  motor_t = entry.getValue();
            motor_t.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        for(Map.Entry<String, DcMotorEx> entry: DriveBase_MotorList.entrySet())
        {
            DcMotorEx motor_t = entry.getValue();
            String motor_name_t = entry.getKey();

            int target_position_t = motor_t.getCurrentPosition() + DRIVE_PARAMETERS.get(motor_name_t) * targetPosition;
            //int target_position_t = motor_t.getCurrentPosition() + targetPosition;

            motor_t.setTargetPosition(target_position_t);
            motor_t.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        start_drivebase(direction, power);

        ElapsedTime timer = new ElapsedTime();
        while(timer.seconds() < 20)
        {

        }

        ElapsedTime timer_t = new ElapsedTime();

        while(isDriveBusy())
        {
            if(timer_t.seconds() > 2) break;
        }

        stop_drivebase();

        for(Map.Entry<String, DcMotorEx> entry: DriveBase_MotorList.entrySet())
        {
            DcMotorEx motor_t = entry.getValue();
            String motor_name_t = entry.getKey();
            motor_t.setMode(curRunModeList.get(motor_name_t));
        }
    }

    public boolean isDriveBusy()
    {
        boolean isBusy = false;
        for(Map.Entry<String, DcMotorEx> entry: DriveBase_MotorList.entrySet())
        {
            DcMotorEx motor_t = entry.getValue();
            if(motor_t.isBusy())
            {
                isBusy = true;
                break;
            }
        }
        return isBusy;
    }
    public void start_flywheel(double targetVelocity)
    {
        rightOuttake.setVelocity(targetVelocity, AngleUnit.DEGREES);
        leftOuttake.setVelocity(targetVelocity, AngleUnit.DEGREES);
    }
    public void stop_flywheel()
    {
        rightOuttake.setPower(0);
        leftOuttake.setPower(0);
    }

    public void show_flywheel_status(double targetVelocity, double[] tolerance)
    {
        double curVelocity1 = rightOuttake.getVelocity(AngleUnit.DEGREES);
        double curVelocity2 = leftOuttake.getVelocity(AngleUnit.DEGREES);

        double error1 = targetVelocity - curVelocity1;
        double error2 = targetVelocity - curVelocity2;

        String color = "Red";

        if(java.lang.Math.abs(error1) < tolerance[0] && java.lang.Math.abs(error2) < tolerance[0] && java.lang.Math.abs(error1-error2) < tolerance[0])
        {
            color = "Green";
        }
        else if(java.lang.Math.abs(error1) < tolerance[1] && java.lang.Math.abs(error2) < tolerance[1] && java.lang.Math.abs(error1-error2) < tolerance[1])
        {
            color = "Yellow";
        }

        t.addData("<font color='" + color + "'>" + "Target Velocity" + "</font>", "<font color='" + color + "'>" + targetVelocity + "</font>");

        t.addData("<font color='" + color + "'>" + "Current Velocity 1" + "</font>", "<font color='" + color + "'>" + curVelocity1 + "</font>");
        t.addData("<font color='" + color + "'>" + "Current Velocity 2" + "</font>", "<font color='" + color + "'>" + curVelocity2 + "</font>");

        t.addData("<font color='" + color + "'>" + "Error 1" + "</font>", "<font color='" + color + "'>" + error1 + "</font>");
        t.addData("<font color='" + color + "'>" + "Error 2" + "</font>", "<font color='" + color + "'>" + error2 + "</font>");

        t.addLine("-----------------------------------");
    }

    public void outtake() //gamepad 2 x and b. (starting and stopping it) gamepad 2 right and left trigger, right and left bumper
    {
        if(gamepad2.right_bumper) {
            cur_FlyWheel_TargetVelocity = HIGH_SPEED;
            indexerHelperSpeed = 0.5;
        }

        if(gamepad2.left_bumper)
        {
            cur_FlyWheel_TargetVelocity = MID_SPEED;
            indexerHelperSpeed = 1;
        }

        if(gamepad2.right_trigger >= 0.25)
        {
            cur_FlyWheel_TargetVelocity = LOW_SPEED;
            indexerHelperSpeed = 1;
        }

            start_flywheel(cur_FlyWheel_TargetVelocity);


        if(gamepad2.bWasPressed())
        {
            stop_flywheel();
            cur_FlyWheel_TargetVelocity = 0;
        }

        show_flywheel_status(cur_FlyWheel_TargetVelocity, new double[] {5, 7});

    }

    public void start_intake()
    {
        intakeMotor.setPower(-1);
    }
    public void stop_intake()
    {
        intakeMotor.setPower(0);
    }
    public void intake()
    {
        if(gamepad2.a == true)
        {
            start_intake();
        }

        if(gamepad2.y == true)
        {
            stop_intake();
        }
        t.addData("intake servo: ", intakeMotor.getPower() + ":" + intakeMotor.getDirection());
    }

    public void start_indexerHelper()
    {
        //artifactNum = 0;
        //artifact_dist1 = Constants.ARTIFACT_DISTANCE_THRESHOLD;
        //artifact_dist2 = Constants.ARTIFACT_DISTANCE_THRESHOLD;
        //turnoff_headlight();

        //indexerMotor.setPower(indexerHelperSpeed);
        start_indexerHelper(indexerHelperSpeed);
    }


    public void start_indexerHelper(double speedIndexerHelper)
    {
        artifactNum = 0;
        artifact_dist1 = Constants.ARTIFACT_DISTANCE_THRESHOLD;
        artifact_dist2 = Constants.ARTIFACT_DISTANCE_THRESHOLD;
        turnoff_headlight();
        indexerMotor.setPower(speedIndexerHelper);
    }

    public void turnon_headlight()
    {
        if(headlight != null) {
            headlight.setPower(0);
        }

    }
    public void turnoff_headlight()
    {
        //headlight.setPower(-1);
        if(headlight != null) {
            headlight.setPower(-1);
        }
    }
    public double getHeadLightBrightness()
    {
        double brightness = -1000;
        if(headlight != null)
            brightness =  headlight.getPower();
        return brightness;
    }
    public void setHeadLightBrightness(double brightness)
    {
        if(headlight != null)
            headlight.setPower(brightness);
    }


    public void stop_IndexerHelper()
    {
        indexerMotor.setPower(0);
    }

    public void indexerHelper()
    {
        if(gamepad2.dpad_left == true)
        {
            start_indexerHelper();
        }


        if(gamepad2.dpad_right == true)
        {
            stop_IndexerHelper();
        }
    }

    public void start_spitOutBalls()
    {
        intakeMotor.setPower(0.25);

        ElapsedTime time_x = new ElapsedTime();
        while(time_x.seconds() > 1)
        {
            indexerMotor.setPower(-0.25);
        }
    }

    public void stop_spitOutBalls()
    {
        intakeMotor.setPower(0);
        indexerMotor.setPower(0);
    }

    public void spitOutBalls()
    {
        if(gamepad2.dpadUpWasPressed())
        {
            start_spitOutBalls();
        }

        if(gamepad2.dpadDownWasPressed())
        {
            stop_spitOutBalls();
        }
    }

    public void stop_all()
    {
        stop_drivebase();
        stop_flywheel();
        stop_intake();
        stop_IndexerHelper();
    }
}