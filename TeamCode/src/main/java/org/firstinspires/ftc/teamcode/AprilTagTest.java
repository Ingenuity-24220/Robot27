package org.firstinspires.ftc.teamcode;
import android.util.Size;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.util.Constants;

@Autonomous(name = "AprilTagTest")
public class AprilTagTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    GoBildaPinpointDriver odo;

    //Robot du = null;

    //CRServo limelightSpin;

    int cnt = 0;

    private void initAprilTag()
    {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(1439.42, 1439.42, 970.514, 537.613)
                .build();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "limelight"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(aprilTagProcessor)
                .build();
    }
    @Override
    public void init()
    {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PinPointName);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelightSpin = hardwareMap.get(CRServo.class, "spinner");
        boolean pipeline = limelight.pipelineSwitch(0); //april tag #20
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        //initAprilTag();
        //du = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        //Constants constant = new Constants();

        //odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    @Override
    public void start()
    {
        //odo.resetPosAndIMU();
        limelight.start(); //if theres a delay put it in init()
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        cnt++;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw()); //tells limelight what current yaw is
        LLResult llResult = limelight.getLatestResult();
        if(llResult != null && llResult.isValid())
        {
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addLine("--------------------------------------");
                telemetry.addData("TxNc", llResult.getTxNC());
                telemetry.addData("TyNc", llResult.getTyNC());
                telemetry.addLine("--------------------------------------");
                telemetry.addData("Ta", llResult.getTa()); //basically how much of the camera's view is being used by the april tag
                telemetry.addLine("--------------------------------------");
                telemetry.addData("Botpose", llResult.getBotpose());

            //telemetry.addData("Pipeline", llResult.getPipelineIndex());

                String tgIdList = "";
                for(LLResultTypes.FiducialResult  t: llResult.getFiducialResults())
                {
                    int tagId = t.getFiducialId();
                    tgIdList = tgIdList + ":" + tagId;
                }
                telemetry.addData("Tag IDs", tgIdList);
        }
        else
        {
            telemetry.addLine("Nothing found!!!: " + cnt);
            //telemetry.update();
        }

        /*
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for(AprilTagDetection detection: detections)
        {
            if((detection.metadata) != null)
            {
                telemetry.addLine("AprilTag=" + detection.id);
            }
        }
        */
        odo.update();
        telemetry.addData("xxxxx", odo.getPosX(DistanceUnit.INCH) + " : " + odo.getEncoderX());
        telemetry.addData("yyyyy", odo.getPosY(DistanceUnit.INCH) + " : " + odo.getEncoderY());
        telemetry.addData("wowowowo direction", odo.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        //if(java.lang.Math.abs(llResult.getTx()) > 5) {
        //    du.start_drivebase(DriveDrection.LEFT_TURN, 0.1);
        //}
        //else
        //{
        //    du.start_drivebase(DriveDrection.LEFT_SHIFT, 0);
        //}
    }
}
