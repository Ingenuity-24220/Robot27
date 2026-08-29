package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="meow")
public class meow extends OpMode {

    DcMotor motorLF;
    DcMotor motorLR;
    DcMotor motorRF;
    DcMotor motorRR;

    int lastMotorLF = 0;
    int lastMotorLR = 0;
    int lastMotorRF = 0;
    int lastMotorRR = 0;

    Limelight3A limelight;

    IMU imu;


//    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        motorLF = hardwareMap.get(DcMotor.class, "motorLF"); // left front
        motorLR = hardwareMap.get(DcMotor.class, "motorLR"); // left rear
        motorRF = hardwareMap.get(DcMotor.class, "motorRF"); // right front
        motorRR = hardwareMap.get(DcMotor.class, "motorRR"); // right rear
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRR.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0); // AKA detection

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection));

        imu.initialize(parameters);
    }

    @Override
    public void loop() {
//      double MAX_POWER = 1.0;
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double LFPower  = axial + lateral + yaw;
        double RFPower = axial - lateral - yaw;
        double LRPower   = axial - lateral + yaw;
        double RRPower  = axial + lateral - yaw;

        double max = Math.max(Math.abs(LFPower), Math.abs(LRPower));
        max = Math.max(max, Math.abs(RFPower));
        max = Math.max(max, Math.abs(RRPower));

        if (max > 1.0) {
            LFPower /= max;
            LRPower /= max;
            RFPower /= max;
            RRPower /= max;
        }

        LLResult result = limelight.getLatestResult();
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }

        motorLF.setPower(LFPower);
        motorLR.setPower(LRPower);
        motorRF.setPower(RFPower);
        motorRR.setPower(RRPower);

        telemetry.addData("LF Motor", LFPower);
        telemetry.addData("LR Motor", LRPower);
        telemetry.addData("RF Motor", RFPower);
        telemetry.addData("RR Motor", RRPower);

        telemetry.addData("LF Motor", motorLF.getCurrentPosition());
        telemetry.addData("LR Motor", motorLR.getCurrentPosition());
        telemetry.addData("RF Motor", motorRF.getCurrentPosition());
        telemetry.addData("RR Motor", motorRR.getCurrentPosition());

        telemetry.addData("LF Motor Change", Math.abs(motorLF.getCurrentPosition() - lastMotorLF));
        telemetry.addData("LR Motor Change", Math.abs(motorLR.getCurrentPosition() - lastMotorLR));
        telemetry.addData("RF Motor Change", Math.abs(motorRF.getCurrentPosition() - lastMotorRF));
        telemetry.addData("RR Motor Change", Math.abs(motorRR.getCurrentPosition() - lastMotorRR));

        lastMotorLF = motorLF.getCurrentPosition();
        lastMotorLR = motorLR.getCurrentPosition();
        lastMotorRF = motorRF.getCurrentPosition();
        lastMotorRR = motorRR.getCurrentPosition();

        telemetry.update();
    }
}
