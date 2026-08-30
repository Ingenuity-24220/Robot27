package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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

import java.util.Locale;

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
    TelemetryManager panelsTelemetry;


//    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        motorLF = hardwareMap.get(DcMotor.class, "motorLF"); // left front
        motorLR = hardwareMap.get(DcMotor.class, "motorLR"); // left rear
        motorRF = hardwareMap.get(DcMotor.class, "motorRF"); // right front
        motorRR = hardwareMap.get(DcMotor.class, "motorRR"); // right rear
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

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
        double yaw = -gamepad1.right_stick_x;

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

        double rawYaw = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

        double robotYaw = AngleUnit.normalizeDegrees(rawYaw + 180.0);
        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();

        panelsTelemetry.addData("IMU yaw sent to MT2", String.format(Locale.US, "%.1f deg", robotYaw));
        if (result != null && result.isValid()) {
            Pose3D botpose_mt1 = result.getBotpose();
            Pose3D botpose_mt2 = result.getBotpose_MT2();

            panelsTelemetry.addData("MT1 pose", formatPose(botpose_mt1));
            panelsTelemetry.addData("MT2 pose", formatPose(botpose_mt2));

            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        } else {
            panelsTelemetry.addData("MT1 pose", "unavailable");
            panelsTelemetry.addData("MT2 pose", "unavailable");
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

        panelsTelemetry.update(telemetry);
    }

    private String formatPose(Pose3D pose) {
        if (pose == null) {
            return "unavailable";
        }

        return String.format(
                Locale.US,
                "x=%.3f m, y=%.3f m, z=%.3f m, yaw=%.1f deg",
                pose.getPosition().x,
                pose.getPosition().y,
                pose.getPosition().z,
                pose.getOrientation().getYaw(AngleUnit.DEGREES)
        );
    }
}
