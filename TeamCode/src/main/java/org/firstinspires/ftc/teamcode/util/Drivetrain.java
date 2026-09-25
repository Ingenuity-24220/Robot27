package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

public class Drivetrain {

    TelemetryManager panelsTelemetry;

    DcMotor motorLF;
    DcMotor motorLR;
    DcMotor motorRF; // right front
    DcMotor motorRR; // right rear

//    Limelight3A limelight;

    IMU imu;

    double currLFPower;
    double currRFPower;
    double currRRPower;
    double currLRPower;

    boolean debug;

    Telemetry telemetry;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry, TelemetryManager panelsTelemetry, boolean debug) {
        this.telemetry = telemetry;
        this.debug = debug;
        this.panelsTelemetry = panelsTelemetry;
        motorLF = hardwareMap.get(DcMotor.class, "motorLF"); // left front
        motorLR = hardwareMap.get(DcMotor.class, "motorLR"); // left rear
        motorRF = hardwareMap.get(DcMotor.class, "motorRF"); // right front
        motorRR = hardwareMap.get(DcMotor.class, "motorRR"); // right rear
        imu = hardwareMap.get(IMU.class, "imu");
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRR.setDirection(DcMotorSimple.Direction.REVERSE);

//        limelight.setPollRateHz(100);
//        limelight.start();
//
//        limelight.pipelineSwitch(0); // AKA detection

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection));

        imu.initialize(parameters);
    }

    public void drive(double axial, double lateral, double yaw) {
        double targetLFPower = axial + lateral + yaw; // this is what we want to hit
        double targetRFPower = axial - lateral - yaw;
        double targetLRPower = axial - lateral + yaw;
        double targetRRPower = axial + lateral - yaw;


        double max = Math.max(Math.abs(targetLFPower), Math.abs(targetLRPower));
        max = Math.max(max, Math.abs(targetRFPower));
        max = Math.max(max, Math.abs(targetRRPower));

        if (max > 1.0) {
            targetLFPower /= max;
            targetLRPower /= max;
            targetRFPower /= max;
            targetRRPower /= max;
        }

        currLFPower += (targetLFPower - currLFPower) * 0.05;
        currLRPower += (targetLRPower - currLRPower) * 0.05;
        currRFPower += (targetRFPower - currRFPower) * 0.05;
        currRRPower += (targetRRPower - currRRPower) * 0.05;

        double rawYaw = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

        double robotYaw = AngleUnit.normalizeDegrees(rawYaw + 180.0);
//        limelight.updateRobotOrientation(robotYaw);
//        LLResult result = limelight.getLatestResult();

        if (debug) {
            panelsTelemetry.addData("IMU yaw sent to MT2", String.format(Locale.US, "%.1f deg", robotYaw));
        }
//        if (result != null && result.isValid()) {
//            Pose3D botpose_mt1 = result.getBotpose();
//            Pose3D botpose_mt2 = result.getBotpose_MT2();
//
//            if (debug) {
//                panelsTelemetry.addData("MT1 pose", formatPose(botpose_mt1));
//                panelsTelemetry.addData("MT2 pose", formatPose(botpose_mt2));
//            }
//
//            if (botpose_mt2 != null) {
//                double x = botpose_mt2.getPosition().x;
//                double y = botpose_mt2.getPosition().y;
//                if (debug) {
//                    panelsTelemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
//                }
//            }
//        } else {
//            if (debug) {
//                panelsTelemetry.addData("MT1 pose", "unavailable");
//                panelsTelemetry.addData("MT2 pose", "unavailable");
//            }
//        }

        motorLF.setPower(currLFPower);
        motorLR.setPower(currLRPower);
        motorRF.setPower(currRFPower);
        motorRR.setPower(currRRPower);

        if (debug) {
            telemetry.addData("Target LF Motor", targetLFPower);
            telemetry.addData("Target LR Motor", targetLRPower);
            telemetry.addData("Target RF Motor", targetRFPower);
            telemetry.addData("Target RR Motor", targetRRPower);

            telemetry.addData("Curr LF Motor", currLFPower);
            telemetry.addData("Curr LR Motor", currLRPower);
            telemetry.addData("Curr RF Motor", currRFPower);
            telemetry.addData("Curr RR Motor", currRRPower);

            panelsTelemetry.update(telemetry);
        }
    }

//    private String formatPose(Pose3D pose) {
//        if (pose == null) {
//            return "unavailable";
//        }
//
//        return String.format(
//                Locale.US,
//                "x=%.3f m, y=%.3f m, z=%.3f m, yaw=%.1f deg",
//                pose.getPosition().x,
//                pose.getPosition().y,
//                pose.getPosition().z,
//                pose.getOrientation().getYaw(AngleUnit.DEGREES)
//        );
//    }
}
