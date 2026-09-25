package org.firstinspires.ftc.teamcode;

import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Localization;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import java.util.Locale;

//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="meow")
public class meow extends OpMode {
    TelemetryManager panelsTelemetry;

    Drivetrain drivetrain;
    Intake intake;

    Localization localizer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.drivetrain = new Drivetrain(hardwareMap, telemetry, panelsTelemetry, false);
        this.intake = new Intake(hardwareMap);
        localizer = new Localization(hardwareMap, telemetry, panelsTelemetry);
        localizer.recalibrateOdometry(true);
    }

    @Override
    public void loop() {
        Gamepad g1 = PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1);
        Gamepad g2 = PanelsGamepad.INSTANCE.getSecondManager().asCombinedFTCGamepad(gamepad2);

        double axial = -g1.left_stick_y;
        double lateral = g1.left_stick_x;
        double yaw = -g1.right_stick_x;

        drivetrain.drive(axial, lateral, yaw);

        telemetry.addData("Intake spinning:", intake.isRunning);
        panelsTelemetry.addData("Intake spinning:", intake.isRunning);
        if (g1.aWasPressed()) {
            if (intake.isRunning) {
                intake.stop();
            } else {
                intake.start();
            }
        }

        if (g1.bWasPressed()) {
            intake.reverse();
        }

        localizer.runOdometry();

        telemetry.update();

    }
}
