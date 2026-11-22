package org.firstinspires.ftc.teamcode;

import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.LLStatus;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Localization;
import org.firstinspires.ftc.teamcode.util.Outtake;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

//import java.util.Locale;

//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("unused")
@TeleOp(name="meow")
public class meow extends OpMode {
    TelemetryManager panelsTelemetry;

    Drivetrain drivetrain;
    Intake intake;

    Localization localizer;

    Outtake outtake;

    DataLogger log;

    final ElapsedTime clock = new ElapsedTime();

    static final int BALLS_PER_VOLLEY = 4;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        drivetrain = new Drivetrain(hardwareMap, telemetry, panelsTelemetry, false);
        intake = new Intake(hardwareMap);
        localizer = new Localization(hardwareMap, telemetry, panelsTelemetry);
        localizer.recalibrateOdometry(true);
        outtake = new Outtake(hardwareMap, telemetry);
        log = new DataLogger("meow",
                "t", "nectar_gate_open", "pollen_gate_open", "intake_spinning",
                "nectar_speed", "pollen_speed", "axial", "lateral", "yaw");
    }

    @Override
    public void loop() {
        double now = clock.seconds();

//        Gamepad g1 = PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1);

        // preferably don't do this but for now as we would like to have the gamepad 2 keep this in
        @SuppressWarnings("unused")
//        Gamepad g2 = PanelsGamepad.INSTANCE.getSecondManager().asCombinedFTCGamepad(gamepad2);

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x; // Drivetrain.drive() treats +yaw as clockwise (right)

        drivetrain.drive(axial, lateral, yaw);

        telemetry.addData("Intake spinning", intake.isRunning);
        panelsTelemetry.addData("Intake spinning", intake.isRunning);

        telemetry.addData("Nectar gate open", outtake.isNectarOpen);
        telemetry.addData("Pollen gate open", outtake.isPollenOpen);


        if (gamepad1.aWasPressed()) {
            if (intake.isRunning) {
                intake.stop();
            } else {
                intake.start();
            }
        }

        if (gamepad1.bWasPressed()) {
            intake.reverse();
        }

        // Bumpers fire a volley of BALLS_PER_VOLLEY balls, one gate pulse per ball (see Outtake.fire).
        // Press again during a volley to cancel it.
        if (gamepad1.rightBumperWasPressed()) {
            outtake.fire("pollen", BALLS_PER_VOLLEY);
        }

        if (gamepad1.leftBumperWasPressed()) {
            outtake.fire("nectar", BALLS_PER_VOLLEY);
        }

        outtake.update();

        if (gamepad1.xWasPressed()) {
            if (outtake.isNectarRunning) {
                outtake.stop("nectar");
            } else {
                outtake.startAtSpeed("nectar");
            }
        }

        if (gamepad1.yWasPressed()) {
            if (outtake.isPollenRunning) {
                outtake.stop("pollen");
            } else {
                outtake.startAtSpeed("pollen");
            }
        }

        localizer.runOdometry();

        telemetry.update();

        log.log(now, outtake.isNectarOpen ? 1 : 0, outtake.isPollenOpen ? 1 : 0, intake.isRunning ? 1 : 0, outtake.velocity("nectar"), outtake.velocity("pollen"), axial, lateral, yaw);
//                                "t", "nectar_gate_open", "pollen_gate_open", "intake_spinning",
//                "nectar_speed", "pollen_speed", "axial", "lateral", "yaw");

    }

    @Override
    public void stop() {
        outtake.close("nectar");
        outtake.close("pollen");
        outtake.stop("nectar");
        outtake.stop("pollen");
        intake.stop();
        log.close();
    }
}
