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

    // Bumpers: true = fire a volley (one gate pulse per ball); false = gate open while held.
    // Toggle during INIT with X.
    boolean useVolleys = true;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        drivetrain = new Drivetrain(hardwareMap, telemetry, panelsTelemetry, false);
        intake = new Intake(hardwareMap);
        localizer = new Localization(hardwareMap, telemetry, panelsTelemetry);
        localizer.recalibrateOdometry(true);
        outtake = new Outtake(hardwareMap, telemetry);
        outtake.setPreset(0); // close
        log = new DataLogger("meow",
                "t", "nectar_gate_open", "pollen_gate_open", "intake_spinning",
                "nectar_speed", "pollen_speed", "axial", "lateral", "yaw");
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) useVolleys = !useVolleys;
        telemetry.addData("Bumpers (X to change)", useVolleys
                ? "volley (nectar " + outtake.patternText("nectar") + ", pollen "
                        + outtake.patternText("pollen") + ")" : "gate open while held");
        telemetry.update();
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
        telemetry.addData("Bumpers", useVolleys ? "volley" : "hold to open");
        telemetry.addData("Unjam (D-pad L nectar / R pollen)", outtake.isUnjamming("nectar") ? "NECTAR"
                : outtake.isUnjamming("pollen") ? "POLLEN" : "-");
        telemetry.addData("Speed preset (D-pad Up/Down)", Outtake.PRESET_NAMES[outtake.preset()]
                + String.format(java.util.Locale.US, "  nectar %.0f / pollen %.0f",
                outtake.targetVelocity("nectar"), outtake.targetVelocity("pollen")));


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

        if (useVolleys) {
            // Bumpers fire a volley: the flywheel's pulse pattern, e.g. pollen 1-1-2 (see Outtake.fire).
            // Press again during a volley to cancel it.
            if (gamepad1.rightBumperWasPressed()) outtake.fire("pollen");
            if (gamepad1.leftBumperWasPressed()) outtake.fire("nectar");
        } else {
            // Gate open while the bumper is held.
            if (gamepad1.rightBumperWasPressed()) outtake.open("pollen");
            if (gamepad1.rightBumperWasReleased()) outtake.close("pollen");
            if (gamepad1.leftBumperWasPressed()) outtake.open("nectar");
            if (gamepad1.leftBumperWasReleased()) outtake.close("nectar");
        }

        // Unjam: D-pad Left = nectar, D-pad Right = pollen (same sides as LB / RB). Swings the
        // gate past open and back for about a second; press again to stop.
        if (gamepad1.dpadLeftWasPressed()) outtake.unjam("nectar");
        if (gamepad1.dpadRightWasPressed()) outtake.unjam("pollen");

        outtake.update();

        // Shooting speed preset: D-pad Up = farther, D-pad Down = closer (close / medium / far).
        if (gamepad1.dpadUpWasPressed()) outtake.setPreset(outtake.preset() + 1);
        if (gamepad1.dpadDownWasPressed()) outtake.setPreset(outtake.preset() - 1);

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
