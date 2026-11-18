package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.Outtake;

@TeleOp(name="outtakeTesting")
public class outtakeTesting extends OpMode {

    Outtake outtake;
    TelemetryManager panelsTelemetry;
    @Override
    public void init() {
        //panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        //outtake = new Outtake(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
            /*telemetry.addData("power", outtake.getPower());
            if (gamepad1.dpadUpWasPressed()) {
                outtake.powerUp();
            }
            if (gamepad1.dpadDownWasPressed()) {
                outtake.powerDown();
            }*/
    }
}
