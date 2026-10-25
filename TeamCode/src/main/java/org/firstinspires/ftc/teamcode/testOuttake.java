package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Intake;
//import org.firstinspires.ftc.teamcode.util.Localization;

@TeleOp(name="testOuttake8u8i")
public class testOuttake extends OpMode {
    DcMotor Outtake;

    double currentPower;



    @Override
    public void init() {
        Outtake = hardwareMap.get(DcMotor.class, "outtake");
    }

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed()) {
            if (Outtake.getPower() >= 1) {
                ;
            } else {
                Outtake.setPower(Outtake.getPower()+.01);
            }
        }
        if (gamepad1.dpadDownWasPressed()) {
            if (Outtake.getPower() <= 0) {
                ;
            } else {
                Outtake.setPower(Outtake.getPower()-.01);
            }        }
    }

}
