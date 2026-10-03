package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    HardwareMap hardwareMap;

    Telemetry telemetry;
    TelemetryManager panelsTelemetry;

    DcMotor outtakeMotor;

    public Outtake(HardwareMap hardwareMap, Telemetry telemetry/*, TelemetryManager panelsTelemetry*/){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        //this.panelsTelemetry = panelsTelemetry;
        this.outtakeMotor = hardwareMap.get(DcMotor.class, "outtake");

    }
    public void powerUp() {
        if(outtakeMotor.getPower()<1) {
            outtakeMotor.setPower(outtakeMotor.getPower() + 0.05);
        }
    }
    public void powerDown() {
        if(outtakeMotor.getPower()>-1){
            outtakeMotor.setPower(outtakeMotor.getPower() - 0.05);
        }
    }
    public double getPower(){
        return outtakeMotor.getPower();
    }
}
