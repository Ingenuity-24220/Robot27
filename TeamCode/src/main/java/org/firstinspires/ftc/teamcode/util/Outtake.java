package org.firstinspires.ftc.teamcode.util;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    TelemetryManager panelsTelemetry;

    DcMotor outtakeMotor;

    public boolean isRunning = false;

    public double power = 0.85;

    public Outtake(HardwareMap hardwareMap, Telemetry telemetry, String motorName/*, TelemetryManager panelsTelemetry*/){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        //this.panelsTelemetry = panelsTelemetry;
        this.outtakeMotor = hardwareMap.get(DcMotor.class, motorName);
    }
    public void setPower(double power){
        this.power = power < 0 ? 0 : power;
        outtakeMotor.setPower(power);
        this.isRunning = (power != 0);
    }
    public double getPower(){
        return outtakeMotor.getPower();
    }
}
