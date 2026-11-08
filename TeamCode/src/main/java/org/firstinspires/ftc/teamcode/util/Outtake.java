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

    public Outtake(HardwareMap hardwareMap, Telemetry telemetry/*, TelemetryManager panelsTelemetry*/){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        //this.panelsTelemetry = panelsTelemetry;
        this.outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
    }
    public void start() {
        isRunning = true;
        outtakeMotor.setPower(1);
    }
    public void stop() {
        isRunning = false;
        outtakeMotor.setPower(0);
    }
    public double getPower(){
        return outtakeMotor.getPower();
    }
}
