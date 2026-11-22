package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class Outtake {
    HardwareMap hardwareMap;

    Telemetry telemetry;
    TelemetryManager panelsTelemetry;

    DcMotor pollenMotor;
    DcMotor nectarMotor;

    Servo pollenServo;
    Servo nectarServo;

    public boolean isNectarOpen = false;
    public boolean isPollenOpen = false;

    public boolean isNectarRunning = false;
    public boolean isPollenRunning = false;


    public Outtake(HardwareMap hardwareMap, Telemetry telemetry/*, TelemetryManager panelsTelemetry*/){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        pollenMotor = hardwareMap.get(DcMotor.class, "outtakeMini");
        nectarMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        pollenServo = hardwareMap.get(Servo.class, "pollenServo");
        nectarServo = hardwareMap.get(Servo.class, "nectarServo");

        pollenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.close("nectar");
        this.close("pollen");
    }
    public void start(String type, double power) {
        if (Objects.equals(type, "nectar")) {
            nectarMotor.setPower(power); // 0.62
            this.isNectarRunning = true;
        } else {
            pollenMotor.setPower(power); // 0.725
            this.isPollenRunning = true;
        }
    }
    public void stop(String type) {
        if (Objects.equals(type, "nectar")) {
            nectarMotor.setPower(0);
            this.isNectarRunning = false;
        } else {
            pollenMotor.setPower(0);
            this.isPollenRunning = false;
        }
    }

    public void open(String type) {
        if (Objects.equals(type, "nectar")) {
            nectarServo.setPosition(0.5);
            this.isNectarOpen = true;
        } else {
            pollenServo.setPosition(0);
            this.isPollenOpen = true;
        }
    }

    public void close(String type) {
        if (Objects.equals(type, "nectar")) {
            nectarServo.setPosition(0.85);
            this.isNectarOpen = false;
        } else {
            pollenServo.setPosition(0.25);
            this.isPollenOpen = false;
        }
    }

}
