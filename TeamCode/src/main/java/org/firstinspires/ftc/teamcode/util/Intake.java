package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    DcMotor motorIntake;

//    Servo intakeGateOne;
//    Servo intakeGateTwo;

//    boolean isReversed = false;

    public boolean isRunning = false;

    public boolean isGateOpen = false;

    public Intake(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotor.class, "intakeMotor"); // intake motor
//        intakeGateOne = hardwareMap.get(Servo.class, "intakeGateOne");
//        intakeGateTwo = hardwareMap.get(Servo.class, "intakeGateTwo");
//
//        intakeGateOne.setDirection(Servo.Direction.FORWARD);
//        intakeGateTwo.setDirection(Servo.Direction.REVERSE);
    }

    public void start() {
        motorIntake.setPower(-.75);
        isRunning = true;
    }

    public void stop() {
        motorIntake.setPower(0);
        isRunning = false;
    }

    public void reverse() {
        if (motorIntake.getDirection() == DcMotorSimple.Direction.FORWARD) {
            motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void openGate() {
        isGateOpen = true;
//        intakeGateOne.setPosition(1.0);
//        intakeGateTwo.setPosition(1.0);
    }

    public void closeGate() {
        isGateOpen = false;
//        intakeGateOne.setPosition(0);
//        intakeGateTwo.setPosition(0);
    }
}
