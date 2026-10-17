package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    DcMotor motorIntake;

//    boolean isReversed = false;

    public boolean isRunning = false;
    public Intake(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotor.class, "intakeMotor"); // intake motor
    }

    public void start() {
        motorIntake.setPower(-1);
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

}
