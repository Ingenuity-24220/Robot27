package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="meow")
public class meow extends OpMode {

    DcMotor motorLF;
    DcMotor motorLR;
    DcMotor motorRF;
    DcMotor motorRR;

//    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        motorLF = hardwareMap.get(DcMotor.class, "motorLF"); // left front
        motorLR = hardwareMap.get(DcMotor.class, "motorLR"); // left rear
        motorRF = hardwareMap.get(DcMotor.class, "motorRF"); // right front
        motorRR = hardwareMap.get(DcMotor.class, "motorRR"); // right rear

        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
//      double MAX_POWER = 1.0;
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double LFPower  = axial + lateral + yaw;
        double RFPower = axial - lateral - yaw;
        double LRPower   = axial - lateral + yaw;
        double RRPower  = axial + lateral - yaw;

        double max = Math.max(Math.abs(LFPower), Math.abs(LRPower));
        max = Math.max(max, Math.abs(RFPower));
        max = Math.max(max, Math.abs(RRPower));

        if (max > 1.0) {
            LFPower /= max;
            LRPower /= max;
            RFPower /= max;
            RRPower /= max;
        }

        motorLF.setPower(LFPower);
        motorLR.setPower(LRPower);
        motorRF.setPower(RFPower);
        motorRR.setPower(RRPower);
    }
}
