package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="PID?????")
@Disabled
public class FlywheelTunerTutorial extends OpMode {
    public DcMotorEx flywheelMotor1;
    public DcMotorEx flywheelMotor2;

    public double highVelocity = 780;
    public double lowVelocity = 720;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {20.0, 10.0, 5.0, 1.0, 0.1};

    int stepIndex = 0;

    DcMotor intakeMotor;
    DcMotor toilet;


    @Override
    public void init()
    {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "dumb1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "dumb2");

        flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        this.toilet = hardwareMap.get(DcMotor.class, "toilet");
        this.intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        telemetry.addLine("Init complete");

    }

    @Override
    public void loop()
    {
        //get all our gamepad commands
        //set target velocity
        //update telemetry

        if(gamepad2.a == true)
        {
            intakeMotor.setPower(-1);
        }

        if(gamepad2.y == true)
        {
            intakeMotor.setPower(0);
        }

        //telemetry.addData("intake servo: ", intakeMotor.getPower() + ":" + intakeMotor.getDirection());

        if(gamepad2.dpad_left == true)
        {
            toilet.setPower(1);
        }

        if(gamepad2.dpad_right == true)
        {
            toilet.setPower(0);
        }

        if(gamepad2.dpad_down == true)
        {
            toilet.setPower(0.25);
        }




        if(gamepad1.yWasPressed())
        {
            if(curTargetVelocity == highVelocity)
            {
                curTargetVelocity = lowVelocity;
            } else {curTargetVelocity = highVelocity;}}

        if(gamepad1.bWasPressed())
        {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed())
        {
            F += stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed())
        {
            F -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed())
        {
            P += stepSizes[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed())
        {
            P -= stepSizes[stepIndex];
        }

        //set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set velocity
        flywheelMotor1.setVelocity(curTargetVelocity);
        flywheelMotor2.setVelocity(curTargetVelocity);

        double curVelocity1 = flywheelMotor1.getVelocity();
        double curVelocity2 = flywheelMotor2.getVelocity();

        double error1 = curTargetVelocity - curVelocity1;
        double error2 = curTargetVelocity - curVelocity2;

        telemetry.addData("Target Velocity", curTargetVelocity);

        telemetry.addData("Current Velocity 1", "%.2f", curVelocity1);
        telemetry.addData("Current Velocity 2", "%.2f", curVelocity2);

        telemetry.addData("Error 1", "%.2f", error1);
        telemetry.addData("Error 2", "%.2f", error2);

        telemetry.addLine("-----------------------------------");

        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);

        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

        telemetry.speak("BOOOOOOOOOOOO AHAHAHAHHBH");
        telemetry.update();
    }
}
