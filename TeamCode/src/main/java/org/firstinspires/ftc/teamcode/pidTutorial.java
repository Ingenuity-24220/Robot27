package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp(name="test_code")
public class pidTutorial extends LinearOpMode {

    //DcMotorEx motor1;
    //DcMotorEx motor2;


    ElapsedTime timer = new ElapsedTime();
    //CRServo headlight = null;
    Servo headlight = null;

    GoBildaPinpointDriver odo = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //motor1 = hardwareMap.get(DcMotorEx.class, "dumb1");
        //motor2 = hardwareMap.get(DcMotorEx.class,"dumb2");
        //motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        headlight = hardwareMap.get(Servo.class, Constants.HeadlightName);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PinPointName);
        double brightness = -1;
        waitForStart();
        while(opModeIsActive())
        {
            //double power1 = PIDControl(2100, motor1.getVelocity());
            //double power2 = PIDControl(2100, motor2.getCurrentPosition());

            //motor1.setPower(power1);
            //motor2.setPower(power2);
            if(gamepad1.aWasPressed())
            {
                brightness += 0.1;
                if(brightness > 1.0) brightness = -1.0;
                telemetry.addData("Brightness", "" + brightness + ":" + odo.getYawScalar());
                telemetry.update();
                //headlight.setPower(brightness);
                headlight.setPosition(brightness);

            }
        }
    }
}
