package org.firstinspires.ftc.teamcode;

//import androidx.annotation.NonNull;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {
0000
 */
@TeleOp(name="boo", group="robot 2025-2026")
@Disabled
public class TeleOp2025_obsolete extends LinearOpMode
{
    //private DcMotorEx test1 = null;

    //private DigitalChannel touch_sensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");

    ElapsedTime time_x;
    //private CRServo headlight;
    private TouchSensor touch_sensor;
    private RevColorSensorV3 color_sensor;
    @Override
    public void runOpMode() {


        time_x = new ElapsedTime();
        telemetry.addData("hello1", "hello");
        telemetry.update();


        //headlight = hardwareMap.get(CRServo.class, "headlight");
        touch_sensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
        color_sensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        //double xyz = headlight.getPower();


        //test1 = hardwareMap.get(DcMotorEx.class, "dumb");

        double power = -0.4;
        //test1.setPower(power);

        //telemetry.addData("hello1:power=", xyz);
        telemetry.update();

        //headlight.setPower(-0.9);
        waitForStart();

        int cnt = 0;
        while (opModeIsActive()) {
            //double power = 0.1;
            //test1.setPower(power);
            //touch_sensor.
            boolean isPressed = touch_sensor.isPressed();
            double x = touch_sensor.getValue();
            if(isPressed)
            {
                power = power + 0.02;
                //headlight.setPower(power);
                time_x.reset();
                while(time_x.seconds()<2)
                {
                    idle();
                }
            }

            if(power > -2.0)
            {
                power = -0.4;
            }
            telemetry.addData("Touch Sensor", "" + isPressed + ":" + cnt + ":" + power + ":" + x);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", power, power);

            telemetry.addData("RGB = ", "" + color_sensor.red() + ":" + color_sensor.green() + ":" + color_sensor.blue());
            //telemetry.addData("distance=", color_sensor.)
            telemetry.addLine("hello hello:" + color_sensor.getDistance(DistanceUnit.MM));
            telemetry.update();

            cnt++;
            //wait(1000);
            //wait(10000);
        }
    }

}
