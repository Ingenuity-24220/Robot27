package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

public class Localization {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    Telemetry telemetry;

    HardwareMap hardwareMap;

    TelemetryManager panelsTelemetry;

    ElapsedTime timer = new ElapsedTime();
    double oldTime = 0;

    public Localization(HardwareMap hardwareMap, Telemetry telemetry, TelemetryManager panelsTelemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        this.panelsTelemetry = panelsTelemetry;
        odo.setOffsets(-180.0, -30.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1


        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

    }

    public void recalibrateOdometry(Boolean reset) {
        if (reset) {
            odo.resetPosAndIMU();
            panelsTelemetry.addData("Status", "Initialized");
            panelsTelemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
            panelsTelemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
            panelsTelemetry.addData("Device Version Number:", odo.getDeviceVersion());
            panelsTelemetry.addData("Heading Scalar", odo.getYawScalar());
            panelsTelemetry.update();

            telemetry.addData("Status", "Initialized");
            telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
            telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
            telemetry.addData("Device Version Number:", odo.getDeviceVersion());
            telemetry.addData("Heading Scalar", odo.getYawScalar());
            telemetry.update();
        } else {
            odo.recalibrateIMU();
        }
    }

    public void runOdometry() {
        odo.update();

        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        panelsTelemetry.addData("Position", data);

        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);
        panelsTelemetry.addData("Velocity", velocity);


        double newTime = timer.seconds();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;

        telemetry.addData("Status", odo.getDeviceStatus());

        telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        telemetry.update();

        panelsTelemetry.addData("Status", odo.getDeviceStatus());

        panelsTelemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

        panelsTelemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        panelsTelemetry.update();
    }

}
