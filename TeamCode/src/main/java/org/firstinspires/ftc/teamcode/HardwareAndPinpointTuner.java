package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.AutoConfig;

import java.util.Locale;

/**
 * Tuner 1: verifies motor directions, Pinpoint pod directions, and measures pod offsets.
 *
 * 1. Hold A/B/X/Y (LF/RF/LR/RR): each wheel must spin forward. Fix directions in AutoConfig.
 * 2. Push robot forward: X increases. Push left: Y increases. Turn CCW: heading increases.
 *    Otherwise fix X_POD_DIR / Y_POD_DIR in AutoConfig.
 * 3. Offsets: LB toggles ZERO_OFFSET mode (pose resets). Spin roughly 180 deg CCW in place,
 *    press RB, copy the computed X_POD_OFFSET_MM / Y_POD_OFFSET_MM into AutoConfig.
 */
@TeleOp(name = "1. Hardware & Pinpoint Tuner", group = "Custom Tuning")
public class HardwareAndPinpointTuner extends OpMode {

    private DcMotor[] motors; // LF, LR, RF, RR
    private GoBildaPinpointDriver odo;
    private TelemetryManager panels;

    private boolean zeroOffsetsMode = false;
    private double calcXOffsetMM = AutoConfig.X_POD_OFFSET_MM;
    private double calcYOffsetMM = AutoConfig.Y_POD_OFFSET_MM;
    private boolean hasCalculatedOffsets = false;

    @Override
    public void init() {
        motors = AutoConfig.initDriveMotors(hardwareMap);
        odo = AutoConfig.initPinpoint(hardwareMap, false);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("Keep robot still for 1s before pressing START.");
        telemetry.update();
    }

    @Override
    public void start() {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {
        // 1. Individual motor check (hold button to spin forward at 0.35 power)
        motors[0].setPower(gamepad1.a ? 0.35 : 0.0); // LF
        motors[1].setPower(gamepad1.x ? 0.35 : 0.0); // LR
        motors[2].setPower(gamepad1.b ? 0.35 : 0.0); // RF
        motors[3].setPower(gamepad1.y ? 0.35 : 0.0); // RR

        // 2. Reset pose / toggle zero-offset mode for offset calibration
        if (gamepad1.dpadDownWasPressed()) {
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }

        if (gamepad1.leftBumperWasPressed()) {
            zeroOffsetsMode = !zeroOffsetsMode;
            odo.setOffsets(
                    zeroOffsetsMode ? 0.0 : AutoConfig.X_POD_OFFSET_MM,
                    zeroOffsetsMode ? 0.0 : AutoConfig.Y_POD_OFFSET_MM,
                    DistanceUnit.MM
            );
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }

        odo.update();
        Pose2D pose = odo.getPosition();
        double xMM = pose.getX(DistanceUnit.MM);
        double yMM = pose.getY(DistanceUnit.MM);
        double theta = pose.getHeading(AngleUnit.RADIANS);

        // 3. After spinning (ideally ~180 deg) in zero-offset mode, press RB to compute offsets.
        // Zero-offset pose = (R(theta) - I) * podPoint, so podPoint = (R(theta) - I)^-1 * pose.
        if (gamepad1.rightBumperWasPressed() && zeroOffsetsMode) {
            double denom = 2.0 * (1.0 - Math.cos(theta));
            if (Math.abs(denom) > 0.1) {
                double a = Math.cos(theta) - 1.0;
                double b = Math.sin(theta);
                calcYOffsetMM = (a * xMM + b * yMM) / denom;   // forward position of strafe pod
                calcXOffsetMM = (-b * xMM + a * yMM) / denom;  // sideways position of forward pod
                hasCalculatedOffsets = true;
            }
        }

        panels.addData("Motors (Hold)", "A=LF  B=RF  X=LR  Y=RR");
        panels.addData("Pinpoint Reset", "DpadDown = zero pose | LB = toggle offset cal ("
                + (zeroOffsetsMode ? "ZERO_OFFSET" : "CONFIG_OFFSET") + ")");
        panels.addData("X (in) [+forward]", String.format(Locale.US, "%.2f", pose.getX(DistanceUnit.INCH)));
        panels.addData("Y (in) [+left]", String.format(Locale.US, "%.2f", pose.getY(DistanceUnit.INCH)));
        panels.addData("Heading (deg) [+CCW]", String.format(Locale.US, "%.2f", pose.getHeading(AngleUnit.DEGREES)));
        panels.addData("Raw Encoders (X, Y)", odo.getEncoderX() + ", " + odo.getEncoderY());
        panels.addData("Device Status", String.valueOf(odo.getDeviceStatus()));

        if (hasCalculatedOffsets) {
            panels.addData("Computed X_POD_OFFSET_MM", String.format(Locale.US, "%.2f", calcXOffsetMM));
            panels.addData("Computed Y_POD_OFFSET_MM", String.format(Locale.US, "%.2f", calcYOffsetMM));
        }
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        AutoConfig.stop(motors);
    }
}
