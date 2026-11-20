package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.AutoConfig;

import java.util.Locale;

/**
 * Tuner 2: live PD + kS tuning on the field-to-robot controller used by CustomAutoRunner.
 *
 * LB = RUNNING/PAUSED (starts PAUSED, use as kill switch).
 * A = toggle 24" forward, X = toggle 24" strafe left, B = toggle 90 deg CCW turn,
 * Y = toggle diagonal (24", 24") + 45 deg.
 * Dpad Up/Down = select gain, Dpad Left/Right = -/+ step, RB = cycle step size.
 * Gains changed here are NOT saved: copy final values into AutoConfig.
 */
@TeleOp(name = "2. Drive PID Tuner", group = "Custom Tuning")
public class DrivePIDTuner extends OpMode {

    private DcMotor[] motors;
    private GoBildaPinpointDriver odo;
    private TelemetryManager panels;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double prevTime = 0;
    private double prevFwdErr = 0, prevLeftErr = 0, prevHeadErr = 0;

    // Start pose in Pedro coordinates (center of one tile)
    private final double startX = 24.0, startY = 24.0, startH = 0.0;
    private double targetX = startX, targetY = startY, targetH = startH;

    private boolean active = false;
    private final double maxPower = 0.65;

    private double kP_xy = AutoConfig.kP_xy;
    private double kD_xy = AutoConfig.kD_xy;
    private double kS_xy = AutoConfig.kS_xy;
    private double kP_h = AutoConfig.kP_h;
    private double kD_h = AutoConfig.kD_h;
    private double kS_h = AutoConfig.kS_h;

    private final String[] paramNames = {"kP_xy", "kD_xy", "kS_xy", "kP_h", "kD_h", "kS_h"};
    private int paramIndex = 0;
    private final double[] stepSizes = {0.1, 0.01, 0.001};
    private int stepIndex = 1;

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
        odo.setPosition(new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.DEGREES, startH));
        loopTimer.reset();
        prevTime = 0;
    }

    @Override
    public void loop() {
        // --- Target selection ---
        if (gamepad1.aWasPressed()) { // Forward 24 in
            targetX = (Math.abs(targetX - startX) < 1.0) ? startX + 24.0 : startX;
            targetY = startY;
            targetH = startH;
        }
        if (gamepad1.xWasPressed()) { // Strafe left 24 in
            targetX = startX;
            targetY = (Math.abs(targetY - startY) < 1.0) ? startY + 24.0 : startY;
            targetH = startH;
        }
        if (gamepad1.bWasPressed()) { // Turn 90 deg CCW
            targetX = startX;
            targetY = startY;
            targetH = (Math.abs(targetH - startH) < 1.0) ? startH + 90.0 : startH;
        }
        if (gamepad1.yWasPressed()) { // Diagonal + turn
            boolean atStart = Math.abs(targetX - startX) < 1.0 && Math.abs(targetY - startY) < 1.0;
            targetX = atStart ? startX + 24.0 : startX;
            targetY = atStart ? startY + 24.0 : startY;
            targetH = atStart ? startH + 45.0 : startH;
        }

        // --- Gain editing ---
        if (gamepad1.leftBumperWasPressed()) active = !active;
        if (gamepad1.rightBumperWasPressed()) stepIndex = (stepIndex + 1) % stepSizes.length;
        if (gamepad1.dpadUpWasPressed()) paramIndex = (paramIndex - 1 + paramNames.length) % paramNames.length;
        if (gamepad1.dpadDownWasPressed()) paramIndex = (paramIndex + 1) % paramNames.length;

        double delta = 0;
        if (gamepad1.dpadRightWasPressed()) delta = stepSizes[stepIndex];
        if (gamepad1.dpadLeftWasPressed()) delta = -stepSizes[stepIndex];
        if (delta != 0) adjustSelectedParam(delta);

        // --- Pinpoint update ---
        odo.update();
        Pose2D pose = odo.getPosition();
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double headingRad = pose.getHeading(AngleUnit.RADIANS);
        double headingDeg = pose.getHeading(AngleUnit.DEGREES);

        double dx = targetX - x;
        double dy = targetY - y;
        double headErr = AngleUnit.normalizeRadians(Math.toRadians(targetH) - headingRad);

        // Rotate field error into robot frame (+forward, +left)
        double fwdErr = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
        double leftErr = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);

        double now = loopTimer.seconds();
        double dt = Math.max(now - prevTime, 1e-4);
        double dFwd = (fwdErr - prevFwdErr) / dt;
        double dLeft = (leftErr - prevLeftErr) / dt;
        double dHead = (headErr - prevHeadErr) / dt;
        prevFwdErr = fwdErr;
        prevLeftErr = leftErr;
        prevHeadErr = headErr;
        prevTime = now;

        if (active) {
            double fwd = kP_xy * fwdErr + kD_xy * dFwd;
            double left = kP_xy * leftErr + kD_xy * dLeft;
            double turn = kP_h * headErr + kD_h * dHead;

            // Static friction feedforward along the direction to the target (same as CustomAutoRunner).
            double dist = Math.hypot(dx, dy);
            if (dist > AutoConfig.POS_TOL_IN) {
                fwd += kS_xy * fwdErr / dist;
                left += kS_xy * leftErr / dist;
            }
            if (Math.abs(Math.toDegrees(headErr)) > AutoConfig.HEAD_TOL_DEG && Math.abs(turn) > 0.005) {
                turn += Math.copySign(kS_h, turn);
            }

            AutoConfig.driveRaw(motors, fwd, left, turn, maxPower);
        } else {
            AutoConfig.stop(motors);
        }

        // --- Telemetry & Panels graphs ---
        panels.addData("State (LB to toggle)", active ? "RUNNING" : "PAUSED");
        panels.addData("Selected Gain (D-Up/Dn)", paramNames[paramIndex] + " | step=" + stepSizes[stepIndex]);
        panels.addData("targetX", targetX);
        panels.addData("x", x);
        panels.addData("targetY", targetY);
        panels.addData("y", y);
        panels.addData("targetH", targetH);
        panels.addData("heading", headingDeg);
        panels.addData("posError (in)", String.format(Locale.US, "%.2f", Math.hypot(dx, dy)));
        panels.addData("headError (deg)", String.format(Locale.US, "%.2f", Math.toDegrees(headErr)));
        panels.addData("kP_xy / kD_xy / kS_xy", String.format(Locale.US, "%.4f / %.4f / %.3f", kP_xy, kD_xy, kS_xy));
        panels.addData("kP_h  / kD_h  / kS_h", String.format(Locale.US, "%.4f / %.4f / %.3f", kP_h, kD_h, kS_h));
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        AutoConfig.stop(motors);
    }

    private void adjustSelectedParam(double delta) {
        switch (paramIndex) {
            case 0: kP_xy = Math.max(0, kP_xy + delta); break;
            case 1: kD_xy = Math.max(0, kD_xy + delta); break;
            case 2: kS_xy = Math.max(0, kS_xy + delta); break;
            case 3: kP_h = Math.max(0, kP_h + delta); break;
            case 4: kD_h = Math.max(0, kD_h + delta); break;
            case 5: kS_h = Math.max(0, kS_h + delta); break;
        }
    }
}
