package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.AutoConfig;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.Localization;

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
    private Localization odo;
    private TelemetryManager panels;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double prevTime = 0;
    private double prevFwdErr = 0, prevLeftErr = 0, prevHeadErr = 0;
    private double dFwd = 0, dLeft = 0, dHead = 0; // filtered error derivatives
    private boolean targetChanged = true;
    private int segment = 0; // increments each time A/B/X/Y changes the target
    private DataLogger log;

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
        odo = new Localization(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        log = new DataLogger("tuner",
                "t", "segment", "targetX", "targetY", "targetH",
                "x", "y", "heading", "velX", "velY", "velH",
                "fwdErr", "leftErr", "headErrDeg", "dFwd", "dLeft", "dHead",
                "fwdOut", "leftOut", "turnOut",
                "kP_xy", "kD_xy", "kS_xy", "kP_h", "kD_h", "kS_h", "D_FILTER", "maxPower",
                "hubHeading", "hubVelH", "pinpointHeading", "pinpointVelH", "headingDiff", "useHubHeading");
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
        double oldX = targetX, oldY = targetY, oldH = targetH;
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

        if (targetX != oldX || targetY != oldY || targetH != oldH) {
            targetChanged = true;
            segment++;
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

        // Both IMUs' headings in the field frame, for comparison.
        double hubHeadingDeg = odo.hubHeadingDeg();
        double headingDiff = AngleUnit.normalizeDegrees(odo.pinpointHeadingDeg() - hubHeadingDeg);

        double dx = targetX - x;
        double dy = targetY - y;
        double headErr = AngleUnit.normalizeRadians(Math.toRadians(targetH) - headingRad);

        // Rotate field error into robot frame (+forward, +left)
        double fwdErr = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
        double leftErr = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);

        double now = loopTimer.seconds();
        double dt = Math.max(now - prevTime, 1e-4);
        if (targetChanged) { // no derivative kick when the target jumps
            prevFwdErr = fwdErr;
            prevLeftErr = leftErr;
            prevHeadErr = headErr;
            targetChanged = false;
        }
        // Low-pass filtered derivative (same as CustomAutoRunner).
        dFwd += AutoConfig.D_FILTER * ((fwdErr - prevFwdErr) / dt - dFwd);
        dLeft += AutoConfig.D_FILTER * ((leftErr - prevLeftErr) / dt - dLeft);
        dHead += AutoConfig.D_FILTER * ((headErr - prevHeadErr) / dt - dHead);
        prevFwdErr = fwdErr;
        prevLeftErr = leftErr;
        prevHeadErr = headErr;
        prevTime = now;

        if (active) {
            double fwd = kP_xy * fwdErr + kD_xy * dFwd;
            double left = kP_xy * leftErr + kD_xy * dLeft;
            double turn = kP_h * headErr + kD_h * dHead;

            // Static friction feedforward, smoothly faded in (same as CustomAutoRunner).
            double dist = Math.hypot(dx, dy);
            if (dist > 1e-6) {
                double ks = kS_xy * AutoConfig.frictionRamp(dist, AutoConfig.POS_TOL_IN);
                fwd += ks * fwdErr / dist;
                left += ks * leftErr / dist;
            }
            turn += Math.signum(headErr) * kS_h
                    * AutoConfig.frictionRamp(Math.abs(Math.toDegrees(headErr)), AutoConfig.HEAD_TOL_DEG);

            AutoConfig.driveRaw(motors, fwd, left, turn, maxPower);

            // Only logged while RUNNING, so paused time doesn't fill the file.
            log.log(now, segment, targetX, targetY, targetH,
                    x, y, headingDeg,
                    odo.velX(), odo.velY(), odo.velH(),
                    fwdErr, leftErr, Math.toDegrees(headErr), dFwd, dLeft, Math.toDegrees(dHead),
                    fwd, left, turn,
                    kP_xy, kD_xy, kS_xy, kP_h, kD_h, kS_h, AutoConfig.D_FILTER, maxPower,
                    hubHeadingDeg, odo.hubVelH(), odo.pinpointHeadingDeg(), odo.pinpointVelH(), headingDiff,
                    odo.usingHubHeading() ? 1 : 0);
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
        panels.addData("Heading source", odo.usingHubHeading() ? "HUB IMU" : "PINPOINT IMU");
        panels.addData("hubHeading", hubHeadingDeg);
        panels.addData("Pinpoint - hub heading (deg)", String.format(Locale.US, "%.2f", headingDiff));
        panels.addData("Log file", log.path());
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        AutoConfig.stop(motors);
        log.close();
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
