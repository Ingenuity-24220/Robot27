package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
 * Manual AUTO runner using Pinpoint + field-to-robot PD + kS (no Pedro follower).
 *
 * Coordinates are Pedro's: field 0-144 in on both axes, origin at the bottom-left corner,
 * +X right, +Y up, heading 0 deg = facing +X, CCW positive. Pinpoint uses the same frame,
 * so poses from the Pedro visualizer can be used directly.
 *
 * Edit runPath() to build the routine. Motor powers go straight to the motors
 * (Drivetrain's 0.05 smoothing filter is bypassed).
 */
@Autonomous(name = "Custom Auto Runner")
public class CustomAutoRunner extends LinearOpMode {

    private static final double SETTLE_SEC = 0.15;
    private static final double DEFAULT_SWITCH_RADIUS_IN = 4.0;

    private DcMotor[] motors;
    private Localization odo;
    private TelemetryManager panels;
    private final ElapsedTime clock = new ElapsedTime();
    private DataLogger log;
    private int step = 0; // increments on every driveTo/driveThrough call

    // Start pose: looking from the red side, 2nd tile from the left and 4th tile from the top,
    // beside the hive's left red pod, facing the red wall (2 tiles away).
    private final double startX = 36.0, startY = 60.0, startH = -90.0;

    // Max power for the whole routine. Change it during INIT with D-pad Up/Down (0.2 to 1.0).
    private double speed = 0.6;

    // Routine to run. Change it during INIT with D-pad Left/Right.
    private static final String[] ROUTINES = {
            "Example: line, curve, turn, back to start",
            "Lap around center, strafing (heading 180)",
            "Lap around center, facing travel direction",
            "S-curve up the left side, turn while driving back",
            "Example x3 (repeatability)",
    };
    private int routine = 0;

    @Override
    public void runOpMode() {
        motors = AutoConfig.initDriveMotors(hardwareMap);
        odo = new Localization(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        log = new DataLogger("auto",
                "t", "step", "passThrough", "targetX", "targetY", "targetH",
                "x", "y", "heading", "velX", "velY", "velH",
                "fwdErr", "leftErr", "headErrDeg", "dist", "dFwd", "dLeft", "dHead",
                "fwdOut", "leftOut", "turnOut", "maxPower",
                "kP_xy", "kD_xy", "kS_xy", "kP_h", "kD_h", "kS_h", "D_FILTER",
                "hubHeading", "pinpointHeading", "useHubHeading");
        try {
            runAuto();
        } finally {
            AutoConfig.stop(motors);
            log.close();
        }
    }

    private void runAuto() {
        // Let the Pinpoint finish its IMU calibration (~0.25 s) while the robot is still.
        sleep(300);
        setStartPose(startX, startY, startH);

        while (opModeInInit()) {
            if (gamepad1.dpadUpWasPressed()) speed = Math.min(1.0, Math.round(speed * 10 + 1) / 10.0);
            if (gamepad1.dpadDownWasPressed()) speed = Math.max(0.2, Math.round(speed * 10 - 1) / 10.0);
            if (gamepad1.dpadRightWasPressed()) routine = (routine + 1) % ROUTINES.length;
            if (gamepad1.dpadLeftWasPressed()) routine = (routine - 1 + ROUTINES.length) % ROUTINES.length;
            odo.update();
            panels.addData("Speed (D-pad Up/Down)", String.format(Locale.US, "%.1f", speed));
            panels.addData("Routine (D-pad Left/Right)", (routine + 1) + ". " + ROUTINES[routine]);
            showPose("INIT - keep robot still");
        }
        if (isStopRequested()) return;

        // Re-apply in case the robot was nudged during init.
        setStartPose(startX, startY, startH);

        runPath();
    }

    /**
     * The routine picked during INIT. All start at (startX, startY, startH) and end back there,
     * so the end position can be checked against tape on the floor.
     *
     * Field keep-out used for the test routines (this season's field, Pedro coordinates):
     * center structure x 52-92, y 46-98; robot center stays 12 in or more from it and the walls.
     */
    private void runPath() {
        switch (routine) {
            case 0: example(); break;
            case 1: lap(false); break;
            case 2: lap(true); break;
            case 3: sCurve(); break;
            case 4: for (int i = 0; i < 3 && opModeIsActive(); i++) example(); break;
        }
    }

    /** meowAuto's line + curve to park, a turn, then back to the start. */
    private void example() {
        driveTo(23.026, 34.411, 180, speed);
        followBezier(23.026, 34.411, 44.315, 47.793, 23.026, 65.816, 180, 180, speed);
        waitSec(0.25);
        turnTo(90, speed);
        turnTo(180, speed);
        driveTo(23.026, 34.411, 180, speed);
        driveTo(startX, startY, startH, speed);
    }

    /**
     * Clockwise lap around the center structure through corners (28, 28), (28, 116), (116, 116),
     * (116, 28) without stopping. Tests long full-speed legs and carrying speed into corners.
     * faceTravel = false keeps heading 180, so the legs are driven forward, strafing, and backward.
     * faceTravel = true turns to face each leg (90, 0, -90, 180), turning while moving at corners.
     */
    private void lap(boolean faceTravel) {
        driveThrough(28, 28, 180, speed);
        driveThrough(28, 116, faceTravel ? 90 : 180, speed);
        driveThrough(116, 116, faceTravel ? 0 : 180, speed);
        driveThrough(116, 28, faceTravel ? -90 : 180, speed);
        driveThrough(36, 28, 180, speed); // go around the red pod, not through it
        driveTo(startX, startY, startH, speed);
    }

    /**
     * Two chained curves up the left side while turning 180 -> 135 -> 90, then straight back down
     * while turning 90 -> 180 at the same time, and back to the start.
     */
    private void sCurve() {
        driveTo(28, 28, 180, speed);
        followBezier(28, 28, 18, 52, 24, 72, 180, 135, speed);
        followBezier(24, 72, 40, 96, 24, 116, 135, 90, speed);
        driveTo(28, 28, 180, speed);
        driveTo(startX, startY, startH, speed);
    }

    // ------------------------------------------------------------------------------------
    // Movement API
    // ------------------------------------------------------------------------------------

    public void setStartPose(double x, double y, double headingDeg) {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDeg));
        odo.update();
    }

    /** Drive to a pose and settle there, with default tolerances and a timeout that scales with distance. */
    public boolean driveTo(double x, double y, double headingDeg, double maxPower) {
        return driveTo(x, y, headingDeg, maxPower, AutoConfig.POS_TOL_IN, AutoConfig.HEAD_TOL_DEG,
                timeoutFor(x, y, maxPower));
    }

    /** At least 3 s, plus time for long moves at low power (assumes 20 in/s per unit of power). */
    private double timeoutFor(double x, double y, double maxPower) {
        Pose2D pose = odo.getPosition();
        double dist = Math.hypot(x - pose.getX(DistanceUnit.INCH), y - pose.getY(DistanceUnit.INCH));
        return Math.max(3.0, 1.0 + dist / (20.0 * Math.max(maxPower, 0.1)));
    }

    /**
     * Drive to a pose and settle: done once position error is under tolIn and heading
     * error under tolDeg continuously for SETTLE_SEC. Stops the motors afterward.
     * Returns false on timeout or stop.
     */
    public boolean driveTo(double x, double y, double headingDeg, double maxPower,
                           double tolIn, double tolDeg, double timeoutSec) {
        boolean ok = move(x, y, headingDeg, maxPower, tolIn, tolDeg, timeoutSec, -1);
        AutoConfig.stop(motors);
        return ok;
    }

    /**
     * Pass through an intermediate waypoint without stopping: drives straight at (x, y) at
     * maxPower and ends as soon as the robot is within switchRadiusIn of it. Heading is not
     * required to be reached. Motors keep running into the next step.
     */
    public boolean driveThrough(double x, double y, double headingDeg, double maxPower, double switchRadiusIn) {
        return move(x, y, headingDeg, maxPower, 0, 0, timeoutFor(x, y, maxPower), switchRadiusIn);
    }

    public boolean driveThrough(double x, double y, double headingDeg, double maxPower) {
        return driveThrough(x, y, headingDeg, maxPower, DEFAULT_SWITCH_RADIUS_IN);
    }

    /** Turn in place to a heading, holding the current position. */
    public boolean turnTo(double headingDeg, double maxPower) {
        Pose2D pose = odo.getPosition();
        return driveTo(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), headingDeg, maxPower,
                AutoConfig.POS_TOL_IN, AutoConfig.HEAD_TOL_DEG, 2.0);
    }

    /** Wait with motors stopped, keeping odometry updated. */
    public void waitSec(double seconds) {
        AutoConfig.stop(motors);
        double end = clock.seconds() + seconds;
        while (opModeIsActive() && clock.seconds() < end) {
            odo.update();
            showPose("WAIT");
        }
    }

    /**
     * Follow a quadratic Bezier (p0 -> control p1 -> p2) from the Pedro visualizer by sampling
     * it at 10 points. Heading changes linearly (shortest way) from startH to endH.
     * Points 1-9 use driveThrough; the last point uses driveTo and settles.
     */
    public boolean followBezier(double x0, double y0, double x1, double y1, double x2, double y2,
                                double startHDeg, double endHDeg, double maxPower) {
        final int samples = 10;
        double dH = AngleUnit.normalizeDegrees(endHDeg - startHDeg);
        for (int i = 1; i <= samples; i++) {
            double t = (double) i / samples;
            double u = 1 - t;
            double x = u * u * x0 + 2 * u * t * x1 + t * t * x2;
            double y = u * u * y0 + 2 * u * t * y1 + t * t * y2;
            double h = startHDeg + dH * t;
            boolean ok = (i < samples)
                    ? driveThrough(x, y, h, maxPower)
                    : driveTo(x, y, h, maxPower, AutoConfig.POS_TOL_IN, AutoConfig.HEAD_TOL_DEG, 3.0);
            if (!ok) {
                AutoConfig.stop(motors);
                return false;
            }
        }
        return true;
    }

    // ------------------------------------------------------------------------------------
    // Controller
    // ------------------------------------------------------------------------------------

    /**
     * Shared PD + kS loop. If switchRadiusIn > 0, exits as soon as within that radius
     * (pass-through). Otherwise exits after settling within tolIn/tolDeg for SETTLE_SEC.
     */
    private boolean move(double tx, double ty, double tHeadingDeg, double maxPower,
                         double tolIn, double tolDeg, double timeoutSec, double switchRadiusIn) {
        double tH = Math.toRadians(tHeadingDeg);
        double start = clock.seconds();
        double settledSince = -1;

        // Seed derivative state with the current error so the first cycle has no D kick.
        odo.update();
        double[] err = errors(tx, ty, tH);
        double prevFwd = err[0], prevLeft = err[1], prevHead = err[2];
        double prevTime = clock.seconds();
        double dFwd = 0, dLeft = 0, dHead = 0; // filtered error derivatives
        step++;

        while (opModeIsActive()) {
            odo.update();
            // Read the clock after update() so dt spans the same interval as the pose change.
            // (Reading it before made the first cycle of every step divide ~20 ms of motion by ~0.1 ms.)
            double now = clock.seconds();
            if (now - start > timeoutSec) return false;

            err = errors(tx, ty, tH);
            double fwdErr = err[0], leftErr = err[1], headErr = err[2], dist = err[3];

            if (switchRadiusIn > 0) {
                if (dist < switchRadiusIn) return true;
            } else {
                boolean inTol = dist < tolIn && Math.abs(Math.toDegrees(headErr)) < tolDeg;
                if (inTol) {
                    if (settledSince < 0) settledSince = now;
                    if (now - settledSince >= SETTLE_SEC) return true;
                } else {
                    settledSince = -1;
                }
            }

            double dt = Math.max(now - prevTime, 1e-4);
            // Low-pass filter the derivative so sensor jitter over a few-ms loop doesn't become power spikes.
            dFwd += AutoConfig.D_FILTER * ((fwdErr - prevFwd) / dt - dFwd);
            dLeft += AutoConfig.D_FILTER * ((leftErr - prevLeft) / dt - dLeft);
            dHead += AutoConfig.D_FILTER * ((headErr - prevHead) / dt - dHead);
            double fwd = AutoConfig.kP_xy * fwdErr + AutoConfig.kD_xy * dFwd;
            double left = AutoConfig.kP_xy * leftErr + AutoConfig.kD_xy * dLeft;
            double turn = AutoConfig.kP_h * headErr + AutoConfig.kD_h * dHead;
            prevFwd = fwdErr;
            prevLeft = leftErr;
            prevHead = headErr;
            prevTime = now;

            // Static friction feedforward along the direction to the target (so a tiny sideways
            // error gets a tiny share), faded in smoothly around the tolerance instead of switched.
            if (dist > 1e-6) {
                double ks = AutoConfig.kS_xy * AutoConfig.frictionRamp(dist, AutoConfig.POS_TOL_IN);
                fwd += ks * fwdErr / dist;
                left += ks * leftErr / dist;
            }
            turn += Math.signum(headErr) * AutoConfig.kS_h
                    * AutoConfig.frictionRamp(Math.abs(Math.toDegrees(headErr)), AutoConfig.HEAD_TOL_DEG);

            if (switchRadiusIn > 0 && dist > 1e-6) {
                // Pass-through: drive straight at the waypoint at full maxPower instead of slowing
                // toward it (which made curves surge and stall at every point). Heading keeps its PD;
                // the final driveTo does the braking.
                fwd = maxPower * fwdErr / dist;
                left = maxPower * leftErr / dist;
            }

            AutoConfig.driveRaw(motors, fwd, left, turn, maxPower);

            Pose2D pose = odo.getPosition();
            log.log(now, step, switchRadiusIn > 0 ? 1 : 0, tx, ty, tHeadingDeg,
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES),
                    odo.velX(), odo.velY(), odo.velH(),
                    fwdErr, leftErr, Math.toDegrees(headErr), dist, dFwd, dLeft, Math.toDegrees(dHead),
                    fwd, left, turn, maxPower,
                    AutoConfig.kP_xy, AutoConfig.kD_xy, AutoConfig.kS_xy,
                    AutoConfig.kP_h, AutoConfig.kD_h, AutoConfig.kS_h, AutoConfig.D_FILTER,
                    odo.hubHeadingDeg(), odo.pinpointHeadingDeg(), odo.usingHubHeading() ? 1 : 0);

            panels.addData("target", String.format(Locale.US, "(%.1f, %.1f, %.1f)", tx, ty, tHeadingDeg));
            panels.addData("posError (in)", String.format(Locale.US, "%.2f", dist));
            panels.addData("headError (deg)", String.format(Locale.US, "%.2f", Math.toDegrees(headErr)));
            showPose(switchRadiusIn > 0 ? "DRIVE THROUGH" : "DRIVE TO");
        }
        return false;
    }

    /** Returns {fwdErr, leftErr, headErrRad, distIn}: field error rotated into the robot frame. */
    private double[] errors(double tx, double ty, double tH) {
        Pose2D pose = odo.getPosition();
        double h = pose.getHeading(AngleUnit.RADIANS);
        double dx = tx - pose.getX(DistanceUnit.INCH);
        double dy = ty - pose.getY(DistanceUnit.INCH);
        double fwdErr = dx * Math.cos(h) + dy * Math.sin(h);
        double leftErr = -dx * Math.sin(h) + dy * Math.cos(h);
        double headErr = AngleUnit.normalizeRadians(tH - h);
        return new double[]{fwdErr, leftErr, headErr, Math.hypot(dx, dy)};
    }

    private void showPose(String state) {
        Pose2D pose = odo.getPosition();
        panels.addData("State", state);
        panels.addData("Heading source", odo.usingHubHeading() ? "HUB IMU" : "PINPOINT IMU");
        panels.addData("Log file", log.path());
        panels.addData("x", pose.getX(DistanceUnit.INCH));
        panels.addData("y", pose.getY(DistanceUnit.INCH));
        panels.addData("heading", pose.getHeading(AngleUnit.DEGREES));
        panels.update(telemetry);
    }
}
