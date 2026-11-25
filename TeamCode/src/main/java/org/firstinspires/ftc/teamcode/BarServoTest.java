package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 * Bar Servo Test: finds which way servoBarL and servoBarR must move to push their shared bar
 * forward. Both servos hold the same bar, so they always move together, by the same small step:
 * by default in opposite directions (L up, R down), since they are mounted mirrored. Moving one
 * alone would twist the bar against the other.
 *
 * Nothing moves until the first D-pad Up/Down after START. That first nudge sends each servo to
 * its HOME +/- one step, so set HOME_L and HOME_R in INIT to where the bar rests now.
 * Every move is clamped to HOME +/- MAX_OFFSET.
 *
 *   INIT  D-pad Up/Down     HOME_L +/- 0.01   (nothing moves)
 *   INIT  RB/LB             HOME_R +/- 0.01
 *   D-pad Up/Down           nudge the bar one step (+ / -)
 *   D-pad Right/Left        bigger / smaller step
 *   Y                       "the bar went FORWARD on that nudge"
 *   A                       "the bar went BACKWARD on that nudge"
 *   B                       back to HOME
 *   X (at HOME only)        switch pairing: opposite (L+, R-) <-> same (L+, R+). Use it only if
 *                           opposite makes the bar bind or twist instead of moving.
 *
 * The RESULT line on the Driver Station says which direction is forward for each servo.
 * Continuous rotation servos get short low-power pulses instead of positions.
 */
@TeleOp(name = "Bar Servo Test")
public class BarServoTest extends OpMode {

    static double HOME_L = 0.5, HOME_R = 0.5;       // positions the test starts from
    static final double MAX_OFFSET = 0.10;          // never further than this from HOME
    static final double[] STEPS = {0.005, 0.01, 0.02};
    static final double CR_POWER = 0.15;            // continuous rotation servos only
    static final double CR_PULSE_SEC = 0.15;

    ServoImplEx servoL, servoR;
    CRServo crL, crR;

    TelemetryManager panels;
    final ElapsedTime clock = new ElapsedTime();
    int stepIdx = 0;
    int pairing = -1;          // R moves pairing * L's direction: -1 opposite, +1 same
    double offset = 0;         // L's offset from HOME_L; R's is pairing * offset
    int lastDir = 0;           // sign of the last nudge (in L's direction)
    int forward = 0;           // L direction that moves the bar forward (0 = unknown)
    int forwardPairing = 0;    // pairing that was active when forward was recorded
    double pulseEnd = -1;
    String message = "Nudge with D-pad Up/Down, then press Y (forward) or A (backward)";

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        servoL = hardwareMap.tryGet(ServoImplEx.class, "servoBarL");
        servoR = hardwareMap.tryGet(ServoImplEx.class, "servoBarR");
        if (servoL == null) crL = hardwareMap.tryGet(CRServo.class, "servoBarL");
        if (servoR == null) crR = hardwareMap.tryGet(CRServo.class, "servoBarR");
        release();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadUpWasPressed()) HOME_L = clampHome(HOME_L + 0.01);
        if (gamepad1.dpadDownWasPressed()) HOME_L = clampHome(HOME_L - 0.01);
        if (gamepad1.rightBumperWasPressed()) HOME_R = clampHome(HOME_R + 0.01);
        if (gamepad1.leftBumperWasPressed()) HOME_R = clampHome(HOME_R - 0.01);
        panels.addData("HOME_L (D-pad Up/Down)", fmt(HOME_L));
        panels.addData("HOME_R (RB/LB)", fmt(HOME_R));
        panels.addData("Note", "set both to where the bar rests now; nothing moves until START + D-pad");
        panels.addData("servoBarL", kind(servoL, crL));
        panels.addData("servoBarR", kind(servoR, crR));
        panels.update(telemetry);
    }

    @Override
    public void start() {
        clock.reset();
    }

    @Override
    public void loop() {
        double now = clock.seconds();
        double step = STEPS[stepIdx];

        // *WasPressed() is true only on its first call per press, so read each button once.
        boolean up = gamepad1.dpadUpWasPressed(), down = gamepad1.dpadDownWasPressed();
        boolean y = gamepad1.yWasPressed(), a = gamepad1.aWasPressed();
        boolean b = gamepad1.bWasPressed(), x = gamepad1.xWasPressed();

        if (gamepad1.dpadRightWasPressed()) stepIdx = Math.min(STEPS.length - 1, stepIdx + 1);
        if (gamepad1.dpadLeftWasPressed()) stepIdx = Math.max(0, stepIdx - 1);

        int dir = up ? 1 : down ? -1 : 0;
        if (dir != 0) {
            double o = offset + dir * step;
            if (Math.abs(o) > MAX_OFFSET + 1e-9) {
                message = "Limit reached (HOME +/- " + fmt(MAX_OFFSET) + ")";
            } else {
                moveBoth(o, dir, now);
                lastDir = dir;
                message = "Nudged L " + (dir > 0 ? "+" : "-") + ", R " + (dir * pairing > 0 ? "+" : "-")
                        + fmt(step) + ". Which way did the bar go? Y = forward, A = backward";
            }
        }

        if (y || a) {
            if (lastDir == 0) {
                message = "Nudge the bar first";
            } else {
                forward = y ? lastDir : -lastDir;
                forwardPairing = pairing;
                message = "Recorded. Press B to send the bar home.";
            }
        }

        if (b) {
            if (servoL != null || servoR != null) {
                moveBoth(0, 0, now);
                message = "Back to HOME";
            } else {
                message = "Continuous servos have no home; nudge them back instead";
            }
        }

        if (x) {
            if (Math.abs(offset) > 1e-9) {
                message = "Press B (back to HOME) before switching pairing";
            } else {
                pairing = -pairing;
                lastDir = 0;
                message = "Pairing: " + pairingText(pairing);
            }
        }

        // End continuous rotation pulses.
        if (pulseEnd >= 0 && now >= pulseEnd) {
            if (crL != null) crL.setPower(0);
            if (crR != null) crR.setPower(0);
            pulseEnd = -1;
        }

        // --- Telemetry ---
        panels.addData("Pairing (X at HOME)", pairingText(pairing));
        panels.addData("Step (D-pad L/R)", fmt(step));
        panels.addData("Message", message);
        panels.addData("servoBarL", kind(servoL, crL) + (servoL != null ? ", position " + fmt(HOME_L + offset) : ""));
        panels.addData("servoBarR", kind(servoR, crR) + (servoR != null ? ", position " + fmt(HOME_R + pairing * offset) : ""));
        panels.addData("RESULT", result());
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        release();
    }

    /** Moves both servos together: L to HOME_L + newOffset, R to HOME_R + pairing * newOffset. */
    private void moveBoth(double newOffset, int pulseDir, double now) {
        if (servoL != null) servoL.setPosition(HOME_L + newOffset);
        if (servoR != null) servoR.setPosition(HOME_R + pairing * newOffset);
        if (pulseDir != 0 && (crL != null || crR != null)) {
            if (crL != null) crL.setPower(pulseDir * CR_POWER);
            if (crR != null) crR.setPower(pairing * pulseDir * CR_POWER);
            pulseEnd = now + CR_PULSE_SEC;
        }
        offset = newOffset;
    }

    /** Unpowers both servos. */
    private void release() {
        if (servoL != null) servoL.setPwmDisable();
        if (servoR != null) servoR.setPwmDisable();
        if (crL != null) crL.setPower(0);
        if (crR != null) crR.setPower(0);
        pulseEnd = -1;
    }

    private String result() {
        if (forward == 0) return "not done: nudge, then press Y or A";
        return String.format(Locale.US, "forward = servoBarL %s, servoBarR %s  (HOME_L %s, HOME_R %s)",
                sign(forward), sign(forward * forwardPairing), fmt(HOME_L), fmt(HOME_R));
    }

    private static String pairingText(int p) {
        return p < 0 ? "opposite (L+ with R-)" : "same (L+ with R+)";
    }

    private static double clampHome(double v) {
        return Math.max(MAX_OFFSET, Math.min(1.0 - MAX_OFFSET, v));
    }

    private static String kind(ServoImplEx s, CRServo c) {
        return s != null ? "servo" : c != null ? "continuous servo" : "NOT FOUND in config";
    }

    private static String sign(int s) {
        return s > 0 ? "increase position (+)" : "decrease position (-)";
    }

    private static String fmt(double v) {
        return String.format(Locale.US, "%.3f", v);
    }
}
