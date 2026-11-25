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
 * Bar Servo Test: finds which way servoBarL and servoBarR must move to push the bar forward.
 * The two servos are mirrored, so forward is one servo's position going up and the other's
 * going down. Test each servo alone first (the other is unpowered), so they never fight.
 *
 * Nothing moves until you press D-pad Up/Down after START. The first nudge sends the servo to
 * HOME +/- one step, so set HOME (D-pad Up/Down during INIT) to where the bar rests now.
 * Every move is clamped to HOME +/- MAX_OFFSET.
 *
 *   INIT  D-pad Up/Down     HOME +/- 0.01 (nothing moves)
 *   X                       select servoBarL -> servoBarR -> both (both only once each is known)
 *   D-pad Up/Down           nudge the selected servo + / -  (both: forward / backward)
 *   D-pad Right/Left        bigger / smaller step
 *   Y                       "the bar went FORWARD on that nudge"  (records the servo's direction)
 *   A                       "the bar went BACKWARD on that nudge"
 *   B                       send the selected servo(s) back to HOME
 *
 * The result line on the Driver Station says which direction is forward for each servo.
 * Works with standard servos; continuous rotation servos get short low-power pulses instead.
 */
@TeleOp(name = "Bar Servo Test")
public class BarServoTest extends OpMode {

    static double HOME = 0.5;                       // position the test starts from
    static final double MAX_OFFSET = 0.10;          // never further than this from HOME
    static final double[] STEPS = {0.005, 0.01, 0.02};
    static final double CR_POWER = 0.15;            // continuous rotation servos only
    static final double CR_PULSE_SEC = 0.15;

    static final String[] NAMES = {"servoBarL", "servoBarR"};

    final ServoImplEx[] servo = new ServoImplEx[2];
    final CRServo[] cr = new CRServo[2];
    final double[] offset = {0, 0};   // commanded position minus HOME (CR: step-equivalents pulsed)
    final int[] lastDir = {0, 0};     // sign of the last nudge
    final int[] forward = {0, 0};     // +1: increasing position = forward, -1: decreasing, 0: unknown
    final double[] pulseEnd = {-1, -1};

    TelemetryManager panels;
    final ElapsedTime clock = new ElapsedTime();
    int selected = 0;                 // 0 = L, 1 = R, 2 = both
    int stepIdx = 0;
    String message = "Select a servo with X, then nudge with D-pad Up/Down";

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        for (int i = 0; i < 2; i++) {
            servo[i] = hardwareMap.tryGet(ServoImplEx.class, NAMES[i]);
            if (servo[i] == null) cr[i] = hardwareMap.tryGet(CRServo.class, NAMES[i]);
            release(i);
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadUpWasPressed()) HOME = Math.min(1.0 - MAX_OFFSET, HOME + 0.01);
        if (gamepad1.dpadDownWasPressed()) HOME = Math.max(MAX_OFFSET, HOME - 0.01);
        panels.addData("HOME (D-pad Up/Down)", fmt(HOME) + "  set this to where the bar rests now");
        panels.addData("Range", fmt(HOME - MAX_OFFSET) + " to " + fmt(HOME + MAX_OFFSET));
        for (int i = 0; i < 2; i++) panels.addData(NAMES[i], kind(i));
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

        if (gamepad1.xWasPressed()) {
            int next = (selected + 1) % 3;
            if (next == 2 && (forward[0] == 0 || forward[1] == 0)) next = 0;
            selected = next;
            // Only the selected servo is powered, so a single-servo test never fights the other one.
            if (selected != 2) release(1 - selected);
            message = "Selected " + selectedName();
        }
        if (gamepad1.dpadRightWasPressed()) stepIdx = Math.min(STEPS.length - 1, stepIdx + 1);
        if (gamepad1.dpadLeftWasPressed()) stepIdx = Math.max(0, stepIdx - 1);

        int dir = gamepad1.dpadUpWasPressed() ? 1 : gamepad1.dpadDownWasPressed() ? -1 : 0;
        if (dir != 0) {
            if (selected == 2) {
                // dir = +1 means forward: each servo moves its own forward way.
                double l = offset[0] + dir * forward[0] * step, r = offset[1] + dir * forward[1] * step;
                if (Math.abs(l) > MAX_OFFSET + 1e-9 || Math.abs(r) > MAX_OFFSET + 1e-9) {
                    message = "Limit reached (HOME +/- " + fmt(MAX_OFFSET) + ")";
                } else {
                    move(0, l, forward[0] * dir, now);
                    move(1, r, forward[1] * dir, now);
                    message = dir > 0 ? "Both: forward one step" : "Both: backward one step";
                }
            } else {
                double o = offset[selected] + dir * step;
                if (Math.abs(o) > MAX_OFFSET + 1e-9) {
                    message = "Limit reached (HOME +/- " + fmt(MAX_OFFSET) + ")";
                } else {
                    move(selected, o, dir, now);
                    lastDir[selected] = dir;
                    message = NAMES[selected] + (dir > 0 ? " +" : " -") + fmt(step)
                            + ". Which way did the bar go? Y = forward, A = backward";
                }
            }
        }

        // *WasPressed() is true only on its first call per press, so read each button once.
        boolean y = gamepad1.yWasPressed(), a = gamepad1.aWasPressed();
        if (selected != 2 && (y || a)) {
            boolean wentForward = y;
            if (lastDir[selected] == 0) {
                message = "Nudge " + NAMES[selected] + " first";
            } else {
                forward[selected] = wentForward ? lastDir[selected] : -lastDir[selected];
                message = "Recorded: " + NAMES[selected] + " forward = " + sign(forward[selected])
                        + ". Press B to send it home.";
            }
        }

        if (gamepad1.bWasPressed()) {
            message = selectedName() + " back to HOME";
            for (int i = 0; i < 2; i++) {
                if (selected != i && selected != 2) continue;
                if (servo[i] != null) move(i, 0, 0, now);
                else message = "Continuous servos have no home; nudge them back instead";
            }
        }

        // End continuous rotation pulses.
        for (int i = 0; i < 2; i++) {
            if (cr[i] != null && pulseEnd[i] >= 0 && now >= pulseEnd[i]) {
                cr[i].setPower(0);
                pulseEnd[i] = -1;
            }
        }

        // --- Telemetry ---
        panels.addData("Selected (X)", selectedName());
        panels.addData("Step (D-pad L/R)", fmt(step));
        panels.addData("Message", message);
        for (int i = 0; i < 2; i++) {
            String pos = servo[i] != null ? "position " + fmt(HOME + offset[i]) : "pulses " + fmt(offset[i] / step);
            panels.addData(NAMES[i], kind(i) + ", " + pos + ", forward = "
                    + (forward[i] == 0 ? "unknown" : sign(forward[i])));
        }
        panels.addData("RESULT", result());
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        release(0);
        release(1);
    }

    /** Sends servo i to HOME + newOffset (CR: one pulse in pulseDir). */
    private void move(int i, double newOffset, int pulseDir, double now) {
        if (servo[i] != null) {
            servo[i].setPosition(HOME + newOffset);
        } else if (cr[i] != null) {
            if (pulseDir == 0) return;
            cr[i].setPower(pulseDir * CR_POWER);
            pulseEnd[i] = now + CR_PULSE_SEC;
        } else {
            return;
        }
        offset[i] = newOffset;
    }

    /** Unpowers servo i so it holds nothing and can't fight the other one. */
    private void release(int i) {
        if (servo[i] != null) servo[i].setPwmDisable();
        if (cr[i] != null) cr[i].setPower(0);
        pulseEnd[i] = -1;
    }

    private String result() {
        if (forward[0] == 0 || forward[1] == 0) return "not done: record forward for both servos (Y/A)";
        String r = String.format(Locale.US, "forward = servoBarL %s, servoBarR %s",
                sign(forward[0]), sign(forward[1]));
        if (forward[0] == forward[1]) r += "  (WARNING: same sign; mirrored servos should be opposite)";
        return r;
    }

    private String selectedName() {
        return selected == 2 ? "both" : NAMES[selected];
    }

    private String kind(int i) {
        return servo[i] != null ? "servo" : cr[i] != null ? "continuous servo" : "NOT FOUND in config";
    }

    private static String sign(int s) {
        return s > 0 ? "increase position (+)" : "decrease position (-)";
    }

    private static String fmt(double v) {
        return String.format(Locale.US, "%.3f", v);
    }
}
