package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/**
 * Two flywheels ("nectar" = outtakeMotor, "pollen" = outtakeMini) and their gates.
 *
 * start(type, power) runs a flywheel at a fixed power, as before. startAtSpeed(type) instead holds
 * the tuned target speed (ticks/s) with the hub's velocity PIDF, so shots stay the same as the
 * battery drains; kF is scaled by NOMINAL_VOLTAGE / battery voltage each time it starts.
 * A *_VELOCITY of 0 means "not tuned yet": startAtSpeed then uses *_POWER.
 * Tune with the "Outtake Tuner" OpMode (doc/CustomAuto.md).
 */
public class Outtake {
    // --- Tuned values: copy them in from the Outtake Tuner ---
    public static double NOMINAL_VOLTAGE = 12.0;  // battery voltage kF refers to
    public static double READY_TOLERANCE = 40;    // ticks/s: "at speed" window

    public static double NECTAR_VELOCITY = 1768.0;     // ticks/s; 0 = use NECTAR_POWER instead
    public static double NECTAR_KP = 3.83; // 3.83
    public static double NECTAR_KF = 12.55; // 12.55
    public static double NECTAR_POWER = 0.675;     // power the tuner measures kF at (TeleOp nectar power)

    public static double POLLEN_VELOCITY = 1676.00;
    public static double POLLEN_KP = 3.54;
    public static double POLLEN_KF = 14.49;
    public static double POLLEN_POWER = 0.725;    // (TeleOp pollen power)

    // --- TeleOp speed presets (ticks/s), picked with setPreset(): close, medium, far ---
    // Starting values: close = the tuned *_VELOCITY; medium / far = what AUTO's fixed 0.8 / 0.9
    // power gives (speed ~ 32767 * power / kF at ~12.2 V). Tune each at the spot it's used from.
    public static final String[] PRESET_NAMES = {"close", "medium", "far"};
    public static double[] NECTAR_PRESETS = {1768, 2100, 2380};
    public static double[] POLLEN_PRESETS = {1676, 1840, 2070};

    // --- Volleys: fire(type) runs that flywheel's pattern of gate pulses (call update() every loop) ---
    // Each entry is how many balls one pulse lets through. Nectar 1-1-1-1; pollen 1-1-2 (team's choice).
    public static int[] NECTAR_PATTERN = {1, 1, 1, 1};
    public static int[] POLLEN_PATTERN = {1, 1, 2};
    // Pulse lengths. Real pulses run 0-31 ms longer than set (meow's loop is ~31 ms).
    // Nectar: 110-150 ms = 1 ball, a 151 ms pulse let 2 through (meow_231449); >=155 ms usually 2.
    public static double NECTAR_PULSE_SEC = 0.11;
    public static double NECTAR_PULSE2_SEC = 0.17;
    // Pollen's gate is slower and less consistent: 150-180 ms mostly 0 balls (meow_231152);
    // 237-274 ms = 1 ball 5/5 in meow_231535 but only 5 of 15 in meow_20261124 (rest 0);
    // 343-380 ms = 0, 1 or 2. Set longer so fewer pulses come up empty.
    public static double POLLEN_PULSE_SEC = 0.30;
    public static double POLLEN_PULSE2_SEC = 0.42;
    public static double MIN_GAP_SEC = 0.6;        // from one gate opening to the next, at least...
    public static double MAX_GAP_SEC = 1.0;        // ...then as soon as atSpeed(), and at most this
    public static double MAX_SPINUP_SEC = 2.5;     // first pulse waits for atSpeed() up to this long

    HardwareMap hardwareMap;

    Telemetry telemetry;
    TelemetryManager panelsTelemetry;

    DcMotorEx pollenMotor;
    DcMotorEx nectarMotor;

    Servo pollenServo;
    Servo nectarServo;

    VoltageSensor battery;

    public boolean isNectarOpen = false;
    public boolean isPollenOpen = false;

    public boolean isNectarRunning = false;
    public boolean isPollenRunning = false;

    // -1 = no preset: use NECTAR_VELOCITY / POLLEN_VELOCITY (the Outtake Tuner and AUTO).
    private int preset = -1;

    // Volley state per flywheel: pulses left in the pattern, the current pulse's length, and when
    // the gate last opened (-1 = not yet).
    private int nectarLeft = 0, pollenLeft = 0;
    private double nectarPulse, pollenPulse;
    private double nectarOpenedAt = -1, pollenOpenedAt = -1, nectarVolleyStart, pollenVolleyStart;


    public Outtake(HardwareMap hardwareMap, Telemetry telemetry/*, TelemetryManager panelsTelemetry*/){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        pollenMotor = hardwareMap.get(DcMotorEx.class, "outtakeMini");
        nectarMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        pollenServo = hardwareMap.get(Servo.class, "pollenServo");
        nectarServo = hardwareMap.get(Servo.class, "nectarServo");
        battery = hardwareMap.voltageSensor.iterator().next();

        pollenMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{pollenMotor, nectarMotor}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // let flywheels coast down
        }
        this.close("nectar");
        this.close("pollen");
    }

    private boolean isNectar(String type) {
        return Objects.equals(type, "nectar");
    }

    private DcMotorEx motor(String type) {
        return isNectar(type) ? nectarMotor : pollenMotor;
    }

    /** Runs a flywheel at a fixed power, with no speed control (TeleOp: 0.62 / 0.725). */
    public void start(String type, double power) {
        runOpenLoop(type, power);
        if (isNectar(type)) this.isNectarRunning = true;
        else this.isPollenRunning = true;
    }

    /**
     * Spins a flywheel up to its tuned target speed (*_VELOCITY) and holds it there with the
     * hub's velocity PIDF. Until that flywheel is tuned (*_VELOCITY = 0) it uses *_POWER instead.
     */
    public void startAtSpeed(String type) {
        double velocity = targetVelocity(type);
        if (velocity <= 0) {
            start(type, isNectar(type) ? NECTAR_POWER : POLLEN_POWER);
            return;
        }
        DcMotorEx m = motor(type);
        applyGains(type);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setVelocity(velocity);
        if (isNectar(type)) this.isNectarRunning = true;
        else this.isPollenRunning = true;
    }

    public void stop(String type) {
        DcMotorEx m = motor(type);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setPower(0);
        if (isNectar(type)) this.isNectarRunning = false;
        else this.isPollenRunning = false;
    }

    /** Sends the current kP and voltage-compensated kF to the hub. */
    public void applyGains(String type) {
        double kP = isNectar(type) ? NECTAR_KP : POLLEN_KP;
        double kF = isNectar(type) ? NECTAR_KF : POLLEN_KF;
        motor(type).setVelocityPIDFCoefficients(kP, 0, 0, compensatedF(kF));
    }

    public double compensatedF(double kF) {
        return kF * NOMINAL_VOLTAGE / Math.max(batteryVoltage(), 1.0);
    }

    /** Runs a flywheel at a fixed power, with no speed control (used by the tuner's measure step). */
    public void runOpenLoop(String type, double power) {
        DcMotorEx m = motor(type);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setPower(power);
    }

    public double targetVelocity(String type) {
        if (preset >= 0) return isNectar(type) ? NECTAR_PRESETS[preset] : POLLEN_PRESETS[preset];
        return isNectar(type) ? NECTAR_VELOCITY : POLLEN_VELOCITY;
    }

    /** Picks a speed preset (0 close, 1 medium, 2 far); running flywheels change speed right away. */
    public void setPreset(int index) {
        preset = Math.max(0, Math.min(PRESET_NAMES.length - 1, index));
        if (isNectarRunning) startAtSpeed("nectar");
        if (isPollenRunning) startAtSpeed("pollen");
    }

    public int preset() {
        return preset;
    }

    /** Measured flywheel speed in ticks/s (needs the motor's encoder cable plugged in). */
    public double velocity(String type) {
        return motor(type).getVelocity();
    }

    /** True when the flywheel is running and within READY_TOLERANCE of its target speed. */
    public boolean atSpeed(String type) {
        boolean running = isNectar(type) ? isNectarRunning : isPollenRunning;
        double target = targetVelocity(type);
        return running && target > 0 && Math.abs(velocity(type) - target) < READY_TOLERANCE;
    }

    /**
     * Starts a volley: one gate pulse per entry of the flywheel's pattern (NECTAR_PATTERN /
     * POLLEN_PATTERN), letting that many balls through. Spins the flywheel up first if it isn't
     * running. Nothing happens until update() is called every loop. Calling fire() again while a
     * volley is running cancels it and closes the gate.
     */
    public void fire(String type) {
        if (isFiring(type)) {
            cancelFire(type);
            return;
        }
        if (!(isNectar(type) ? isNectarRunning : isPollenRunning)) startAtSpeed(type);
        double now = now();
        int n = pattern(type).length;
        if (isNectar(type)) { nectarLeft = n; nectarOpenedAt = -1; nectarVolleyStart = now; }
        else { pollenLeft = n; pollenOpenedAt = -1; pollenVolleyStart = now; }
    }

    private int[] pattern(String type) {
        return isNectar(type) ? NECTAR_PATTERN : POLLEN_PATTERN;
    }

    /** Total balls one fire() shoots, for telemetry. */
    public int ballsPerVolley(String type) {
        int total = 0;
        for (int b : pattern(type)) total += b;
        return total;
    }

    public boolean isFiring(String type) {
        return (isNectar(type) ? nectarLeft : pollenLeft) > 0
                || (isNectar(type) ? isNectarOpen && nectarOpenedAt >= 0 : isPollenOpen && pollenOpenedAt >= 0);
    }

    public void cancelFire(String type) {
        if (isNectar(type)) { nectarLeft = 0; nectarOpenedAt = -1; }
        else { pollenLeft = 0; pollenOpenedAt = -1; }
        close(type);
    }

    /** Runs the gate pulses of any volley started with fire(). Call once per loop. */
    public void update() {
        updateVolley("nectar");
        updateVolley("pollen");
    }

    private void updateVolley(String type) {
        boolean nectar = isNectar(type);
        int left = nectar ? nectarLeft : pollenLeft;
        double openedAt = nectar ? nectarOpenedAt : pollenOpenedAt;
        double pulse = nectar ? nectarPulse : pollenPulse;
        double now = now();

        // Close the gate once the pulse is over.
        if (openedAt >= 0 && (nectar ? isNectarOpen : isPollenOpen) && now - openedAt >= pulse) {
            close(type);
            if (left == 0) {
                if (nectar) nectarOpenedAt = -1; else pollenOpenedAt = -1;
            }
        }
        if (left == 0) return;

        boolean ready;
        if (openedAt < 0) {
            // First ball: wait for the flywheel to reach speed (or give up waiting after MAX_SPINUP_SEC).
            double started = nectar ? nectarVolleyStart : pollenVolleyStart;
            ready = atSpeed(type) || now - started >= MAX_SPINUP_SEC;
        } else {
            // Later balls: let the previous ball hit and the flywheel recover.
            double since = now - openedAt;
            ready = since >= MAX_GAP_SEC || (since >= MIN_GAP_SEC && atSpeed(type));
        }
        if (!ready) return;

        int[] p = pattern(type);
        int balls = p[p.length - left];
        double next = balls >= 2
                ? (nectar ? NECTAR_PULSE2_SEC : POLLEN_PULSE2_SEC)
                : (nectar ? NECTAR_PULSE_SEC : POLLEN_PULSE_SEC);
        open(type);
        if (nectar) { nectarOpenedAt = now; nectarLeft--; nectarPulse = next; }
        else { pollenOpenedAt = now; pollenLeft--; pollenPulse = next; }
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    public double batteryVoltage() {
        return battery.getVoltage();
    }

    public void open(String type) {
        if (isNectar(type)) {
            nectarServo.setPosition(0.5);
            this.isNectarOpen = true;
        } else {
            pollenServo.setPosition(0);
            this.isPollenOpen = true;
        }
    }

    public void close(String type) {
        if (isNectar(type)) {
            nectarServo.setPosition(0.85);
            this.isNectarOpen = false;
        } else {
            pollenServo.setPosition(0.25);
            this.isPollenOpen = false;
        }
    }

}
