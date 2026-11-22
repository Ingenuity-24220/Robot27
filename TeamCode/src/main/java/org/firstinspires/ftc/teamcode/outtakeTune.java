package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.Outtake;

import java.util.Locale;

/**
 * Outtake Tuner: velocity PIDF for both flywheels (nectar = outtakeMotor, pollen = outtakeMini).
 * See doc/CustomAuto.md, "Outtake Tuner", for the procedure.
 *
 *   X               switch flywheel (nectar / pollen)
 *   B               measure kF: runs the flywheel at its old fixed power for 3 s
 *   Y               spin up / stop at the target speed
 *   D-pad Up/Down   target speed +/- 20 ticks/s     (hold LB: kF +/- 1% instead)
 *   D-pad Right/Left  kP x1.25 / ÷1.25
 *   RB (hold)       open the gate to fire; shows the speed dip and recovery time
 *
 * Values edited here go straight into Outtake's static fields, so they last until the app
 * restarts. Copy the "Copy into Outtake.java" lines into Outtake.java to keep them.
 * Every loop is logged to /sdcard/FIRST/data/outtake_<time>.csv.
 */
@TeleOp(name = "Outtake Tuner")
public class outtakeTune extends OpMode {

    static final double MEASURE_SEC = 3.0;     // run time for the kF measurement
    static final double MEASURE_AVG_SEC = 1.0; // average the last second (after spin-up)
    static final double TARGET_STEP = 20;      // ticks/s per D-pad press

    Outtake outtake;
    TelemetryManager panels;
    DataLogger log;
    final ElapsedTime clock = new ElapsedTime();

    String wheel = "nectar";
    boolean running = false;
    double lastGainUpdate = 0;
    String message = "Press B to measure kF first";

    // kF measurement
    boolean measuring = false;
    double measureStart, velSum, voltSum;
    int samples;

    // Spin-up stats
    double spinStart = -1, riseTime = -1, peak = 0;
    // Shot stats
    boolean gateOpen = false;
    double shotTime = -1, dipMin = 0, recoverTime = -1;
    boolean dipped = false;

    @Override
    public void init() {
        outtake = new Outtake(hardwareMap, telemetry);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        log = new DataLogger("outtake",
                "t", "wheel", "running", "measuring", "target", "velocity", "error",
                "voltage", "kP", "kF", "kF_compensated", "gateOpen");
    }

    @Override
    public void loop() {
        double now = clock.seconds();
        double velocity = outtake.velocity(wheel);
        double voltage = outtake.batteryVoltage();

        // --- Controls ---
        if (gamepad1.xWasPressed() && !measuring) {
            stopWheel();
            setGate(false);
            wheel = isNectar() ? "pollen" : "nectar";
            message = "Switched to " + wheel;
        }

        if (gamepad1.bWasPressed() && !measuring) {
            stopWheel();
            measuring = true;
            measureStart = now;
            velSum = voltSum = 0;
            samples = 0;
            outtake.runOpenLoop(wheel, power());
            message = "Measuring kF at power " + power() + "...";
        }

        if (gamepad1.yWasPressed() && !measuring) {
            if (running) {
                stopWheel();
            } else if (target() <= 0) {
                message = "No target speed: press B to measure first, or D-pad Up";
            } else {
                running = true;
                outtake.start(wheel);
                spinStart = now;
                riseTime = -1;
                peak = 0;
            }
        }

        boolean lb = gamepad1.left_bumper;
        if (gamepad1.dpadUpWasPressed()) {
            if (lb) setKF(kF() * 1.01); else setTarget(target() + TARGET_STEP);
        }
        if (gamepad1.dpadDownWasPressed()) {
            if (lb) setKF(kF() * 0.99); else setTarget(Math.max(0, target() - TARGET_STEP));
        }
        if (gamepad1.dpadRightWasPressed()) setKP(kP() <= 0 ? 1.0 : kP() * 1.25);
        if (gamepad1.dpadLeftWasPressed()) setKP(kP() / 1.25);

        // RB held: gate open (fire). Track the speed dip and how long it takes to recover.
        if (gamepad1.right_bumper != gateOpen) {
            setGate(gamepad1.right_bumper);
            if (gateOpen) {
                shotTime = now;
                dipMin = velocity;
                dipped = false;
                recoverTime = -1;
            }
        }

        // --- kF measurement ---
        if (measuring) {
            double elapsed = now - measureStart;
            if (elapsed > MEASURE_SEC - MEASURE_AVG_SEC) {
                velSum += velocity;
                voltSum += voltage;
                samples++;
            }
            if (elapsed >= MEASURE_SEC) {
                measuring = false;
                outtake.stop(wheel);
                finishMeasurement(velSum / Math.max(samples, 1), voltSum / Math.max(samples, 1));
            }
        }

        // --- Live gains while running (refresh voltage compensation twice a second) ---
        if (running && now - lastGainUpdate > 0.5) {
            outtake.applyGains(wheel);
            lastGainUpdate = now;
        }

        // --- Stats ---
        double tol = Outtake.READY_TOLERANCE;
        if (running) {
            peak = Math.max(peak, velocity);
            if (riseTime < 0 && Math.abs(velocity - target()) < tol) riseTime = now - spinStart;
        }
        if (running && shotTime >= 0 && recoverTime < 0) {
            dipMin = Math.min(dipMin, velocity);
            if (velocity < target() - tol) dipped = true;
            if (dipped && Math.abs(velocity - target()) < tol) recoverTime = now - shotTime;
        }

        double target = running ? target() : 0;
        log.log(now, isNectar() ? 0 : 1, running ? 1 : 0, measuring ? 1 : 0, target, velocity,
                target - velocity, voltage, kP(), kF(), outtake.compensatedF(kF()), gateOpen ? 1 : 0);

        // --- Telemetry (graph "target" and "velocity" in Panels) ---
        String prefix = isNectar() ? "NECTAR" : "POLLEN";
        panels.addData("Flywheel (X)", wheel + (isNectar() ? " (outtakeMotor)" : " (outtakeMini)"));
        panels.addData("State", measuring ? "MEASURING" : running ? "RUNNING" : "STOPPED");
        panels.addData("Message", message);
        panels.addData("target", target);
        panels.addData("velocity", velocity);
        panels.addData("error", target - velocity);
        panels.addData("At speed", outtake.atSpeed(wheel));
        panels.addData("Overshoot (ticks/s)", running && peak > 0 ? fmt(peak - target()) : "-");
        panels.addData("Rise time (s)", riseTime >= 0 ? fmt(riseTime) : "-");
        panels.addData("Shot dip (ticks/s)", shotTime >= 0 ? fmt(target() - dipMin) : "-");
        panels.addData("Shot recovery (s)", recoverTime >= 0 ? fmt(recoverTime) : "-");
        panels.addData("Battery (V)", fmt(voltage));
        panels.addData("kP (D-pad R/L)", fmt(kP()));
        panels.addData("kF (LB + D-pad U/D)", fmt(kF()) + "  compensated " + fmt(outtake.compensatedF(kF())));
        panels.addData("Copy into Outtake.java", "");
        panels.addData("  " + prefix + "_VELOCITY", fmt(target()));
        panels.addData("  " + prefix + "_KP", fmt(kP()));
        panels.addData("  " + prefix + "_KF", fmt(kF()));
        panels.addData("Log file", log.path());
        panels.update(telemetry);
    }

    /**
     * The hub's velocity loop outputs kF * target / 32767 as feedforward power. At a fixed power p
     * the flywheel settles at velocity v, so kF = 32767 * p / v. Stored at NOMINAL_VOLTAGE.
     */
    private void finishMeasurement(double avgVel, double avgVolt) {
        if (avgVel < -100) {
            message = "Velocity is negative: the encoder counts backward. Reverse the motor direction.";
            return;
        }
        if (avgVel < 100) {
            message = "No speed reading (" + fmt(avgVel) + "). Is the encoder cable plugged in?";
            return;
        }
        double measuredF = 32767 * power() / avgVel;
        setKF(measuredF * avgVolt / Outtake.NOMINAL_VOLTAGE);
        if (target() <= 0) setTarget(Math.round(avgVel));
        if (kP() <= 0) setKP(0.1 * kF());
        message = String.format(Locale.US, "Measured %.0f ticks/s at %.2f V -> kF %.2f. Press Y.",
                avgVel, avgVolt, kF());
    }

    private void stopWheel() {
        running = false;
        outtake.stop(wheel);
    }

    private void setGate(boolean open) {
        gateOpen = open;
        if (open) outtake.open(wheel); else outtake.close(wheel);
    }

    private boolean isNectar() { return wheel.equals("nectar"); }

    private double power() { return isNectar() ? Outtake.NECTAR_POWER : Outtake.POLLEN_POWER; }
    private double target() { return isNectar() ? Outtake.NECTAR_VELOCITY : Outtake.POLLEN_VELOCITY; }
    private double kP() { return isNectar() ? Outtake.NECTAR_KP : Outtake.POLLEN_KP; }
    private double kF() { return isNectar() ? Outtake.NECTAR_KF : Outtake.POLLEN_KF; }

    private void setTarget(double v) {
        if (isNectar()) Outtake.NECTAR_VELOCITY = v; else Outtake.POLLEN_VELOCITY = v;
        if (running) {
            outtake.start(wheel); // sends the new target
            spinStart = clock.seconds();
            riseTime = -1;
            peak = 0;
        }
    }

    private void setKP(double v) {
        if (isNectar()) Outtake.NECTAR_KP = v; else Outtake.POLLEN_KP = v;
        if (running) outtake.applyGains(wheel);
    }

    private void setKF(double v) {
        if (isNectar()) Outtake.NECTAR_KF = v; else Outtake.POLLEN_KF = v;
        if (running) outtake.applyGains(wheel);
    }

    private static String fmt(double v) {
        return String.format(Locale.US, "%.2f", v);
    }

    @Override
    public void stop() {
        outtake.stop("nectar");
        outtake.stop("pollen");
        outtake.close("nectar");
        outtake.close("pollen");
        log.close();
    }
}
