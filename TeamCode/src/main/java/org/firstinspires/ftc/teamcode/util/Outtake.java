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
        return isNectar(type) ? NECTAR_VELOCITY : POLLEN_VELOCITY;
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
