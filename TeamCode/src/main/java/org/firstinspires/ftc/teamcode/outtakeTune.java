package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "outtakeTuning")
public class outtakeTune extends OpMode {

    static final double NOMINAL_VOLTAGE = 11.0;  // voltage F was tuned at
    static final double TOLERANCE = 50;          // ticks/sec "ready to fire" window

    double targetSpeed = 1450;  // ticks/sec
    double kF = 14.7;           // tuned at 1450 ticks/sec
    double kP = 23.52;           // starting point ≈ 0.1 × F

    DcMotorEx outtakeMotor;
    VoltageSensor batteryVoltageSensor;
    TelemetryManager panels;

    final ElapsedTime voltageTimer = new ElapsedTime();
    double voltage;
    double lastP = Double.NaN, lastF = Double.NaN;
    boolean running = false;

    @Override
    public void init() {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        // outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // if needed
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();

        panels = PanelsTelemetry.INSTANCE.getTelemetry();  // stored as a field now
    }

    @Override
    public void loop() {
        // Read voltage twice a second instead of every loop
        if (voltageTimer.seconds() > 0.5) {
            voltage = batteryVoltageSensor.getVoltage();
            voltageTimer.reset();
        }

        // --- Controls ---
        if (gamepad1.yWasPressed()) running = !running;    // Y: flywheel on/off

        if (gamepad1.dpadUpWasPressed())    kF += 0.1;     // fine-tune F
        if (gamepad1.dpadDownWasPressed())  kF -= 0.1;

        if (gamepad1.dpadRightWasPressed()) kP += 0.1;     // small P steps
        if (gamepad1.dpadLeftWasPressed())  kP -= 0.1;
        if (gamepad1.aWasPressed())         kP *= 2;       // A: double P
        if (gamepad1.bWasPressed())         kP /= 2;       // B: halve P (back off)
        kP = Math.max(0, kP);

        // --- Voltage-compensated F, only written to the hub when it changes ---
        double compensatedF = kF * (NOMINAL_VOLTAGE / voltage);
        if (kP != lastP || Math.abs(compensatedF - lastF) > 0.01) {
            outtakeMotor.setVelocityPIDFCoefficients(kP, 0, 0, compensatedF);
            lastP = kP;
            lastF = compensatedF;
        }

        double target = running ? targetSpeed : 0;
        outtakeMotor.setVelocity(target);
        double actual = outtakeMotor.getVelocity();
        boolean atSpeed = running && Math.abs(target - actual) < TOLERANCE;

        // --- Send numbers to Panels (graphable) and the Driver Hub ---
        panels.addData("target", target);
        panels.addData("velocity", actual);
        panels.addData("error", target - actual);
        panels.addData("voltage", voltage);
        panels.addData("atSpeed", atSpeed ? 1 : 0);
        panels.addData("kP", kP);
        panels.addData("kF (raw)", kF);
        panels.addData("kF (compensated)", compensatedF);
        panels.update(telemetry);
    }
}