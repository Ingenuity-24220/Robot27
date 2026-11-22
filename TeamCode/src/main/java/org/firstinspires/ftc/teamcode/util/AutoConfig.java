package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Shared hardware settings and gains for the custom tuners and CustomAutoRunner.
 * Replaces Pedro Pathing's util/pedro/Constants.java for autonomous driving.
 */
public class AutoConfig {

    // --- Motor names & directions (match Drivetrain.java; verify in Tuner 1) ---
    public static final String LF_NAME = "motorLF";
    public static final String LR_NAME = "motorLR";
    public static final String RF_NAME = "motorRF";
    public static final String RR_NAME = "motorRR";

    public static DcMotorSimple.Direction LF_DIR = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction LR_DIR = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction RF_DIR = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction RR_DIR = DcMotorSimple.Direction.FORWARD;

    // --- Pinpoint settings (match Localization.java; verify in Tuner 1) ---
    public static final String PINPOINT_NAME = "pinpoint";
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    public static GoBildaPinpointDriver.EncoderDirection X_POD_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection Y_POD_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // Sideways distance of forward pod (+ left, - right) and forward distance of strafe pod (+ forward, - back)
    public static double X_POD_OFFSET_MM = -62.5;
    public static double Y_POD_OFFSET_MM = -176.3;

    // --- Gains (starting values; copy tuned values in from Tuner 2) ---
    public static double kP_xy = 0.08;
    public static double kD_xy = 0.005;
    public static double kS_xy = 0.10;

    public static double kP_h = 1.2;
    public static double kD_h = 0.08;
    public static double kS_h = 0.08;

    public static double POS_TOL_IN = 1.0;
    public static double HEAD_TOL_DEG = 2.0;

    public static DcMotor[] initDriveMotors(HardwareMap hardwareMap) {
        DcMotor lf = hardwareMap.get(DcMotor.class, LF_NAME);
        DcMotor lr = hardwareMap.get(DcMotor.class, LR_NAME);
        DcMotor rf = hardwareMap.get(DcMotor.class, RF_NAME);
        DcMotor rr = hardwareMap.get(DcMotor.class, RR_NAME);

        lf.setDirection(LF_DIR);
        lr.setDirection(LR_DIR);
        rf.setDirection(RF_DIR);
        rr.setDirection(RR_DIR);

        DcMotor[] motors = {lf, lr, rf, rr};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        return motors;
    }

    /** Configures the Pinpoint and starts an IMU calibration; keep the robot still for ~0.3 s afterward. */
    public static GoBildaPinpointDriver initPinpoint(HardwareMap hardwareMap, boolean zeroOffsetsForCalibration) {
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        if (zeroOffsetsForCalibration) {
            odo.setOffsets(0.0, 0.0, DistanceUnit.MM);
        } else {
            odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        }
        odo.setEncoderResolution(POD_TYPE);
        odo.setEncoderDirections(X_POD_DIR, Y_POD_DIR);
        odo.resetPosAndIMU(); // calibrates IMU over ~250ms while stationary
        return odo;
    }

    /**
     * Drives in robot frame (+forward, +left, +ccw), writing motor powers directly
     * (no smoothing filter), with the same wheel mixing as Drivetrain.java.
     */
    public static void driveRaw(DcMotor[] motors, double forward, double left, double ccw, double maxPower) {
        double lateral = -left; // Drivetrain.java uses +lateral = right
        double yaw = -ccw;      // Drivetrain.java uses +yaw = CW

        double lf = forward + lateral + yaw;
        double lr = forward - lateral + yaw;
        double rf = forward - lateral - yaw;
        double rr = forward + lateral - yaw;

        double peak = Math.max(Math.max(Math.abs(lf), Math.abs(lr)),
                Math.max(Math.abs(rf), Math.abs(rr)));
        if (peak > maxPower && peak > 0) {
            double scale = maxPower / peak;
            lf *= scale;
            lr *= scale;
            rf *= scale;
            rr *= scale;
        }

        motors[0].setPower(lf);
        motors[1].setPower(lr);
        motors[2].setPower(rf);
        motors[3].setPower(rr);
    }

    public static void stop(DcMotor[] motors) {
        for (DcMotor m : motors) m.setPower(0);
    }

    // Low-pass filter on the D term, 0-1. Lower = smoother but laggier derivative.
    public static double D_FILTER = 0.3;

    /**
     * Fraction of kS to apply for an error of absErr: 0 below half the tolerance, rising
     * linearly to 1 at 1.5x the tolerance. A smooth ramp instead of an on/off step keeps
     * kS from kicking the robot back and forth around the tolerance edge.
     */
    public static double frictionRamp(double absErr, double tol) {
        return Math.max(0.0, Math.min(1.0, (absErr - 0.5 * tol) / tol));
    }

    // --- Control Hub IMU, used only to cross-check the Pinpoint's heading ---
    // Hub is mounted upright: REV logo faces the back of the robot, USB ports face the right.
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    public static IMU initHubImu(HardwareMap hardwareMap) {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        imu.resetYaw();
        return imu;
    }

    // --- Heading source (see util/Localization.java) ---
    // true: ignore the Pinpoint's IMU; compute X/Y from its raw pod counts with the hub IMU's heading.
    // false: use the Pinpoint's own pose.
    public static boolean USE_HUB_HEADING = true;

    // Used only when USE_HUB_HEADING is true. goBILDA 4-bar pod: 19.89436789 ticks/mm
    // (swingarm pod: 13.26291192). Signs: +1 if "Raw Encoders" X goes up when pushing the robot
    // forward and Y goes up when pushing it left in Tuner 1; otherwise -1.
    public static double TICKS_PER_MM = 19.89436789;
    public static double X_ENC_SIGN = 1;
    public static double Y_ENC_SIGN = 1;

    // --- Blocked moves (CustomAutoRunner) ---
    // true: a move whose robot stays stopped short of its target for BLOCKED_SEC (pushing into a
    // game element, for example the flower when intaking) ends early and the routine continues.
    // false: no early detection; a move waits for its timeout, and the team path ends if one fails.
    public static boolean CONTINUE_IF_BLOCKED = true;
    public static double BLOCKED_SEC = 1.5;          // how long to push while stopped before moving on
    public static double BLOCKED_SPEED_IN_S = 1.0;   // "stopped": slower than this...
    public static double BLOCKED_TURN_DEG_S = 5.0;   // ...and turning slower than this
    public static double BLOCKED_MIN_POWER = 0.12;   // ...while asking for at least this much power

    // --- Which side is the "front" in CustomAutoRunner (AUTO only; TeleOp is unaffected) ---
    // Every heading in the team path (routine 6: start pose, targets, turns) is the direction this
    // side faces. 180 = the outtake, on the back of the robot. 0 = the intake/drive front.
    // Test routines 1-5 always use the intake front.
    public static double AUTO_FRONT_OFFSET_DEG = 180;
}
