package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

/**
 * Robot pose for the tuners and CustomAutoRunner, in Pedro field coordinates.
 *
 * With AutoConfig.USE_HUB_HEADING = false this is just the Pinpoint's own pose.
 * With it true, the Pinpoint's IMU is ignored: X/Y are integrated here from the Pinpoint's
 * two raw pod counts, using the Control Hub IMU for heading. The pod offset correction uses
 * the same convention as the Pinpoint (X_POD_OFFSET_MM = forward pod's sideways position,
 * + left; Y_POD_OFFSET_MM = strafe pod's forward position, + forward).
 *
 * Both headings are always read, so they can be compared in telemetry and logs.
 *
 * Pinpoint pods, directions, and offsets come from AutoConfig. The telemetry constructor plus
 * recalibrateOdometry() / runOdometry() are for TeleOps such as meow.java.
 */
public class Localization {
    private final GoBildaPinpointDriver pinpoint;
    private final IMU hub;
    private final boolean useHub = AutoConfig.USE_HUB_HEADING;
    private final ElapsedTime clock = new ElapsedTime();

    private double xOffMm = AutoConfig.X_POD_OFFSET_MM;
    private double yOffMm = AutoConfig.Y_POD_OFFSET_MM;

    // Hub-heading pose (field frame). Heading = hub yaw + hubOffset.
    private double xMm, yMm, headingRad, hubOffset;
    private int lastEncX, lastEncY;
    private double lastHubYaw, lastT;
    private boolean hasLast = false;

    private Pose2D pinpointPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    private double velXIn, velYIn, velHDeg, hubVelHDeg;

    // Optional, for recalibrateOdometry() / runOdometry().
    private Telemetry telemetry;
    private TelemetryManager panelsTelemetry;
    private double lastRunTime = 0;

    public Localization(HardwareMap hardwareMap) {
        pinpoint = AutoConfig.initPinpoint(hardwareMap, false);
        hub = AutoConfig.initHubImu(hardwareMap);
    }

    public Localization(HardwareMap hardwareMap, Telemetry telemetry, TelemetryManager panelsTelemetry) {
        this(hardwareMap);
        this.telemetry = telemetry;
        this.panelsTelemetry = panelsTelemetry;
    }

    public void update() {
        pinpoint.update();
        pinpointPose = pinpoint.getPosition();
        double t = clock.seconds();
        double yaw = hub.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        int encX = pinpoint.getEncoderX();
        int encY = pinpoint.getEncoderY();

        if (hasLast) {
            double dt = Math.max(t - lastT, 1e-4);
            double dTheta = AngleUnit.normalizeRadians(yaw - lastHubYaw);
            hubVelHDeg = Math.toDegrees(dTheta) / dt;

            // Pod travel in mm, then remove the part caused by rotation to get the robot center's motion.
            double dPar = AutoConfig.X_ENC_SIGN * (encX - lastEncX) / AutoConfig.TICKS_PER_MM;
            double dPerp = AutoConfig.Y_ENC_SIGN * (encY - lastEncY) / AutoConfig.TICKS_PER_MM;
            double dxRobot = dPar + xOffMm * dTheta;
            double dyRobot = dPerp - yOffMm * dTheta;

            // Rotate into the field frame at the midpoint heading of this step.
            double mid = headingRad + dTheta / 2;
            double dxField = dxRobot * Math.cos(mid) - dyRobot * Math.sin(mid);
            double dyField = dxRobot * Math.sin(mid) + dyRobot * Math.cos(mid);
            xMm += dxField;
            yMm += dyField;
            headingRad = AngleUnit.normalizeRadians(yaw + hubOffset);

            if (useHub) {
                velXIn = dxField / 25.4 / dt;
                velYIn = dyField / 25.4 / dt;
                velHDeg = hubVelHDeg;
            }
        } else {
            headingRad = AngleUnit.normalizeRadians(yaw + hubOffset);
        }
        lastEncX = encX;
        lastEncY = encY;
        lastHubYaw = yaw;
        lastT = t;
        hasLast = true;

        if (!useHub) {
            velXIn = pinpoint.getVelX(DistanceUnit.INCH);
            velYIn = pinpoint.getVelY(DistanceUnit.INCH);
            velHDeg = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        }
    }

    /** The pose used for driving. */
    public Pose2D getPosition() {
        return useHub ? new Pose2D(DistanceUnit.MM, xMm, yMm, AngleUnit.RADIANS, headingRad) : pinpointPose;
    }

    /** Sets the pose of both the Pinpoint and the hub-heading odometry. */
    public void setPosition(Pose2D pose) {
        pinpoint.setPosition(pose);
        double yaw = hub.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        xMm = pose.getX(DistanceUnit.MM);
        yMm = pose.getY(DistanceUnit.MM);
        headingRad = pose.getHeading(AngleUnit.RADIANS);
        hubOffset = headingRad - yaw;
        lastHubYaw = yaw;
    }

    /** Pod offsets for both the Pinpoint and the hub-heading odometry (0, 0 for offset calibration). */
    public void setOffsets(double xPodOffsetMm, double yPodOffsetMm) {
        pinpoint.setOffsets(xPodOffsetMm, yPodOffsetMm, DistanceUnit.MM);
        xOffMm = xPodOffsetMm;
        yOffMm = yPodOffsetMm;
    }

    /** Field-frame velocities of the pose used for driving: in/s and deg/s. */
    public double velX() { return velXIn; }
    public double velY() { return velYIn; }
    public double velH() { return velHDeg; }

    /** Both headings in the field frame (deg), for comparison. */
    public double hubHeadingDeg() { return Math.toDegrees(AngleUnit.normalizeRadians(lastHubYaw + hubOffset)); }
    public double hubVelH() { return hubVelHDeg; }
    public double pinpointHeadingDeg() { return pinpointPose.getHeading(AngleUnit.DEGREES); }
    public double pinpointVelH() { return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES); }

    public boolean usingHubHeading() { return useHub; }
    public GoBildaPinpointDriver pinpoint() { return pinpoint; }

    // ------------------------------------------------------------------------------------
    // TeleOp helpers (used by meow.java); these need the telemetry constructor.
    // ------------------------------------------------------------------------------------

    /** reset = true: zero the pose and recalibrate both IMUs (robot must be still). false: Pinpoint IMU only. */
    public void recalibrateOdometry(Boolean reset) {
        if (reset) {
            pinpoint.resetPosAndIMU();
            hub.resetYaw();
            setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));
            panelsTelemetry.addData("Status", "Initialized");
            panelsTelemetry.addData("X offset", xOffMm);
            panelsTelemetry.addData("Y offset", yOffMm);
            panelsTelemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
            panelsTelemetry.addData("Heading Scalar", pinpoint.getYawScalar());
            panelsTelemetry.addData("Heading source", useHub ? "HUB IMU" : "PINPOINT IMU");
            panelsTelemetry.update(telemetry);
        } else {
            pinpoint.recalibrateIMU();
        }
    }

    /** Updates the pose and shows it, plus velocities and loop rates, on both telemetries. */
    public void runOdometry() {
        update();

        Pose2D pos = getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                velXIn * 25.4, velYIn * 25.4, velHDeg);

        double now = clock.seconds();
        double frequency = 1 / Math.max(now - lastRunTime, 1e-4);
        lastRunTime = now;

        panelsTelemetry.addData("Position", data);
        panelsTelemetry.addData("Velocity", velocity);
        panelsTelemetry.addData("Heading source", useHub ? "HUB IMU" : "PINPOINT IMU");
        panelsTelemetry.addData("Status", String.valueOf(pinpoint.getDeviceStatus()));
        panelsTelemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        panelsTelemetry.addData("REV Hub Frequency: ", frequency);
        panelsTelemetry.update(telemetry);
    }
}
