package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

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
 */
public class Odometry {
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

    public Odometry(HardwareMap hardwareMap) {
        pinpoint = AutoConfig.initPinpoint(hardwareMap, false);
        hub = AutoConfig.initHubImu(hardwareMap);
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
}
