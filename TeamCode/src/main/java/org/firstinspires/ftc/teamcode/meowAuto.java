package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.Paths;
import com.pedropathing.paths.curves.bezier.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.util.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous
public class meowAuto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(59.521, 34.411, 180);
    private final Pose intermediatePose = p.of(23.026018099547514, 34.411199095022624, 180);
    private final Pose park = p.of(23.026018099547514, 65.8158042824116, 180);
    private final Pose controlPose = p.of(44.315045248868785, 47.79336594211078, 90);

    public Path linePath() {
        return Paths.line(startPose, intermediatePose);
    }
    public Path curvePath() {
        return Paths.curve(intermediatePose, controlPose, park);
    }
    public Path compoundPath() {
        return Paths.path(linePath(), curvePath());
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        assert follower != null;
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        schedule(follow(follower, compoundPath()));
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}