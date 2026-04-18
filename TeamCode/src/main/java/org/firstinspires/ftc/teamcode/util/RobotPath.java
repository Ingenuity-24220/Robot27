package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.paths.PathChain;

public class RobotPath {
    public PathChain path;
    public double shootTime;
    public double shootingVelocity;
    public double indexerHelp_power;

    public double maxPower = 1;

    public RobotPath(PathChain path, double shootTime, double shootingVelocity, double indexerHelp_power, double maxPower)
    {
        this.path = path;
        this.shootTime = shootTime;
        this.shootingVelocity = shootingVelocity;
        this.indexerHelp_power = indexerHelp_power;
        this.maxPower = maxPower;
    }
}
