package org.firstinspires.ftc.teamcode.util;

public class AngleCalculator_Right {
    public double getAngle(double x, double y)
    {
        double angle = java.lang.Math.atan2(y, x);

        if(angle < 0)
        {
            angle += 2*java.lang.Math.PI;
        }

        angle = angle * 180/java.lang.Math.PI;

        return angle;
    }

    public int findRegion(double x, double y)
    {

        double angle = getAngle(x, y);
        //int region_left = 0;
        int region_right = 0;

        //if(angle >= 22.5 && angle <= 67.5)
        if(angle >= 0 && angle <= 90 || angle >= 270 && angle <= 360)
        {
            region_right = 5;
            //turn_drivebase right
        }

        //if(angle >= 112.5 && angle <= 157.5)
        if(angle >= 90 && angle <= 270)
        {
            region_right = 6;
            //turn_drivebase left
        }

        return region_right;
    }
}

