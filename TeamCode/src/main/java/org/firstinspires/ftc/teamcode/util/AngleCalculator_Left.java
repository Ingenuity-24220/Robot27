package org.firstinspires.ftc.teamcode.util;

public class AngleCalculator_Left {
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
        int region_left = 0;
        //int region_right = 0;

        //if((angle >= 0 && angle <= 22.5) || (angle >= 337.5 && angle <= 360))
        if((angle >= 0 && angle <= 45) || (angle >= 315 && angle <= 360))
        {
            region_left = 1;
            //sideways right
            //System.out.println("REGION 1");
        }

        if(angle >= 45 && angle <= 135)
        {
            region_left = 2;
            //forward
        }

        if(angle >= 135 && angle <= 225)
        {
            region_left = 3;
            //sideways left
        }

        if(angle >= 225 && angle <= 315)
        {
            region_left = 4;
            //backwards
        }
        return region_left;
    }
}
