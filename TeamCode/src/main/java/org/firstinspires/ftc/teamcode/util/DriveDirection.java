package org.firstinspires.ftc.teamcode.util;

public enum DriveDirection
{
    LEFT_TURN(1),
    RIGHT_TURN(2),
    LEFT_SHIFT(3),
    RIGHT_SHIFT(4),
    FORWARD(5),
    BACKWARD(6);

    private final int id;

    DriveDirection(int id)
    {
        this.id = id;
    }

    int getId()
    {
        return this.id;
    }
}
