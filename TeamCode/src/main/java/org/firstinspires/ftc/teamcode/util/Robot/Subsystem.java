package org.firstinspires.ftc.teamcode.util.Robot;

import com.qualcomm.robotcore.util.Range;

public abstract class Subsystem {
    private int numMotors;

    Robot robot;


    public double clip(double value, double max) {
        return Range.clip(value, -max, max);
    }
}
