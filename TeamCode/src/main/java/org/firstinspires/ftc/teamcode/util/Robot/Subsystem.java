package org.firstinspires.ftc.teamcode.util.Robot;

import com.qualcomm.robotcore.util.Range;

public abstract class Subsystem {
    private int numMotors;

    Robot robot;

    public double clip(double value, double max) {
        return max * Range.clip(value, -1, 1);
    }
}
