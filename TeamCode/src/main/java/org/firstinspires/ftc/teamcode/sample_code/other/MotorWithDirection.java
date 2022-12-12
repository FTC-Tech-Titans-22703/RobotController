package org.firstinspires.ftc.teamcode.sample_code.other;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorWithDirection{
    public DcMotor getMotor() {
        return motor;
    }

    public boolean getDirection() {
        return direction;
    }

    DcMotor motor;
    boolean direction; // true is forwards
    public MotorWithDirection(DcMotor motor, boolean direction)
    {
        this.motor = motor;
        this.direction = direction;
    }
}
