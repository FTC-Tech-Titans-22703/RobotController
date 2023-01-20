package org.firstinspires.ftc.teamcode.util.Robot.Types;

public enum LiftPosition {
    GROUND(0), LOW(690), MEDIUM(1165), HIGH(1640);

    public final int pos;
    LiftPosition(int pos){
        this.pos = pos;
    }
}
