package org.firstinspires.ftc.teamcode.util.Robot.Types;

public enum LiftPosition {
    GROUND(0), LOW(2), MEDIUM(3), HIGH(1630), MOVING_DOWN(-1), MOVING_UP(-1);

    public final int pos;
    LiftPosition(int pos){
        this.pos = pos;
    }
}
