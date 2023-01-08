package org.firstinspires.ftc.teamcode.util.Robot;

enum AprilTag {
    NOT_FOUND(-1),
    LEFT(1),
    MIDDLE(2),
    RIGHT(3);

    public final int id;
    AprilTag(int id) {
        this.id = id;
    }
}