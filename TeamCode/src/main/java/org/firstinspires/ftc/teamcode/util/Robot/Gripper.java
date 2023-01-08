package org.firstinspires.ftc.teamcode.util.Robot;

import com.qualcomm.robotcore.hardware.Servo;

public class Gripper extends Subsystem {
    private Servo gripper;
    private GripperPosition position;

    private final double MIN_POSITION = 0.01;
    private final double MAX_POSITION = 0.05;

    public Gripper(String gripper, Robot robot) {
        this.robot = robot;

        this.gripper = robot.hardwareMap.get(Servo.class, gripper);
        position = GripperPosition.OPEN;
        this.gripper.scaleRange(MIN_POSITION, MAX_POSITION);
    }

    public void open() {
        //gripper.setDirection(Servo.Direction.FORWARD);
        position = GripperPosition.CLOSED;
        gripper.setPosition(0);
    }

    public void close() {
        //gripper.setDirection(Servo.Direction.REVERSE);
        position = GripperPosition.OPEN;
        gripper.setPosition(1);
    }

    public GripperPosition getPosition() {
        return position;
    }
}
