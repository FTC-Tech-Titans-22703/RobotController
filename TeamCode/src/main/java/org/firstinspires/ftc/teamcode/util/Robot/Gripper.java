package org.firstinspires.ftc.teamcode.util.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Robot.Types.GripperPosition;

@Config
public class Gripper extends Subsystem {
    private final Servo gripper;
    private GripperPosition position;

    private final double MIN_POSITION = 0;
    private final double MAX_POSITION = 1;

    public Gripper(String gripper, Robot robot) {
        this.robot = robot;

        this.gripper = robot.hardwareMap.get(Servo.class, gripper);
        position = GripperPosition.OPEN;
        this.gripper.setDirection(Servo.Direction.REVERSE);
        this.gripper.scaleRange(MIN_POSITION, MAX_POSITION);
    }

    public void open() {
        //gripper.setDirection(Servo.Direction.FORWARD);
        position = GripperPosition.CLOSED;
        gripper.setPosition(1);
    }

    public void close() {
        //gripper.setDirection(Servo.Direction.REVERSE);
        position = GripperPosition.OPEN;
        gripper.setPosition(0);
    }

    public GripperPosition getPosition() {
        return position;
    }
}
