package org.firstinspires.ftc.teamcode.util.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lift extends Subsystem {
    private final DcMotorEx leftLift;
    private final DcMotorEx rightLift;

    private double maxPower = 1;

    public int MIN_POSITION = 0;
    public int MAX_POSITION = 0;

    public Lift(String leftLift, String rightLift, Robot robot) {
        this.robot = robot;

        this.leftLift = robot.hardwareMap.get(DcMotorEx.class, leftLift);
        this.rightLift = robot.hardwareMap.get(DcMotorEx.class, rightLift);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorDirection(boolean leftLift, boolean rightLift) {
        setMotorDirection(leftLift ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE,
                rightLift ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
    }

    public void setMotorDirection(DcMotorSimple.Direction leftLift, DcMotorSimple.Direction rightLift) {
        this.leftLift.setDirection(leftLift);
        this.rightLift.setDirection(rightLift);
    }

    public void setBrakeMode(boolean brakes) {
        setBrakeMode(brakes ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setBrakeMode(DcMotor.ZeroPowerBehavior brakes) {
        leftLift.setZeroPowerBehavior(brakes);
        rightLift.setZeroPowerBehavior(brakes);
    }

    public void setPower(double power) {
        if(power > 0 && rightLift.getCurrentPosition() < MAX_POSITION || power < 0 && rightLift.getCurrentPosition() > MIN_POSITION) {
            leftLift.setPower(clip(power, maxPower));
            rightLift.setPower(clip(power, maxPower));
        }
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = clip(maxPower, 1);
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void move() {

    }

    public void moveForSeconds() {

    }

    public void moveToPosition() {

    }

    public void stop() {
        setPower(0);
    }

    public void setMode(DcMotor.RunMode runMode) {
        if(runMode == DcMotor.RunMode.RUN_TO_POSITION) resetEncoders();

        rightLift.setMode(runMode);
    }

    public void resetEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
