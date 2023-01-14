package org.firstinspires.ftc.teamcode.util.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm extends Subsystem {
    private final DcMotorEx arm;

    private double maxPower = 1;

    public Arm(String arm, Robot robot) {
        this.arm = robot.hardwareMap.get(DcMotorEx.class, arm);
    }

    public void setMotorDirection(boolean arm) {
        setMotorDirection(arm ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
    }

    public void setMotorDirection(DcMotorSimple.Direction arm) {
        this.arm.setDirection(arm);
    }

    public void setBrakeMode(boolean brakes) {
        setBrakeMode(brakes ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setBrakeMode(DcMotor.ZeroPowerBehavior brakes) {
        arm.setZeroPowerBehavior(brakes);
    }

    public void setPower(double power) {
        arm.setPower(clip(power, maxPower));
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
        if(runMode == DcMotor.RunMode.RUN_TO_POSITION) resetEncoder();

        arm.setMode(runMode);
    }

    public void resetEncoder() {
        DcMotor.RunMode prevMode = arm.getMode();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(prevMode);
    }
}
