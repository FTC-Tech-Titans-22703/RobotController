package org.firstinspires.ftc.teamcode.util.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumDrive extends Subsystem {
    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;

    private double maxPower = 1;

    private final double WHEEL_RADIUS_INCHES = 1.88976377953;
    private final double MOTOR_CPR = 537.6;

    public MecanumDrive(String leftFront, String rightFront, String leftBack, String rightBack, Robot robot) {
        this.robot = robot;

        this.leftFront = robot.hardwareMap.get(DcMotorEx.class, leftFront);
        this.rightFront = robot.hardwareMap.get(DcMotorEx.class, rightFront);
        this.leftBack = robot.hardwareMap.get(DcMotorEx.class, leftBack);
        this.rightBack = robot.hardwareMap.get(DcMotorEx.class, rightBack);

        resetEncoders();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorDirection(DcMotorSimple.Direction leftFront, DcMotorSimple.Direction rightFront, DcMotorSimple.Direction leftBack, DcMotorSimple.Direction rightBack) {
        this.leftFront.setDirection(leftFront);
        this.rightFront.setDirection(rightFront);
        this.leftBack.setDirection(leftBack);
        this.rightBack.setDirection(rightBack);
    }

    public void setMotorDirection(boolean leftFront, boolean rightFront, boolean leftBack, boolean rightBack) {
        setMotorDirection(leftFront ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE,
                rightFront ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE,
                leftBack ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE,
                rightBack ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
    }

    public void setBrakeMode(boolean brakes) {
        setBrakeMode(brakes ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setBrakeMode(DcMotor.ZeroPowerBehavior brakes) {
        leftFront.setZeroPowerBehavior(brakes);
        rightFront.setZeroPowerBehavior(brakes);
        leftBack.setZeroPowerBehavior(brakes);
        rightBack.setZeroPowerBehavior(brakes);
    }

    public void setPower(double leftFront, double rightFront, double leftBack, double rightBack) {
        this.leftFront.setPower(clip(leftFront, maxPower));
        this.rightFront.setPower(clip(rightFront, maxPower));
        this.leftBack.setPower(clip(leftBack, maxPower));
        this.rightBack.setPower(clip(rightBack, maxPower));
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = clip(maxPower, 1);
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void move(double fwdBackPower, double strafePower, double turnPower) {
        setPower(fwdBackPower + turnPower + strafePower,
                fwdBackPower - turnPower - strafePower,
                fwdBackPower + turnPower - strafePower,
                fwdBackPower - turnPower + strafePower);
    }

    public void moveForSeconds(double fwdBackPower, double strafePower, double turnPower, int time) {
        move(fwdBackPower, strafePower, turnPower);
        robot.runtime.wait(time);
        stop();
        robot.runtime.wait(25);
    }

    public void moveDistance(double fwdBackPower, double strafePower, double turnPower, double distance) {
        moveToPosition(fwdBackPower, strafePower, turnPower, (int) (distance / ((2 * Math.PI * WHEEL_RADIUS_INCHES) / MOTOR_CPR)));
    }

    public void moveRotations(double fwdBackPower, double strafePower, double turnPower, double rotations) {
        moveToPosition(fwdBackPower, strafePower, turnPower, (int) (rotations * MOTOR_CPR));
    }

    public void moveToPosition(double fwdBackPower, double strafePower, double turnPower, int position) {
        resetEncoders();

        leftFront.setTargetPosition(position);
        rightFront.setTargetPosition(position);
        leftBack.setTargetPosition(position);
        rightBack.setTargetPosition(position);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        move(fwdBackPower, strafePower, turnPower);

        while(robot.opMode.opModeIsActive() && isBusy()) {
            robot.telemetry.addData("pos", leftFront.getCurrentPosition());
            robot.telemetry.update();
            robot.opMode.idle();
        }

        stop();
    }

    public boolean isBusy() {
        return rightFront.isBusy() || leftFront.isBusy() || rightBack.isBusy() || leftBack.isBusy();
    }

    public void stop() {
        setPower(0, 0, 0, 0);
    }

    public void setMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
    }

    public void resetEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}