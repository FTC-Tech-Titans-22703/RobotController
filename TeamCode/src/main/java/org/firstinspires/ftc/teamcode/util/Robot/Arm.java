package org.firstinspires.ftc.teamcode.util.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class Arm extends Subsystem {
    private final DcMotorEx leftArm;
    private final DcMotorEx rightArm;

    private double maxPower = 0.8;

    public final static int MIN_POSITION = 0;
    public final static int MAX_POSITION = 200;

    public static double p = 12;
    public static double kP = 12;
    public static double kI = 5;
    public static double kD = 0;
    public static double kF = 22;

    public static int posThresholdTop = 7;
    public static int posThresholdBottom = 7;

    public static double slowSpeed = 0.3;

    public Arm(String leftArm, String rightArm, Robot robot) {
        this.robot = robot;

        this.leftArm = robot.hardwareMap.get(DcMotorEx.class, leftArm);
        this.rightArm = robot.hardwareMap.get(DcMotorEx.class, rightArm);

        setMotorDirection(false, true);

        resetEncoder();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.leftArm.setPositionPIDFCoefficients(p);
        this.leftArm.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        this.rightArm.setPositionPIDFCoefficients(p);
        this.rightArm.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    public void setMotorDirection(boolean leftArm, boolean rightArm) {
        setMotorDirection(leftArm ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE,
                rightArm ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
    }

    public void setMotorDirection(DcMotorSimple.Direction leftArm, DcMotorSimple.Direction rightArm) {
        this.leftArm.setDirection(leftArm);
        this.rightArm.setDirection(rightArm);
    }

    public void setBrakeMode(boolean brakes) {
        setBrakeMode(brakes ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setBrakeMode(DcMotor.ZeroPowerBehavior brakes) {
        leftArm.setZeroPowerBehavior(brakes);
        rightArm.setZeroPowerBehavior(brakes);
    }


    public void setPowerAdjusted(double power) {
        if(power > 0.15 && Math.abs(leftArm.getCurrentPosition() - MAX_POSITION) > posThresholdTop && Math.abs(rightArm.getCurrentPosition() - MAX_POSITION) > posThresholdTop ||
                power < -0.15 && Math.abs(leftArm.getCurrentPosition() - MIN_POSITION) > posThresholdBottom && Math.abs(rightArm.getCurrentPosition() - MIN_POSITION) > posThresholdBottom) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if((rightArm.getCurrentPosition() < 20 || leftArm.getCurrentPosition() < 20) && power > 0.15 || (rightArm.getCurrentPosition() > MAX_POSITION - 20 || leftArm.getCurrentPosition() > MAX_POSITION - 20) && power < -0.15){
                setPower(clip(power, 0.7));
            }
            else{
                setPower(clip(power, slowSpeed));
            }
        } else {
            if(Math.abs(leftArm.getCurrentPosition() - MIN_POSITION) <= posThresholdBottom) {
                setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.opMode.idle();
            }

            leftArm.setTargetPosition(leftArm.getCurrentPosition());
            rightArm.setTargetPosition(leftArm.getCurrentPosition());
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setPower(double power) {
        leftArm.setPower(clip(power, maxPower));
        rightArm.setPower(clip(power, maxPower));
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = clip(maxPower, 1);
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void move(int targetPosition, double power) {
        resetEncoder();

        leftArm.setTargetPosition(targetPosition);
        rightArm.setTargetPosition(targetPosition);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPower(power);

        while(robot.opMode.opModeIsActive() && isBusy()) {
            robot.opMode.idle();
        }
    }

    public void moveForSeconds() {

    }

    public void moveToPosition() {

    }

    public void stop() {
        setPower(0);
    }

    public void setMode(DcMotor.RunMode runMode) {
        leftArm.setMode(runMode);
        rightArm.setMode(runMode);
    }

    public void resetEncoder() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void print() {
        robot.telemetry.addData("left", leftArm.getCurrentPosition());
        robot.telemetry.addData("right", rightArm.getCurrentPosition());
    }

    public boolean isBusy() {
        return leftArm.isBusy() || rightArm.isBusy();
    }
}
