package org.firstinspires.ftc.teamcode.util.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Lift extends Subsystem {
    private final DcMotorEx leftLift;
    private final DcMotorEx rightLift;

    private double maxPower = 1;

    public final static int MIN_POSITION = 0;
    public final static int MAX_POSITION = 215;

    public static double p = 8;
    public static double kP = 11.7;
    public static double kI = 4;
    public static double kD = 1.1;
    public static double kF = 10;

    public static int posThresholdTop = 30;
    public static int posThresholdBottom = 40;

    public static double slowSpeed = 0.4;

    public Lift(String leftLift, String rightLift, Robot robot) {
        this.robot = robot;

        this.leftLift = (DcMotorEx) robot.hardwareMap.get(DcMotorEx.class, leftLift);
        this.rightLift = (DcMotorEx) robot.hardwareMap.get(DcMotorEx.class, rightLift);

        setMotorDirection(false, true);

        resetEncoders();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.leftLift.setPositionPIDFCoefficients(p);
        this.leftLift.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        this.rightLift.setPositionPIDFCoefficients(p);
        this.rightLift.setVelocityPIDFCoefficients(kP, kI, kD, kF);
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

    public void setPowerAdjusted(double power) {
        if(power > 0.15 && Math.abs(rightLift.getCurrentPosition() - MAX_POSITION) > posThresholdTop && Math.abs(leftLift.getCurrentPosition() - MAX_POSITION) > posThresholdTop ||
                power < -0.15 && Math.abs(rightLift.getCurrentPosition() - MIN_POSITION) > posThresholdBottom && Math.abs(leftLift.getCurrentPosition() - MIN_POSITION) > posThresholdBottom) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(rightLift.getCurrentPosition() < 100 || leftLift.getCurrentPosition() < 100 || rightLift.getCurrentPosition() > MAX_POSITION - 100 || leftLift.getCurrentPosition() > MAX_POSITION - 100) {
                setPower(clip(power, slowSpeed));
            } else {
                setPower(power);
            }
        } else {
            if(Math.abs(rightLift.getCurrentPosition() - MIN_POSITION) <= posThresholdBottom || Math.abs(leftLift.getCurrentPosition() - MIN_POSITION) <= posThresholdBottom) {
                setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.opMode.idle();
            }

            leftLift.setTargetPosition(rightLift.getCurrentPosition());
            rightLift.setTargetPosition(rightLift.getCurrentPosition());
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setPower(double power) {
        leftLift.setPower(clip(power, maxPower));
        rightLift.setPower(clip(power, maxPower));
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = clip(maxPower, 1);
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void move(int targetPosition, double power) {
        resetEncoders();

        leftLift.setTargetPosition(targetPosition);
        rightLift.setTargetPosition(targetPosition);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPower(power);

        while(robot.opMode.opModeIsActive() && isBusy()) {
            /*
            robot.telemetry.addData("pos1", leftLift.getCurrentPosition());
            robot.telemetry.addData("pos2", rightLift.getCurrentPosition());
            robot.telemetry.update();*/

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
        leftLift.setMode(runMode);
        rightLift.setMode(runMode);
    }

    public void resetEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void print() {
        robot.telemetry.addData("posLeft", leftLift.getCurrentPosition());
        robot.telemetry.addData("posRight", rightLift.getCurrentPosition());
        robot.telemetry.addData("mode", leftLift.getMode());
    }

    public boolean isBusy() {
        return leftLift.isBusy() || rightLift.isBusy();
    }
}
