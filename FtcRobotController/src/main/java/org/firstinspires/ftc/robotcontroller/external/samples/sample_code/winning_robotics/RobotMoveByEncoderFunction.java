package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotMoveByEncoderFunction extends LinearOpMode{

    RobotConfig robotConfig = new RobotConfig();
    public void runOpMode() {
        waitForStart();
        // robot move forward
        RobotMoveEncoderPositions(20, 0.3,
                    robotConfig.leftDrive, robotConfig.rightDrive);
        // robot move backward
        RobotMoveEncoderPositions(-20, -0.3,
                robotConfig.leftDrive, robotConfig.rightDrive);
    }

    public void RobotMoveEncoderPositions(double inches,
                                          double RobotStraightMovePower,
                                          DcMotor leftMotor, DcMotor rightMotor) {
        final double EncoderPositionsPerInch = 89.17;

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int encoderPositionsNeeded =
                (int)(EncoderPositionsPerInch*inches);

        leftMotor.setTargetPosition(encoderPositionsNeeded);
        rightMotor.setTargetPosition(encoderPositionsNeeded);

        leftMotor.setPower(RobotStraightMovePower);
        rightMotor.setPower(RobotStraightMovePower);
        while (this.opModeIsActive() && leftMotor.isBusy()
                && rightMotor.isBusy()) {
            // Do nothing. We are waiting for the motors to report that they are done
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
