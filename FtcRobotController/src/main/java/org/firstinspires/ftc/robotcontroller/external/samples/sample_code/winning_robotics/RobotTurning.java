package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.winning_robotics;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class RobotTurning extends LinearOpMode {
    RobotConfig robotConfig = new RobotConfig();
    private BNO055IMU imu;
    public void runOpMode() {
        robotConfig.init(hardwareMap);
        initializeIMU();
        waitForStart();
        turnRobotInDegrees(90, 0.3, robotConfig.leftDrive, robotConfig.rightDrive);
        turnRobotInDegrees(-90, 0.3, robotConfig.leftDrive, robotConfig.rightDrive);
    }

    public void initializeIMU() {
        this.imu = this.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        this.imu.initialize(parameters);

        if(!this.imu.isGyroCalibrated()) {
            this.sleep(50);
        }
    }
    public void turnRobotInDegrees(double angle, double power,
                                   DcMotor leftMotor, DcMotor rightMotor) {
        double startAngle = this.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double robotTurnedAngle = 0;

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while(this.opModeIsActive() &&
                Math.abs(robotTurnedAngle) < Math.abs(angle)) {
            robotTurnedAngle = this.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle;

            if(robotTurnedAngle > 180) {
                robotTurnedAngle = robotTurnedAngle - 360;
            }
            if(robotTurnedAngle < -180) {
                robotTurnedAngle = robotTurnedAngle + 360;
            }

            if(angle < 0) {
                leftMotor.setPower(-power);
                rightMotor.setPower(power);
            } else {
                leftMotor.setPower(power);
                rightMotor.setPower(-power);
            }
        }
    }
}
