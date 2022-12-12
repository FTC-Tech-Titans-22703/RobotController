package org.firstinspires.ftc.teamcode.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Zig Zag Color Sensor Line Follower", group="Exercises")
public class REvColorSensorZigZagLineFollowerSample extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    double power = 0.2;
    ColorSensor sensorColor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensorColor = hardwareMap.get(ColorSensor.class, "rev_color_sensor");
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        double alpha = 0;
        while (opModeIsActive()) {
            alpha = sensorColor.alpha();
            if(alpha > 62.5) {
                // turn left
                leftMotor.setPower(power - 0.1);
                rightMotor.setPower(power);
            } else {
                // turn right
                leftMotor.setPower(power);
                rightMotor.setPower(power - 0.1);
            }
        }
        // Set it so the motors brake when robot is done moving.
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set power levels.
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}