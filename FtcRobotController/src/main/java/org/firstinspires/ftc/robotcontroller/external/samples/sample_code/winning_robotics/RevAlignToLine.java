package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Rev Align To line", group="Exercises")
public class RevAlignToLine extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    double power = 0.3;
    ColorSensor sensorColor1;
    ColorSensor sensorColor2;

    @Override
    public void runOpMode() throws InterruptedException {
        sensorColor1 = hardwareMap.get(ColorSensor.class, "rev_color_sensor");
        sensorColor2 = hardwareMap.get(ColorSensor.class, "rev_color_sensor2");
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        leftMotor.setPower(power);
        rightMotor.setPower(power);
        double alpha = 0;

        //note that to get a good align to line, you will need to repeat the
        // process below 2-3 times
        while(opModeIsActive()) {
            if(sensorColor1.alpha() > 72) {
                leftMotor.setPower(0);
                while(sensorColor2.alpha() < 72) {
                    rightMotor.setPower(power);
                }
                rightMotor.setPower(0);

                break;
            }
            if(sensorColor2.alpha() > 72) {
                rightMotor.setPower(0);
                while(sensorColor1.alpha() < 72) {
                    leftMotor.setPower(power);
                }
                leftMotor.setPower(0);
                break;
            }
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}