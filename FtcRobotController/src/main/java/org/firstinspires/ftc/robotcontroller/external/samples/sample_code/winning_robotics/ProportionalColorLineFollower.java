package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Proportional Color Line Follower", group="Exercises")
public class ProportionalColorLineFollower extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    double minPower = 0.2;
    double maxPower = 0.3;
    double colorSensorMidPointAlpha = 72;
    ColorSensor sensorColor;
    @Override
    public void runOpMode() throws InterruptedException {
        sensorColor = hardwareMap.get(ColorSensor.class,
                "rev_color_sensor");
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        PController pController = new PController(0.1);
        pController.setInputRange(25, 169);
        pController.setSetPoint(colorSensorMidPointAlpha);
        pController.setOutputRange(minPower, maxPower);

        waitForStart();

        double alpha;
        while (opModeIsActive()) {

            alpha = sensorColor.alpha();
            if(alpha >= colorSensorMidPointAlpha) {
                // turn left
                leftMotor.setPower(minPower);
                rightMotor.setPower(minPower + pController.getComputedOutput(alpha));
                telemetry.addData("right",
                        minPower + pController.getComputedOutput(alpha));
            } else {
                // turn right

                leftMotor.setPower(minPower + pController.getComputedOutput(alpha));
                rightMotor.setPower(minPower);
                telemetry.addData("left",
                        minPower + pController.getComputedOutput(alpha));
            }
            telemetry.addData("alpha", alpha);
            telemetry.update();
        }
    }
}