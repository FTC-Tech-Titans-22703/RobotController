package org.firstinspires.ftc.teamcode.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Rev Proportional Ultrasonic Wall Follower", group="Exercises")
public class RevProportionalToFDistanceWallFollower extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    double minPower = 0.15;
    double maxPower = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor revDistanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_range");
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        PController pController = new PController(0.08);
        pController.setInputRange(20, 40);
        pController.setSetPoint(30);
        pController.setOutputRange(minPower, maxPower);

        waitForStart();

        double distance = 0;
        while (opModeIsActive()) {

            distance = revDistanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance", revDistanceSensor.getDistance(DistanceUnit.CM));

            if(distance > 30) {
                // turn left
                leftMotor.setPower(minPower);
                rightMotor.setPower(minPower + pController.getComputedOutput(distance));
                telemetry.addData("right",  minPower + pController.getComputedOutput(distance));
            } else {
                leftMotor.setPower(minPower + pController.getComputedOutput(distance));
                rightMotor.setPower(minPower);
                telemetry.addData("left",  minPower + pController.getComputedOutput(distance));
            }

            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}