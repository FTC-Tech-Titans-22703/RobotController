package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// this is the faked Maxbotix sensor, made to look like regular fully supported sensor

@Autonomous(name="Zig Zag Distance Sensor Wall Follower", group="Exercises")
public class DistanceSensorZigZagWallFollowerSample extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    double power = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        MaxbotixMB1242 distanceSensor = hardwareMap.get(MaxbotixMB1242.class,
                "maxbotix");
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        distanceSensor.resetDistance();
        while (opModeIsActive() &&
                distanceSensor.getDistance(DistanceUnit.CM) == 0 ) {
            // do nothing - the sensor has not yet gotten its first reading
        }
        double distance = 0;
        while (opModeIsActive()) {
            distance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance",
                    distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            if(distance > 30) {
                // turn left
                leftMotor.setPower(0);
                rightMotor.setPower(power);
            } else {
                // turn right
                leftMotor.setPower(power);
                rightMotor.setPower(0);
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