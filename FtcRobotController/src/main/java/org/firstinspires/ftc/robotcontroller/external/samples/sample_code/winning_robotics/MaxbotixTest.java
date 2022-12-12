package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="maxbotix test", group="Exercises")
public class MaxbotixTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MaxbotixMB1242 maxbotixus = hardwareMap.get(MaxbotixMB1242.class,
                "maxbotix");
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()) {
            telemetry.addData("Distance",
                    maxbotixus.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}