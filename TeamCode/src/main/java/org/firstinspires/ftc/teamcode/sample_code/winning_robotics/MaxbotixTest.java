package org.firstinspires.ftc.teamcode.sample_code.winning_robotics;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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