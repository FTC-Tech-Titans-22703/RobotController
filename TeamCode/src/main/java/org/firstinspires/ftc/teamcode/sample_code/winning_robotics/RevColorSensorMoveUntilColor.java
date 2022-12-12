package org.firstinspires.ftc.teamcode.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Move Until Color", group="Exercises")
public class RevColorSensorMoveUntilColor extends LinearOpMode {
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
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        double alpha = 0;
        ColorRanges.ColorFromHue hueColor  = ColorRanges.GetColor(sensorColor.red(),
                sensorColor.green(), sensorColor.blue());
        while (opModeIsActive() && hueColor != ColorRanges.ColorFromHue.BLUE) {
            hueColor = ColorRanges.GetColor(sensorColor.red(),
                    sensorColor.green(), sensorColor.blue());
            telemetry.addData("Color",
                    hueColor.toString());
            telemetry.update();
        }
        // Set it so the motors brake when robot is done moving.
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set power levels.
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}