package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@Autonomous(name = "Digital Touch Sensor", group = "Exercises")
public class DigitalTouchSensor extends LinearOpMode {
    DigitalChannel touchSensor;

    @Override
    public void runOpMode() {

        // get a reference to our digitalTouch object.
        touchSensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");
        // set channel to read/input mode as we can only read from this pin
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while (opModeIsActive()) {
            // HIGH/true means the switch is pressed - short circuited
            if (touchSensor.getState() == true) {
                telemetry.addData("Touch Sensor", "Released");
            } else {
                telemetry.addData("Touch Sensor", "Pressed");
            }
            telemetry.update();
        }
    }
}
