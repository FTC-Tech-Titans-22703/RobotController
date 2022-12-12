package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Claw with Servos sample", group="Exercises")
public class ClawOperation extends LinearOpMode {
    Servo leftServo;
    Servo rightServo;
    @Override
    public void runOpMode() throws InterruptedException {
        leftServo = hardwareMap.servo.get("left_claw_servo");
        rightServo = hardwareMap.servo.get("right_claw_servo");

        double leftServoOpenPosition = 0;
        double leftServoClosePosition = 0.3;

        waitForStart();

        while(opModeIsActive()) {
            // open the claw
            if(gamepad1.left_bumper == true) {
                leftServo.setPosition(leftServoOpenPosition);
                rightServo.setPosition(1 - leftServoOpenPosition);
            }

            // close the claw
            if(gamepad1.right_bumper == true) {
                leftServo.setPosition(leftServoClosePosition);
                rightServo.setPosition(1 - leftServoClosePosition);
            }

        }

    }
}