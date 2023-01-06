package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "TeleOP", group = "main")
public class TeleOP extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        waitForStart();

        boolean reverseDrive = true;
        double driveFactor = 1;
        while(opModeIsActive()) {
            robot.drivetrain.setMotorDirection(reverseDrive, !reverseDrive, reverseDrive, !reverseDrive);
            robot.drivetrain.move(gamepad1.right_stick_y * driveFactor, gamepad1.right_stick_x * driveFactor, gamepad1.left_stick_x * driveFactor);

            if(gamepad1.right_trigger > 0.5) {
                robot.gripper.open();
            }
            if(gamepad1.left_trigger > 0.5) {
                robot.gripper.close();
            }

            if(gamepad1.a) {
                reverseDrive = !reverseDrive;
            }

            if(gamepad1.b) {
                driveFactor = driveFactor == 1 ? 0.3 : 1;
            }
        }
    }
}
