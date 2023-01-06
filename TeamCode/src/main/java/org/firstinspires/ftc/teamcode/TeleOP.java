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

        double driveFactor = 1;
        while(opModeIsActive()) {
            robot.drivetrain.move(gamepad1.right_stick_y * -driveFactor, gamepad1.right_stick_x * driveFactor, gamepad1.left_stick_x * driveFactor);

            if(gamepad1.right_trigger > 0.5) {
                robot.gripper.open();
            }
            if(gamepad1.left_trigger > 0.5) {
                robot.gripper.close();
            }

            if(gamepad1.a) {
                driveFactor = -driveFactor;
            }

            if(gamepad1.b) {
                robot.drivetrain.setMaxPower(robot.drivetrain.getMaxPower() == 1 ? 0.3 : 1);
            }

            telemetry.addData("Reverse", driveFactor == -1);
            telemetry.addData("Speed", (int) (robot.drivetrain.getMaxPower() * 100) + "%");
            telemetry.update();
        }
    }
}
