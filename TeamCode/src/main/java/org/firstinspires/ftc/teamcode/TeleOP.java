package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot.Robot;

@TeleOp(name = "TeleOP", group = "TeleOp")
public class TeleOP extends LinearOpMode {
    private Robot robot;
    private double reverseDrive = 1;

    private boolean driveMode = true;

    private double slowSpeed = 0.3;
    private double fastSpeed = 0.78;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        waitForStart();

        while(opModeIsActive()) {
            robot.drivetrain.move((driveMode ? gamepad1.left_stick_y : gamepad1.right_stick_y) * -reverseDrive * (gamepad1.right_trigger > 0.3 ? slowSpeed : fastSpeed), (driveMode ? gamepad1.left_stick_x : gamepad1.right_stick_x) * reverseDrive * (gamepad1.right_trigger > 0.3 ? slowSpeed : fastSpeed), (driveMode ? gamepad1.right_stick_x : gamepad1.left_stick_x) * reverseDrive * (gamepad1.right_trigger > 0.3 ? slowSpeed : fastSpeed));

            if(gamepad1.right_bumper) {
                robot.gripper.open();
            }

            if(gamepad1.left_bumper) {
                robot.gripper.close();
            }

            /*
            if(gamepad1.a) {
                reverseDrive = -reverseDrive;
            }

            if(gamepad1.b) {
                robot.drivetrain.setMaxPower(robot.drivetrain.getMaxPower() == 1 ? 0.3 : 1);
            }
            */

            telemetry.addData("Reverse", reverseDrive == -1);
            telemetry.addData("Speed", (int) (robot.drivetrain.getMaxPower() * 100) + "%");
            telemetry.update();
        }
    }
}
