package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Robot.Robot;

@TeleOp(name = "ArmTOP", group = "1")
public class ArmTOP extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        waitForStart();
        robot.arm.resetEncoder();

        while (opModeIsActive()) {
            robot.arm.setPowerAdjusted(-gamepad1.left_stick_y);
            robot.arm.print();
            telemetry.update();
        }
    }
}
