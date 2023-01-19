package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Robot.Robot;

@TeleOp(name = "LiftTOP", group = "1")
public class LiftTOP extends LinearOpMode {
    private Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        waitForStart();
        robot.lift.resetEncoders();
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            robot.lift.setPowerAdjusted(-gamepad1.left_stick_y);
            robot.lift.print();
            telemetry.update();
        }
    }
}
