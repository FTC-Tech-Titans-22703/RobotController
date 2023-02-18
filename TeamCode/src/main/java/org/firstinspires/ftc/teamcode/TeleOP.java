package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Robot.Arm;
import org.firstinspires.ftc.teamcode.util.Robot.Lift;
import org.firstinspires.ftc.teamcode.util.Robot.Robot;
import org.firstinspires.ftc.teamcode.util.Robot.Types.ArmPosition;
import org.firstinspires.ftc.teamcode.util.Robot.Types.GripperPosition;
import org.firstinspires.ftc.teamcode.util.Robot.Types.LiftPosition;

@TeleOp(name = "TeleOP", group = "0")
public class TeleOP extends LinearOpMode {
    private Robot robot;
    private double reverseDrive = 1;

    private boolean driveMode = true;

    private double slowSpeed = 0.15;
    private double fastSpeed = 0.75;

    public LiftPosition liftPosition;
    public ArmPosition armPosition;
    public GripperPosition gripperPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        liftPosition = LiftPosition.GROUND;
        armPosition = ArmPosition.FRONT;
        gripperPosition = GripperPosition.OPEN;

        waitForStart();

        robot.lift.resetEncoders();
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.arm.resetEncoder();
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()) {
            robot.drivetrain.move((driveMode ? gamepad1.left_stick_y : gamepad1.right_stick_y) * -reverseDrive * (gamepad1.right_trigger > 0.3 ? slowSpeed : fastSpeed), (driveMode ? gamepad1.left_stick_x : gamepad1.right_stick_x) * reverseDrive * (gamepad1.right_trigger > 0.3 ? slowSpeed : fastSpeed), (driveMode ? gamepad1.right_stick_x : gamepad1.left_stick_x) * reverseDrive * (gamepad1.right_trigger > 0.3 ? slowSpeed : fastSpeed));

            robot.lift.setPowerAdjusted(-gamepad2.left_stick_y);
            robot.arm.setPowerAdjusted(-gamepad2.right_stick_y);

            if(gamepad2.left_trigger > 0.5) {
                robot.gripper.close();
            }

            if(gamepad2.right_trigger > 0.5) {
                robot.gripper.open();
            }

            //Gamepad2 - Lift Positions
            if(gamepad2.dpad_down){
                liftPosition = LiftPosition.GROUND;
            }
            if(gamepad2.dpad_left){
                liftPosition = LiftPosition.LOW;
            }
            if(gamepad2.dpad_right){
                liftPosition = LiftPosition.MEDIUM;
            }
            if(gamepad2.dpad_up){
                liftPosition = LiftPosition.HIGH;
            }

            //Gamepad2 - Arm Positions
            if(gamepad2.left_bumper){
                armPosition = ArmPosition.FRONT;
            }
            if(gamepad2.right_bumper){
                armPosition = ArmPosition.BACK;
            }

            /*
            switch(liftPosition){
                case GROUND:
                    robot.lift.move(LiftPosition.GROUND.pos, 0.7);
                    break;
                case LOW:
                    robot.lift.move(LiftPosition.LOW.pos, 0.7);
                    break;
                case MEDIUM:
                    robot.lift.move(LiftPosition.MEDIUM.pos, 0.7);
                    break;
                case HIGH:
                    robot.lift.move(LiftPosition.HIGH.pos, 0.7);
                    break;
            }

            switch(armPosition){
                case FRONT:
                    robot.arm.move(Arm.MIN_POSITION, 0.7);
                    break;
                case BACK:
                    robot.arm.move(Arm.MAX_POSITION, 0.7);
                    break;
            }
            */

            /*
            switch(gripperPosition){
                case OPEN:
                    robot.gripper.open();
                    break;
                case CLOSED:
                    robot.gripper.close();
                    break;
            }
             */

            telemetry.addData("Reverse", reverseDrive == -1);
            //telemetry.addData("Speed", (int) (robot.drivetrain.getMaxPower() * 100) + "%");
            telemetry.update();
        }
    }
}
