package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "Detect", group = "Example")
public class Detect extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.vision.initScan();
        robot.vision.waitForStartAndScan();

        while(opModeIsActive()) {
            robot.drivetrain.moveToPosition(0.5, 0, 0, 24);
            robot.drivetrain.stop();
            break;
        }
        /*if(robot.vision.getDetectedTag() == Robot.AprilTag.LEFT) {
            robot.drivetrain.moveForSeconds(0, -0.4, 0, 1400);

            //robot.drivetrain.moveForSeconds(0.5, 0, 0, 1500);
        }
        else if(robot.vision.getDetectedTag() == Robot.AprilTag.RIGHT) {
            robot.drivetrain.moveForSeconds(0, 0.4, 0, 1400);
            //robot.drivetrain.moveForSeconds(0.5, 0, 0, 1500);
        }
        else {
            robot.drivetrain.moveForSeconds(0.5, 0, 0, 1500);
        }*/
    }
}
