package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot.AprilTag;
import org.firstinspires.ftc.teamcode.util.Robot.Robot;

@Autonomous(name = "Park", group = "Main")
public class Park extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this);

        robot.vision.initScan();
        robot.vision.waitForStartAndScan();

        while(opModeIsActive()) {
            robot.runtime.wait(15000);
            robot.drivetrain.moveDistance(0.6, 0, 0, 25);
            if(robot.vision.getDetectedTag() == AprilTag.LEFT) {
                robot.drivetrain.moveDistance(0, -0.6, 0, 27);
            }
            else if(robot.vision.getDetectedTag() == AprilTag.RIGHT) {
                robot.drivetrain.moveDistance(0, 0.6, 0, 27);
            }
            break;
        }
    }
}
