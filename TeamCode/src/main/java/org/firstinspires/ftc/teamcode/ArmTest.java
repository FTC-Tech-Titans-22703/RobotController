package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot.Arm;
import org.firstinspires.ftc.teamcode.util.Robot.Lift;
import org.firstinspires.ftc.teamcode.util.Robot.Robot;
import org.firstinspires.ftc.teamcode.util.Robot.Types.ArmPosition;
import org.firstinspires.ftc.teamcode.util.Robot.Types.LiftPosition;

@Autonomous(name = "Arm Test", group = "1")
public class ArmTest extends LinearOpMode {
    private Robot robot;
    private ArmPosition armState = ArmPosition.FRONT;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Moving to back", "");
            telemetry.update();
            robot.lift.move(Arm.MAX_POSITION, 1);
            telemetry.addData("Hold", "");
            telemetry.update();
            sleep(5000);
            telemetry.addData("Moving to front", "");
            telemetry.update();
            robot.lift.move(-Arm.MAX_POSITION, 1);
            telemetry.addData("Hold", "");
            telemetry.update();
            sleep(5000);
        }
    }
}
