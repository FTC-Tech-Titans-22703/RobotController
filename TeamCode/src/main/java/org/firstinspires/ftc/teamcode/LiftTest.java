package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot.Lift;
import org.firstinspires.ftc.teamcode.util.Robot.Robot;
import org.firstinspires.ftc.teamcode.util.Robot.Types.LiftPosition;

@Autonomous(name = "Lift Test", group = "1")
public class LiftTest extends LinearOpMode {
    private Robot robot;
    private LiftPosition liftState = LiftPosition.GROUND;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Moving up", "");
            telemetry.update();
            robot.lift.move(Lift.MAX_POSITION, 1);
            telemetry.addData("Hold", "");
            telemetry.update();
            sleep(5000);
            telemetry.addData("Moving down", "");
            telemetry.update();
            robot.lift.move(-Lift.MAX_POSITION, 1);
            telemetry.addData("Hold", "");
            telemetry.update();
            sleep(5000);
        }
    }
}
