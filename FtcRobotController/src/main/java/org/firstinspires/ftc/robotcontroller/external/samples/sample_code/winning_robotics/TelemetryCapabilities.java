package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Telemetry Capabilities", group = "Exercises")
public class TelemetryCapabilities extends LinearOpMode{

    final int LOG_LINES = 3;
    ElapsedTime timer = new ElapsedTime();
    String sampleText = "Go Robot, Go!";
    Telemetry.Item gamepad1LeftStickItem;
    Telemetry.Item gamepad1RightStickItem;

    public void addFreeFormLog(String logText) {
        telemetry.log().add(logText);
    }

    public void initLogging() {
        telemetry.setAutoClear(false);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(LOG_LINES);
    }

    public void runOpMode() {
        initLogging();
        timer.reset();
        Telemetry.Item timerItem = telemetry.addData("time",
                "%.1f seconds", timer.seconds());
        // notice that the variable argument version of the addData function
        // allows us to put more values in one single line, allow us to display
        // higher density in our dashboard
        gamepad1LeftStickItem = telemetry.addData("values Lx, Ly",
                "| %.3f | %.3f|", gamepad1.left_stick_x, gamepad1.left_stick_y);
        gamepad1RightStickItem = telemetry.addData("values Rx, Ry",
                "| %.3f | %.3f|", gamepad1.right_stick_x, gamepad1.right_stick_y);

        waitForStart();
        while(opModeIsActive()) {
            timerItem.setValue("%.1f seconds", timer.seconds());
            gamepad1LeftStickItem.setValue("| %.3f | %.3f|",
                    gamepad1.left_stick_x, gamepad1.left_stick_y);
            gamepad1RightStickItem.setValue("| %.3f | %.3f|",
                    gamepad1.right_stick_x, gamepad1.right_stick_y);
            // free form text is appended at the end. You may place some easily
            // understandable warnings in here.
            addFreeFormLog(sampleText);
            telemetry.update(); // this pushes the data to the DS phone
        }
    }
}
