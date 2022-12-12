
package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Driving Utils Test harness", group ="Turning")
//@Disabled
public class DrivingUtilsTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    DcMotor leftDrive;
    DcMotor rightDrive;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "left_motor");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor");
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Positive
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // pass in this linear opmode as Driving Utils needs access to a few functions from here
        DrivingUtils drivingUtils = new DrivingUtils(this);

        drivingUtils.initIMU();

        waitForStart();
        runtime.reset();

        PController pController = new PController(0.05); // set the proportional constant to 0.05 or other suitable value
        pController.setSetPoint(90); // we need to reach 90 degrees, so setting it as the setpoint
        pController.setInputRange(0, 90); // assume that the robot will not overshoot at all
        pController.setOutputRange(0.2, 0.5);
        pController.setThresholdValue(3);

        drivingUtils.RobotMoveEncoderPositions(1120, 0.3, leftDrive, rightDrive);
        drivingUtils.turnDegreesPController(-90, pController, leftDrive, rightDrive);
        drivingUtils.turnDegreesPController(-90, pController, leftDrive, rightDrive);
        drivingUtils.turnDegreesPController(-90, pController, leftDrive, rightDrive);
        drivingUtils.turnDegreesPController(90, pController, leftDrive, rightDrive);
        drivingUtils.RobotMoveEncoderPositions(2240, 0.3, leftDrive, rightDrive);

    }
}