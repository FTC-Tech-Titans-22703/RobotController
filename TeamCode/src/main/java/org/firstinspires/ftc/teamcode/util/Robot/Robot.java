package org.firstinspires.ftc.teamcode.util.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDriveOdometry;

public class Robot {
    public LinearOpMode opMode;
    public MultipleTelemetry telemetry;
    public HardwareMap hardwareMap;
    public FtcDashboard dashboard;

    public MecanumDriveOdometry drivetrain;
    public Lift lift;
    public Arm arm;
    public Gripper gripper;

    public Vision vision;

    public Runtime runtime;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
     
        hardwareMap = opMode.hardwareMap;
        dashboard = FtcDashboard.getInstance();
        
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

        drivetrain = new MecanumDriveOdometry(hardwareMap);

        //lift = new Lift("leftLift", "rightLift", this);
        //lift.setMotorDirection(true, false);

        //arm = new Arm("arm", this);

        //gripper = new Gripper("gripper", this);

        //vision = new Vision("Webcam 1", this);

        runtime = new Runtime();
    }
}
