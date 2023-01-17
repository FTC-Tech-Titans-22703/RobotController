package org.firstinspires.ftc.teamcode.util.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public LinearOpMode opMode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    public MecanumDrive drivetrain;
    public Lift lift;
    public Arm arm;
    public Gripper gripper;

    public Vision vision;

    public Runtime runtime;
    public TelemetryLogger logger;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        drivetrain = new MecanumDrive("leftFront", "rightFront", "leftRear", "rightRear", this);
        drivetrain.setMotorDirection(true, false, true, false);
        drivetrain.setBrakeMode(true);

        //lift = new Lift("leftLift", "rightLift", this);
        //lift.setMotorDirection(true, false);

        //arm = new Arm("arm", this);

        //gripper = new Gripper("gripper", this);

        //vision = new Vision("Webcam 1", this);

        runtime = new Runtime();
        logger = new TelemetryLogger(telemetry);
    }
}
