package org.firstinspires.ftc.teamcode.util.Robot;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

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
    public TelemetryLogger console;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        drivetrain = new MecanumDrive("leftFront", "rightFront", "leftBack", "rightBack", this);
        drivetrain.setMotorDirection(true, false, true, false);
        drivetrain.setBrakeMode(true);

        lift = new Lift("leftLift", "rightLift", this);
        lift.setMotorDirection(true, false);

        arm = new Arm("arm", this);

        gripper = new Gripper("gripper", this);

        vision = new Vision("Webcam 1", this);

        runtime = new Runtime();
        console = new TelemetryLogger(telemetry);
    }
}
