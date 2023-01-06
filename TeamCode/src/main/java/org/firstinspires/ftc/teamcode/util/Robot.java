package org.firstinspires.ftc.teamcode.util;

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

    public enum AprilTag {
        NOT_FOUND(-1),
        LEFT(1),
        MIDDLE(2),
        RIGHT(3);

        public final int id;
        AprilTag(int id) {
            this.id = id;
        }
    }

    public enum GripperPosition {
        OPEN,
        CLOSED;
    }

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        drivetrain = new MecanumDrive("leftFront", "rightFront", "leftBack", "rightBack");
        drivetrain.setMotorDirection(true, false, true, false);
        drivetrain.setBrakeMode(true);

        lift = new Lift("leftLift", "rightLift");
        lift.setMotorDirection(true, false);

        arm = new Arm("arm");

        gripper = new Gripper("gripper");

        vision = new Vision("Webcam 1");

        runtime = new Runtime();
        console = new TelemetryLogger();
    }

    public class MecanumDrive {
        private final DcMotor leftFront;
        private final DcMotor rightFront;
        private final DcMotor leftBack;
        private final DcMotor rightBack;

        private double maxPower = 1;

        private final double WHEEL_RADIUS_INCHES = 1.88976377953;
        private final double MOTOR_CPR = 537.6;

        public MecanumDrive(String leftFront, String rightFront, String leftBack, String rightBack) {
            this(hardwareMap.get(DcMotor.class, leftFront), hardwareMap.get(DcMotor.class, rightFront), hardwareMap.get(DcMotor.class, leftBack), hardwareMap.get(DcMotor.class, rightBack));
        }

        public MecanumDrive(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
            this.leftFront = leftFront;
            this.rightFront = rightFront;
            this.leftBack = leftBack;
            this.rightBack = rightBack;

            resetEncoders();
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void setMotorDirection(DcMotorSimple.Direction leftFront, DcMotorSimple.Direction rightFront, DcMotorSimple.Direction leftBack, DcMotorSimple.Direction rightBack) {
            this.leftFront.setDirection(leftFront);
            this.rightFront.setDirection(rightFront);
            this.leftBack.setDirection(leftBack);
            this.rightBack.setDirection(rightBack);
        }

        public void setMotorDirection(boolean leftFront, boolean rightFront, boolean leftBack, boolean rightBack) {
            setMotorDirection(leftFront ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE,
                    rightFront ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE,
                    leftBack ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE,
                    rightBack ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        }

        public void setBrakeMode(boolean brakes) {
            setBrakeMode(brakes ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public void setBrakeMode(DcMotor.ZeroPowerBehavior brakes) {
            leftFront.setZeroPowerBehavior(brakes);
            rightFront.setZeroPowerBehavior(brakes);
            leftBack.setZeroPowerBehavior(brakes);
            rightBack.setZeroPowerBehavior(brakes);
        }

        public void setPower(double leftFront, double rightFront, double leftBack, double rightBack) {
            this.leftFront.setPower(clip(leftFront, maxPower));
            this.rightFront.setPower(clip(rightFront, maxPower));
            this.leftBack.setPower(clip(leftBack, maxPower));
            this.rightBack.setPower(clip(rightBack, maxPower));
        }

        public void setMaxPower(double maxPower) {
            this.maxPower = clip(maxPower, 1);
        }

        public double getMaxPower() {
            return maxPower;
        }

        public void move(double fwdBackPower, double strafePower, double turnPower) {
            setPower(fwdBackPower + turnPower + strafePower,
                    fwdBackPower - turnPower - strafePower,
                    fwdBackPower + turnPower - strafePower,
                    fwdBackPower - turnPower + strafePower);
        }

        public void moveForSeconds(double fwdBackPower, double strafePower, double turnPower, int time) {
            drivetrain.move(fwdBackPower, strafePower, turnPower);
            runtime.wait(time);
            drivetrain.stop();
            runtime.wait(25);
        }

        public void moveDistance(double fwdBackPower, double strafePower, double turnPower, double distance) {
            moveToPosition(fwdBackPower, strafePower, turnPower, (int) (distance / ((2 * Math.PI * WHEEL_RADIUS_INCHES) / MOTOR_CPR)));
        }

        public void moveRotations(double fwdBackPower, double strafePower, double turnPower, double rotations) {
            moveToPosition(fwdBackPower, strafePower, turnPower, (int) (rotations * MOTOR_CPR));
        }

        public void moveToPosition(double fwdBackPower, double strafePower, double turnPower, int position) {
            drivetrain.resetEncoders();

            leftFront.setTargetPosition(position);
            rightFront.setTargetPosition(position);
            leftBack.setTargetPosition(position);
            rightBack.setTargetPosition(position);

            drivetrain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drivetrain.move(fwdBackPower, strafePower, turnPower);

            while(opMode.opModeIsActive() && isBusy()) {
                telemetry.addData("pos", leftFront.getCurrentPosition());
                telemetry.update();
                opMode.idle();
            }

            drivetrain.stop();
        }

        public boolean isBusy() {
            return rightFront.isBusy() || leftFront.isBusy() || rightBack.isBusy() || leftBack.isBusy();
        }

        public void stop() {
            setPower(0, 0, 0, 0);
        }

        public void setMode(DcMotor.RunMode runMode) {
            leftFront.setMode(runMode);
            rightFront.setMode(runMode);
            leftBack.setMode(runMode);
            rightBack.setMode(runMode);
        }

        public void resetEncoders() {
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public class Lift {
        private final DcMotor leftLift;
        private final DcMotor rightLift;

        private double maxPower = 1;

        public int MIN_POSITION = 0;
        public int MAX_POSITION = 0;

        public Lift(String leftLift, String rightLift) {
            this(hardwareMap.get(DcMotor.class, leftLift), hardwareMap.get(DcMotor.class, rightLift));
        }

        public Lift(DcMotor leftLift, DcMotor rightLift) {
            this.leftLift = leftLift;
            this.rightLift = rightLift;

            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void setMotorDirection(boolean leftLift, boolean rightLift) {
            setMotorDirection(leftLift ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE,
                    rightLift ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        }

        public void setMotorDirection(DcMotorSimple.Direction leftLift, DcMotorSimple.Direction rightLift) {
            this.leftLift.setDirection(leftLift);
            this.rightLift.setDirection(rightLift);
        }

        public void setBrakeMode(boolean brakes) {
            setBrakeMode(brakes ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public void setBrakeMode(DcMotor.ZeroPowerBehavior brakes) {
            leftLift.setZeroPowerBehavior(brakes);
            rightLift.setZeroPowerBehavior(brakes);
        }

        public void setPower(double power) {
            if(power > 0 && rightLift.getCurrentPosition() < MAX_POSITION || power < 0 && rightLift.getCurrentPosition() > MIN_POSITION) {
                leftLift.setPower(clip(power, maxPower));
                rightLift.setPower(clip(power, maxPower));
            }
        }

        public void setMaxPower(double maxPower) {
            this.maxPower = clip(maxPower, 1);
        }

        public double getMaxPower() {
            return maxPower;
        }

        public void move() {

        }

        public void moveForSeconds() {

        }

        public void moveToPosition() {

        }

        public void stop() {
            setPower(0);
        }

        public void setMode(DcMotor.RunMode runMode) {
            if(runMode == DcMotor.RunMode.RUN_TO_POSITION) resetEncoders();

            rightLift.setMode(runMode);
        }

        public void resetEncoders() {
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public class Arm {
        private final DcMotor arm;

        private double maxPower = 1;

        public Arm(String arm) {
            this(hardwareMap.get(DcMotor.class, arm));
        }

        public Arm(DcMotor arm) {
            this.arm = arm;
        }

        public void setMotorDirection(boolean arm) {
            setMotorDirection(arm ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        }

        public void setMotorDirection(DcMotorSimple.Direction arm) {
            this.arm.setDirection(arm);
        }

        public void setBrakeMode(boolean brakes) {
            setBrakeMode(brakes ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public void setBrakeMode(DcMotor.ZeroPowerBehavior brakes) {
            arm.setZeroPowerBehavior(brakes);
        }

        public void setPower(double power) {
            arm.setPower(clip(power, maxPower));
        }

        public void setMaxPower(double maxPower) {
            this.maxPower = clip(maxPower, 1);
        }

        public double getMaxPower() {
            return maxPower;
        }

        public void move() {

        }

        public void moveForSeconds() {

        }

        public void moveToPosition() {

        }

        public void stop() {
            setPower(0);
        }

        public void setMode(DcMotor.RunMode runMode) {
            if(runMode == DcMotor.RunMode.RUN_TO_POSITION) resetEncoder();

            arm.setMode(runMode);
        }

        public void resetEncoder() {
            DcMotor.RunMode prevMode = arm.getMode();
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMode(prevMode);
        }
    }

    public class Gripper {
        private final Servo gripper;
        private GripperPosition position;

        private final double MIN_POSITION = 0.01;
        private final double MAX_POSITION = 0.05;

        public Gripper(String gripper) {
            this(hardwareMap.get(Servo.class, gripper));
        }

        public Gripper(Servo gripper) {
            this.gripper = gripper;
            position = GripperPosition.OPEN;

            gripper.scaleRange(MIN_POSITION, MAX_POSITION);
        }

        public void open() {
            //gripper.setDirection(Servo.Direction.FORWARD);
            position = GripperPosition.CLOSED;
            gripper.setPosition(0);
        }

        public void close() {
            //gripper.setDirection(Servo.Direction.REVERSE);
            position = GripperPosition.OPEN;
            gripper.setPosition(1);
        }

        public GripperPosition getPosition() {
            return position;
        }
    }

    public class Vision {
        private final WebcamName webcam;
        private OpenCvCamera camera;
        private AprilTagDetectionPipeline aprilTagDetectionPipeline;

        static final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        private final double fx = 578.272;
        private final double fy = 578.272;
        private final double cx = 402.145;
        private final double cy = 221.506;

        // UNITS ARE METERS
        private final double tagsize = 0.06429863382690999;

        private AprilTagDetection detectedTag = null;

        public Vision(String webcam) {
            this(hardwareMap.get(WebcamName.class, webcam));
        }

        public Vision(WebcamName webcam) {
            this.webcam = webcam;
        }

        public void initScan() {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            //Camera initializing and streaming/error handling
            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });

            telemetry.setMsTransmissionInterval(50);
        }

        public void waitForStartAndScan() {
            /*
             * The INIT-loop:
             * This REPLACES waitForStart!
             */

            while(!opMode.isStarted() && !opMode.isStopRequested()) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if(currentDetections.size() != 0) {
                    boolean tagFound = false;
                    for(AprilTagDetection tag : currentDetections) {
                        if(tag.id == AprilTag.LEFT.id || tag.id == AprilTag.MIDDLE.id || tag.id == AprilTag.RIGHT.id) {
                            detectedTag = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if(tagFound) {
                        telemetry.addLine("Tag Detected:");
                        tagToTelemetry(detectedTag);
                    }
                    else {
                        telemetry.addLine("Tag NOT Detected");

                        if(detectedTag == null) {
                            telemetry.addLine("(Tag has never been detected)");
                        } else {
                            telemetry.addLine("Previous Tag:");
                            tagToTelemetry(detectedTag);
                        }
                    }

                }
                else {
                    telemetry.addLine("Tag NOT Detected");

                    if(detectedTag == null) {
                        telemetry.addLine("(Tag has never been detected)");
                    }
                    else {
                        telemetry.addLine("Previous Tag:");
                        tagToTelemetry(detectedTag);
                    }
                }

                telemetry.update();
                opMode.sleep(20);
            }

            telemetry.clearAll();
            camera.stopStreaming();
        }

        private @SuppressLint("DefaultLocale")
        void tagToTelemetry(AprilTagDetection detection) {
            telemetry.addLine("\nDetected Tag = " + getDetectedTag() + " (id: " + detection.id + ")");
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }

        public AprilTag getDetectedTag() {
            if(detectedTag.id == -1) return AprilTag.NOT_FOUND;
            if(detectedTag.id == 1) return AprilTag.LEFT;
            if(detectedTag.id == 2) return AprilTag.MIDDLE;
            return AprilTag.RIGHT;
        }
    }

    //Runtime
    public static class Runtime {
        private final ElapsedTime runtime;

        public Runtime() {
            runtime = new ElapsedTime();
        }

        public void reset() {
            runtime.reset();
        }

        public void delay(int millis) {
            try {
                Thread.sleep(millis);
            } catch(InterruptedException ignored) {}
        }

        public int getTime() {
            return (int) runtime.milliseconds();
        }

        public void wait(int time) {
            reset();
            while(runtime.milliseconds() < time) {}
        }
    }

    //Telemetry display
    public class TelemetryLogger {
        private final HashMap<String, Object> data;

        public TelemetryLogger() {
            data = new HashMap<>();
        }

        public void update(String caption, Object value) {
            update(caption, value, true);
        }

        public void update(String caption, Object value, boolean update) {
            data.put(caption, value);
            if(update) {
                for(Map.Entry<String, Object> element : data.entrySet()) {
                    telemetry.addData(element.getKey(), element.getValue());
                }

                telemetry.update();
            }
        }
    }

    private double clip(double value, double max) {
        return Range.clip(value, -max, max);
    }
}
