package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    public Vision vision;
    public MecanumDrive drivetrain;
    public Intake intake;

    //AprilTag enum
    public enum AprilTag {
        LEFT(1),
        MIDDLE(2),
        RIGHT(3);

        public final int id;
        AprilTag(int id) {
            this.id = id;
        }

        public static AprilTag getTag(int id) {
            if(id == 1) return LEFT;
            if(id == 2) return MIDDLE;
            return RIGHT;
        }
    }

    public Runtime runtime;
    public TelemetryLogger console;

    //Robot Constructor
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        drivetrain = new MecanumDrive("leftFront", "rightFront", "leftBack", "rightBack");
        drivetrain.setMotorDirection(true, false, true, false);

        intake = new Intake("leftSpinner", "rightSpinner");
        intake.setMotorDirection(true, false);

        runtime = new Runtime();
        console = new TelemetryLogger();
    }

    public class Vision {
        private final WebcamName webcam;
        public OpenCvCamera camera;
        private AprilTagDetectionPipeline aprilTagDetectionPipeline;

        static final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        public AprilTagDetection detectedTag = null;

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
            while (!opMode.isStarted() && !opMode.isStopRequested()) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                //Assign AprilTag is tag is detected
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
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(detectedTag);
                    }
                    else {
                        telemetry.addLine("Tag not found");

                        if(detectedTag == null) {
                            telemetry.addLine("(The tag has never been seen)");
                        }
                        else {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(detectedTag);
                        }
                    }

                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(detectedTag == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(detectedTag);
                    }
                }

                telemetry.update();
                opMode.sleep(20);
            }
        }

        //Calculate distance to tag
        private @SuppressLint("DefaultLocale")
        void tagToTelemetry(AprilTagDetection detection) {
            telemetry.addLine("\nDetected Tag=" + AprilTag.getTag(detection.id));
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }
    }

    public class MecanumDrive {
        public DcMotor leftFront;
        public DcMotor rightFront;
        public DcMotor leftBack;
        public DcMotor rightBack;

        private int maxPower = 1;

        //Drivetrain String parameters
        public MecanumDrive(String leftFront, String rightFront, String leftBack, String rightBack) {
            this(hardwareMap.get(DcMotor.class, leftFront), hardwareMap.get(DcMotor.class, rightFront), hardwareMap.get(DcMotor.class, leftBack), hardwareMap.get(DcMotor.class, rightBack));
        }

        //Drivetrain DcMotor parameters
        public MecanumDrive(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
            this.leftFront = leftFront;
            this.rightFront = rightFront;
            this.leftBack = leftBack;
            this.rightBack = rightBack;
        }

        //DriveTrain Motor Direction boolean parameters
        public void setMotorDirection(boolean leftFront, boolean rightFront, boolean leftBack, boolean rightBack) {
            setMotorDirection(leftFront ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE,
                    rightFront ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE,
                    leftBack ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE,
                    rightBack ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        }

        //DriveTrain Motor Direction DcMotor parameters
        public void setMotorDirection(DcMotorSimple.Direction leftFront, DcMotorSimple.Direction rightFront, DcMotorSimple.Direction leftBack, DcMotorSimple.Direction rightBack) {
            this.leftFront.setDirection(leftFront);
            this.rightFront.setDirection(rightFront);
            this.leftBack.setDirection(leftBack);
            this.rightBack.setDirection(rightBack);
        }

        //Zero Power Brakes boolean parameter
        public void setBrakeMode(boolean brakes) {
            setBrakeMode(brakes ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //Zero Power Brakes DcMotor parameter
        public void setBrakeMode(DcMotor.ZeroPowerBehavior brakes) {
            leftFront.setZeroPowerBehavior(brakes);
            rightFront.setZeroPowerBehavior(brakes);
            leftBack.setZeroPowerBehavior(brakes);
            rightBack.setZeroPowerBehavior(brakes);
        }

        //Set power of DriveTrain motors
        public void setPower(double leftFront, double rightFront, double leftBack, double rightBack) {
            this.leftFront.setPower(Range.clip(leftFront, -maxPower, maxPower));
            this.rightFront.setPower(Range.clip(rightFront, -maxPower, maxPower));
            this.leftBack.setPower(Range.clip(leftBack, -maxPower, maxPower));
            this.rightBack.setPower(Range.clip(rightBack, -maxPower, maxPower));
        }

        //Max power setter
        public void setMaxPower(int maxPower) {
            this.maxPower = Range.clip(maxPower, -1, 1);
        }

        //Max power getter
        public int getMaxPower() {
            return maxPower;
        }

        //Movement calculations
        public void move(double fwdBackPower, double strafePower, double turnPower) {
            setPower(fwdBackPower + turnPower + strafePower,
                    fwdBackPower - turnPower - strafePower,
                    fwdBackPower + turnPower - strafePower,
                    fwdBackPower - turnPower + strafePower);
        }

        //Move in set direction for seconds
        public void moveForSeconds(double leftFront, double rightFront, double leftBack, double rightBack, double seconds) {

        }

        //Move to specified position
        public void moveToPosition() {

        }

        //Stop all DriveTrain motors
        public void stop() {
            setPower(0, 0, 0, 0);
        }

        //Rotate encoders to zero position
        void setEncoderMode(DcMotor.RunMode encoderMode) {
            if(encoderMode == DcMotor.RunMode.RUN_TO_POSITION) resetEncoders();

            leftFront.setMode(encoderMode);
            rightFront.setMode(encoderMode);
            leftBack.setMode(encoderMode);
            rightBack.setMode(encoderMode);
        }

        //Reset encoders to zero at current position
        void resetEncoders() {
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public class Intake {
        public DcMotor left;
        public DcMotor right;

        public final double INTAKE_SPEED = 0.2;

        //Intake String parameters
        public Intake(String left, String right) {
            this(hardwareMap.get(DcMotor.class, left), hardwareMap.get(DcMotor.class, right));
        }

        //Intake DcMotor parameters
        public Intake(DcMotor left, DcMotor right) {
            this.left = left;
            this.right = right;
        }

        //Intake Motor Direction boolean parameters
        public void setMotorDirection(boolean left, boolean right) {
            setMotorDirection(left ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE, right ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        }

        //Intake Motor Direction DcMotor parameters
        public void setMotorDirection(DcMotorSimple.Direction left, DcMotorSimple.Direction right) {
            this.left.setDirection(left);
            this.right.setDirection(right);
        }

        //Intake - In
        public void in() {
            left.setPower(INTAKE_SPEED);
            right.setPower(INTAKE_SPEED);
        }

        //Intake - Out
        public void out() {
            left.setPower(-INTAKE_SPEED);
            right.setPower(-INTAKE_SPEED);
        }

        //Intake - Stop
        public void stop() {
            left.setPower(0);
            right.setPower(0);
        }
    }

    //Runtime
    private static class Runtime {
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
            } catch (InterruptedException ignored) {
            }
        }

        public int getTime() {
            return (int) runtime.milliseconds();
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
}