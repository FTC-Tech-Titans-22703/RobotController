package org.firstinspires.ftc.teamcode.util.Robot;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.Robot.Types.AprilTag;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Vision extends Subsystem{
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

    public Vision(String webcam, Robot robot) {
        this.robot = robot;
        this.webcam = robot.hardwareMap.get(WebcamName.class, webcam);
    }

    public void initScan() {
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
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

        robot.telemetry.setMsTransmissionInterval(50);
    }

    public void waitForStartAndScan() {
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while(!robot.opMode.isStarted() && !robot.opMode.isStopRequested()) {
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
                    robot.telemetry.addLine("Tag Detected:");
                    tagToTelemetry(detectedTag);
                }
                else {
                    robot.telemetry.addLine("Tag NOT Detected");

                    if(detectedTag == null) {
                        robot.telemetry.addLine("(Tag has never been detected)");
                    } else {
                        robot.telemetry.addLine("Previous Tag:");
                        tagToTelemetry(detectedTag);
                    }
                }

            }
            else {
                robot.telemetry.addLine("Tag NOT Detected");

                if(detectedTag == null) {
                    robot.telemetry.addLine("(Tag has never been detected)");
                }
                else {
                    robot.telemetry.addLine("Previous Tag:");
                    tagToTelemetry(detectedTag);
                }
            }

            robot.telemetry.update();
            robot.opMode.sleep(20);
        }

        robot.telemetry.clearAll();
        camera.stopStreaming();
    }

    private @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        robot.telemetry.addLine("\nDetected Tag = " + getDetectedTag() + " (id: " + detection.id + ")");
        /*
        robot.telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        robot.telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        robot.telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        robot.telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        robot.telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        robot.telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        */
    }

    public AprilTag getDetectedTag() {
        if(detectedTag == null) return AprilTag.NOT_FOUND;
        if(detectedTag.id == -1) return AprilTag.NOT_FOUND;
        if(detectedTag.id == 1) return AprilTag.LEFT;
        if(detectedTag.id == 2) return AprilTag.MIDDLE;
        return AprilTag.RIGHT;
    }
}
