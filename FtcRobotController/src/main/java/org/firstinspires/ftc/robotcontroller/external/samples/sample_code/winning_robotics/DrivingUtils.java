package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.winning_robotics;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class DrivingUtils{

    private BNO055IMU imu;

    // some of the functions such as sleep and opModeIsActive are only available inside an opMode
    // so putting this work around. You supply the opMode class where you are going to be using
    // DrivingUtils. This does create a circular dependency - the OpMode on DriverUtils and the
    // DriverUtils back on the OpMode but we only have a handful of opmodes and these programs
    // don't live very long- worst case, kill the app and restart it if memory freeing starts
    // becoming an issue.
    private LinearOpMode opMode;
    public DrivingUtils(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void logTelemetry(String key, String data) {
        this.opMode.telemetry.addData(key, data);
        this.opMode.telemetry.update();
    }

    public void initIMU() {
        this.imu = this.opMode.hardwareMap.get(BNO055IMU.class, "imu");
        logTelemetry("Beginning", "Gyro Calibration");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        this.imu.initialize(parameters);

        while(!imu.isGyroCalibrated()) {
            this.opMode.sleep(50);
            logTelemetry("Calibrating now", "");
        }

        logTelemetry("Done", "Calibrating Gyro");
    }

    /*
     * The following turn programs turns the robot with a constant power and does not accomodate for
     * slipping etc. On the other hand, it is much simpler easier to understand.
     * NOTE: The sample robot for which this code was written uses AndyMark Neverest 40 motors
     * The Neverest 40 motors, when the shaft faces vertically up, turn clockwise on positive power.
     * Thus, the left motor will actually be spinning on the positive power. We are making an
     * assumption that the left motor is set to move forward and the right motor is set to move
     * in reverse during configuration early on. If the assumptions are not held, the turn will
     * not work and you will need to modify the leftMotor and rightMotor setpower below where it says
     * "modify ordering if your motors behave differently" in the code below.
     */
    public void turnDegreesSimple(double angle, double power, DcMotor leftMotor, DcMotor rightMotor)
    {
        double accumulatedTurn = 0;
        double startAngle = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // apply brakes when motor power hits zero to avoid slipping as much as posible
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(this.opMode.opModeIsActive() && Math.abs(accumulatedTurn) <= Math.abs(angle)) {

            accumulatedTurn = startAngle - this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; // calculate the turn amount

            // normalize the turned amount within -180 to +180
            // Note that after doing the following math, you will, surprisingly get the exact
            // magnitude of the actual turn no matter where you start from i.e. even with the weird
            // math below, the magnitude continues on increasing till we are done with the turn.
            // It would be good to note that with the Rev hub sitting parallel to the ground
            // and with the connectors pointing up, left turns will produce a heading of 0-180
            // whereas right turns will produce a heading of 0- negative 180. The 180 point is a
            // cross over mark where we go from positive to negative or vice versa. The weird math
            // below in the if/else fixes it so that in spite of turning left or right, the
            // accumulatedTurn variable keeps track of the total amount turned so far.
            if (accumulatedTurn < -180)
                accumulatedTurn += 360;
            else if (accumulatedTurn > 180)
                accumulatedTurn -= 360;

            if (angle < 0) {
                // turn left - modify ordering if your motors behave differently
                leftMotor.setPower(-power);
                rightMotor.setPower(power);
            } else {
                // turn right - modify ordering if your motors behave differently
                leftMotor.setPower(power);
                rightMotor.setPower(-power);
            }
        }

        // set power to zero to apply brakes
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        this.opMode.sleep(500); // wait for half a second so the robot stops all motion
    }

    public void turnDegreesPController(double angle, PController pController, DcMotor leftMotor, DcMotor rightMotor)
    {
        double accumulatedTurn = 0;
        double startAngle = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;;
        double powerToMotors = 0;

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // reset the setpoint based on the new angle
        pController.setSetPoint(Math.abs(angle));

        // apply brakes when motor power hits zero to avoid slipping as much as posible
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        logTelemetry("turn", "entering pid loop");
        do {
            logTelemetry("turn", "inside pid loop");
            accumulatedTurn = startAngle - this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; // calculate the turn amount

            // normalize the turned amount within -180 to +180
            // Note that after doing the following math, you will, surprisingly get the exact
            // magnitude of the actual turn no matter where you start from i.e. even with the weird
            // math below, the magnitude continues on increasing till we are done with the turn.
            // It would be good to note that with the Rev hub sitting parallel to the ground
            // and with the connectors pointing up, left turns will produce a heading of 0-180
            // whereas right turns will produce a heading of 0- negative 180. The 180 point is a
            // cross over mark where we go from positive to negative or vice versa. The weird math
            // below in the if/else fixes it so that in spite of turning left or right, the
            // accumulatedTurn variable keeps track of the total amount turned so far.
            if (accumulatedTurn < -180)
                accumulatedTurn += 360;
            else if (accumulatedTurn > 180)
                accumulatedTurn -= 360;

            // user the PController to figure out how far we are from the setPoint and compute
            // the power required for the motors at this instant. Note that the pController
            // is an input to this function and should already have the setPoint, min and max Inputs
            // as well as the min and maxOutputs along with the threshold Percent defined.
            // the following line makes assumption that all of that is already set.
            powerToMotors = pController.minOutput + pController.getComputedOutput(accumulatedTurn);
            logTelemetry("turn", Double.toString(powerToMotors));
            if (angle < 0) {
                // turn left - modify ordering if your motors behave differently
                leftMotor.setPower(-powerToMotors);
                rightMotor.setPower(powerToMotors);
            } else {
                // turn right - modify ordering if your motors behave differently
                leftMotor.setPower(powerToMotors);
                rightMotor.setPower(-powerToMotors);
            }
        }while(this.opMode.opModeIsActive() && !pController.hasPControllerReachedTarget());

        // set power to zero to apply brakes
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        this.opMode.sleep(500); // wait for half a second so the robot stops all motion
    }

    public void RobotMoveEncoderPositions(int positions, double power, DcMotor leftMotor, DcMotor rightMotor) {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(positions);
        rightMotor.setTargetPosition(positions);

        while (this.opMode.opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {


            this.opMode.telemetry.addData("1 encoder left", leftMotor.getCurrentPosition());
            this.opMode.telemetry.addData("2 encoder right", rightMotor.getCurrentPosition());
            //telemetry.addData("3 correction", correction);
            //telemetry.addData("left position", leftMotor.getCurrentPosition());
            //telemetry.addData("right position", rightMotor.getCurrentPosition());

            this.opMode.telemetry.update();

            // set power levels.
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

}
