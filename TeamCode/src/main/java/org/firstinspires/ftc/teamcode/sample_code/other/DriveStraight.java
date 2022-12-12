
package org.firstinspires.ftc.teamcode.sample_code.other;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "DriveStraight", group = "")
public class DriveStraight extends LinearOpMode {

  private DcMotor lf;
  private DcMotor rf;
  private DcMotor lb;
  private DcMotor rb;
  private BNO055IMU imuAsBNO055IMU;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
	rf = hardwareMap.get(DcMotor.class, "rf");
	rb = hardwareMap.get(DcMotor.class, "rb");
	lf = hardwareMap.get(DcMotor.class, "lf");
	lb = hardwareMap.get(DcMotor.class, "lb");
	imuAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imu");

	// This op mode uses the REV Hub's built-in gyro to
	// to allow a robot to move straight forward and then
	// make a right turn.
	// The op mode assume you have
	// (1) Connected two motors to the expansion
	//	   hub.
	// (2) Created a config file that
	// (a) names the motors lf, rf, lb, rb
	// (b) configures the imu on I2C bus 0 port 0
	//	  as a REV Expansion Hub IMU
	//	  with the name "imu".
	// Setup so motors will brake the wheels
	// when motor power is set to zero.
	rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	// Reverse direction of one motor so robot moves
	// forward rather than spinning in place.
	lf.setDirection(DcMotorSimple.Direction.REVERSE);
	lb.setDirection(DcMotorSimple.Direction.REVERSE);
	// Create an IMU parameters object.
	BNO055IMU.Parameters IMU_Parameters = new BNO055IMU.Parameters();
	// Set the IMU sensor mode to IMU. This mode uses
	// the IMU gyroscope and accelerometer to
	// calculate the relative orientation of hub and
	// therefore the robot.
	IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
	// Intialize the IMU using parameters object.
	imuAsBNO055IMU.initialize(IMU_Parameters);
	// Report the initialization to the Driver Station.
	telemetry.addData("Status", "IMU initialized, calibration started.");
	telemetry.update();
	// Wait one second to ensure the IMU is ready.
	sleep(1000);
	// Loop until IMU has been calibrated.
	while (!IMU_Calibrated()) {
	  telemetry.addData("If calibration ", "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
	  telemetry.update();
	  // Wait one second before checking calibration
	  // status again.
	  sleep(1000);
	}
	// Report calibration complete to Driver Station.
	telemetry.addData("Status", "Calibration Complete");
	telemetry.addData("Action needed:", "Please press the start triangle");
	telemetry.update();
	// Wait for Start to be pressed on Driver Station.
	waitForStart();
	// Create a timer object with millisecond
	// resolution and save in ElapsedTime variable.
	ElapsedTime ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	// Initialize motor power variables to 30%.
	
	double Base_Power =0.0;
	double Front_Power = Base_Power;
	double Back_Power = Base_Power;
	// Set motor powers to the variable values.
	//lf.setPower(-Front_Power);
	//rf.setPower(Front_Power);
	//rb.setPower(-Back_Power);
	//lb.setPower(Back_Power);
	ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

	while (!(ElapsedTime2.milliseconds() >= 2000|| isStopRequested())) {
	  // Save gyro's yaw angle

	  float Yaw_Angle = imuAsBNO055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
	  // Report yaw orientation to Driver Station.
	  telemetry.addData("Yaw angle", Yaw_Angle);
	  // If the robot is moving straight ahead the
	  // yaw value will be close to zero. If it's not, we
	  // need to adjust the motor powers to adjust heading.
	  // If robot yaws right or left by 5 or more,
	  // adjust motor power variables to compensation.
	  if (Yaw_Angle < -0.05) {
		// Front correction
		Base_Power=0.5;
		Front_Power = Base_Power - 0.05;
		Back_Power = Base_Power + 0.05;
	  } else if (Yaw_Angle > .05) {
		// Back correction.
		Base_Power=0.5;
		Front_Power = Base_Power + 0.05;
		Back_Power = Base_Power - 0.05;
	  } else {
		// Continue straight
		Base_Power=0.5;
		Front_Power = Base_Power;
		Back_Power = Base_Power;
	  }
	  // Report the new power levels to the Driver Station.
	  //telemetry.addData("Front Motors Power", Front_Power);
	  //telemetry.addData("Back Motors Power", Back_Power);
	  // Update the motors to the new power levels.
	  lf.setPower(-Front_Power);
	  rf.setPower(Front_Power);
	  rb.setPower(-Back_Power);
	  lb.setPower(Back_Power);
	  telemetry.update();
	  // Wait 1/5 second before checking again.
	  sleep(50);
	}

	// We're done. Turn off motors
	lf.setPower(0);
	rf.setPower(0);
	lb.setPower(0);
	rb.setPower(0);
	// Pause so final telemetry is displayed.
	sleep(100);
  }

  /**
   * Function that becomes true when gyro is calibrated and
   * reports calibration status to Driver Station in the meantime.
   */
  private boolean IMU_Calibrated() {
	telemetry.addData("IMU Calibration Status", imuAsBNO055IMU.getCalibrationStatus());
	telemetry.addData("Gyro Calibrated", imuAsBNO055IMU.isGyroCalibrated() ? "True" : "False");
	telemetry.addData("System Status", imuAsBNO055IMU.getSystemStatus().toString());
	return imuAsBNO055IMU.isGyroCalibrated();
  }
}
