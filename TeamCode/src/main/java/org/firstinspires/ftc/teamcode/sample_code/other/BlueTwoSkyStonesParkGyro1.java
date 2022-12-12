

package org.firstinspires.ftc.teamcode.sample_code.other;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import android.app.Activity;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// this is the gyro library
import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "BlueTwoSkyStonesParkGyro1", group = "")
//@Disabled
public class BlueTwoSkyStonesParkGyro1 extends LinearOpMode {
  /** The colorSensor field will contain a reference to our color sensor hardware object */
  NormalizedColorSensor colorSensor;
	private DcMotor rf;
	private DcMotor lb;
	private DcMotor rb;
	private DcMotor lf;
	private DcMotor rightspinny;
	private DcMotor leftspinny;
   // private DcMotor clawArm;
	private CRServo servoArm;
	private BNO055IMU imuAsBNO055IMU;
	private CRServo ls;
	private DistanceSensor sensorRange;
	private CRServo rs;
 /** The relativeLayout field is used to aid in providing interesting visual feedback
   * in this sample application; you probably *don't* need something analogous when you
   * use a color sensor on your robot */
  View relativeLayout;
	 private CRServo ServoArmClamp;
private CRServo LeftServoArm;
private CRServo leftServoArmClamp;
	private void teleLog(String message) {
		telemetry.addData(">", message);
		telemetry.update();
	}

	private void initHardwareMembers() {
	  rf  = hardwareMap.get(DcMotor.class, "rf");
	  rb  = hardwareMap.get(DcMotor.class, "rb");
	  lf  = hardwareMap.get(DcMotor.class, "lf");
	  lb = hardwareMap.get(DcMotor.class, "lb");
	  imuAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imu");
	  ServoArmClamp= hardwareMap.get(CRServo.class, "ServoArmClamp");
	  sensorRange = hardwareMap.get(DistanceSensor.class, "sensorRange");
	  rs = hardwareMap.get(CRServo.class, "rs");
	  // Setup so motors will brake the wheels
	// when motor power is set to zero.
	rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

	  rf.setDirection(DcMotor.Direction.REVERSE);
	  lf.setDirection(DcMotor.Direction.FORWARD);
	  rb.setDirection(DcMotor.Direction.REVERSE);
	  lb.setDirection(DcMotor.Direction.FORWARD);

	  servoArm = hardwareMap.get(CRServo.class, "servoArm");
			ls = hardwareMap.get(CRServo.class, "ls");
LeftServoArm = hardwareMap.get(CRServo.class,"LeftServoArm");
				leftServoArmClamp = hardwareMap.get(CRServo.class,"leftServoArmClamp");
	 // clawArm = hardwareMap.get(DcMotor.class, "Arm");
	  rightspinny  = hardwareMap.get(DcMotor.class, "rightspinny");
	  leftspinny  = hardwareMap.get(DcMotor.class, "leftspinny");

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
	sleep(100);
	// Loop until IMU has been calibrated.
	while (!IMU_Calibrated()) {
	  telemetry.addData("If calibration ", "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
	  telemetry.update();
	  // Wait one second before checking calibration
	  // status again.
	  sleep(100);
	}
	// Report calibration complete to Driver Station.
	telemetry.addData("Status", "Calibration Complete");
	telemetry.addData("Action needed:", "Please press the start triangle");
	telemetry.update();
	}
	
	
//Method to drive Robot straight using Gyro
	private void DriveRoboStraight(double Base_Power, double GyroRunTime) {
	double Left_Power = Base_Power;
	double Right_Power = Base_Power;
	// Set motor powers to the variable values.
	lf.setPower(Left_Power);
	rf.setPower(Right_Power);
	lb.setPower(Left_Power);
	rb.setPower(Right_Power);
  ElapsedTime ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	
  while (!(ElapsedTime2.milliseconds() >= GyroRunTime|| isStopRequested())) {
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
		// Turn left
		Left_Power = Base_Power - 0.05;
		Right_Power = Base_Power + 0.05;
	  } else if (Yaw_Angle >0.05) {
		// Turn right.
		Left_Power = Base_Power + 0.05;
		Right_Power = Base_Power - 0.05;
	  } else {
		// Continue straight
		Left_Power = Base_Power;
		Right_Power = Base_Power;
	  }
	  // Report the new power levels to the Driver Station.
	  telemetry.addData("Left Motor Power", Left_Power);
	  telemetry.addData("Right Motor Power", Right_Power);
	  // Update the motors to the new power levels.
	  lf.setPower(Left_Power);
	  rf.setPower(Right_Power);
	  lb.setPower(Left_Power);
	  rb.setPower(Right_Power);
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
	//sleep(100);
  }
	
	private void strafeRight(double power, double msTime) {
		ElapsedTime runtime = new ElapsedTime();
		runtime.reset();

		while (runtime.milliseconds() < msTime) {
			lf.setPower(power + 0.05);
			rf.setPower(-power - 0.02);
			rb.setPower(power);
			lb.setPower(-power - 0.02);
		}
		stopRobo();
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

	private void strafeLeft(double power, double msTime) {
		ElapsedTime runtime = new ElapsedTime();
		runtime.reset();

		while (runtime.milliseconds() < msTime) {
			lf.setPower(-power);
			rf.setPower(power);
			rb.setPower(-power);
			lb.setPower(power);
		}
		stopRobo();
	}
	
	private void strafeRightStraight(double power, double msTime) {
		ElapsedTime runtime = new ElapsedTime();
		runtime.reset();
		while (runtime.milliseconds() < 500) {
			lf.setPower(power);
			rf.setPower(-power);
			rb.setPower(power);
			lb.setPower(-power);
		}
		strafeRight(power, msTime - 500);
	}
	
	private void strafeLeftStraight(double power, double msTime) {
		ElapsedTime runtime = new ElapsedTime();
		runtime.reset();
		while (runtime.milliseconds() < 500) {
			lf.setPower(-power);
			rf.setPower(power);
			rb.setPower(-power);
			lb.setPower(power);
		}
		strafeLeft(power, msTime - 500);
	}
	
	private void strafeForward(double power, double msTime) {
		ElapsedTime runtime = new ElapsedTime();
		runtime.reset();

		while (runtime.milliseconds() < msTime) {
			lf.setPower(power);
			rf.setPower(power);
			rb.setPower(power);
			lb.setPower(power);
		}
		stopRobo();
	}

	private void strafeBackward(double power, double msTime) {
		ElapsedTime runtime = new ElapsedTime();
		runtime.reset();

		while (runtime.milliseconds() < msTime) {
			lf.setPower(-power);
			rf.setPower(-power);
			rb.setPower(-power);
			lb.setPower(-power);
		}
		stopRobo();
	}

	private void rotateCCW(double power, double msTime) {
		ElapsedTime runtime = new ElapsedTime();
		runtime.reset();

		while (runtime.milliseconds() < msTime) {
			lf.setPower(-power);
			rb.setPower(power);
			lb.setPower(-power);
			rf.setPower(power);
		}
		stopRobo();
	}
	
	private void stopRobo() {
		lf.setPower(0);
		rf.setPower(0);
		rb.setPower(0);
		lb.setPower(0);
	}

	private void servoArmDown(double power, double msTime) {
	  ElapsedTime runtime = new ElapsedTime();
	  runtime.reset();

	  while (runtime.milliseconds() < msTime) {
		servoArm.setPower(power);
	  }
	}

	private void servoArmUp(double power, double msTime) {
	  ElapsedTime runtime = new ElapsedTime();
	  runtime.reset();

	  while (runtime.milliseconds() < msTime) {
		servoArm.setPower(-power);
	  }
	}
	
	 private void ServoArmClamp(double power, double msTime) {
	  ElapsedTime runtime = new ElapsedTime();
	  runtime.reset();

	  while (runtime.milliseconds() < msTime) {
		ServoArmClamp.setPower(power);
	  }
	}

	@Override
	public void runOpMode() {
	  // Get a reference to the RelativeLayout so we can later change the background
	// color of the Robot Controller app to match the hue detected by the RGB sensor.
	int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
	relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
try {
	  runSample(); // actually execute the sample
	} finally {
	  // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
	  // as pure white, but it's too much work to dFig out what actually was used, and this is good
	  // enough to at least make the screen reasonable again.
	  // Set the panel back to the default color
	  relativeLayout.post(new Runnable() {
		public void run() {
		  relativeLayout.setBackgroundColor(Color.WHITE);
		}
	  });
	  }
		  }
protected void runSample() {
  /** Wait for the game to begin */
		teleLog("Press Play to start op mode");
	
		initHardwareMembers();
		teleLog("Initialized Multi Move 23");
		// values is a reference to the hsvValues array.
	float[] hsvValues = new float[3];
	final float values[] = hsvValues;

	// bPrevState and bCurrState keep track of the previous and current state of the button
	boolean bPrevState = false;
	boolean bCurrState = false;

	// Get a reference to our sensor object.
	colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
	  rf  = hardwareMap.get(DcMotor.class, "rf");
	  rb  = hardwareMap.get(DcMotor.class, "rb");
	  lf  = hardwareMap.get(DcMotor.class, "lf");
	  lb = hardwareMap.get(DcMotor.class, "lb");
	  ServoArmClamp= hardwareMap.get(CRServo.class, "ServoArmClamp");
	 
	  rf.setDirection(DcMotor.Direction.REVERSE);
	  lf.setDirection(DcMotor.Direction.FORWARD);
	  rb.setDirection(DcMotor.Direction.REVERSE);
	  lb.setDirection(DcMotor.Direction.FORWARD);

	  servoArm = hardwareMap.get(CRServo.class, "servoArm");
	 // clawArm = hardwareMap.get(DcMotor.class, "Arm");
	  
	// If possible, turn the light on in the beginning (it might already be on anyway,
	// we just make sure it is if we can).
	if (colorSensor instanceof SwitchableLight) {
	  ((SwitchableLight)colorSensor).enableLight(true);
	}

	// Wait for the start button to be pressed.
	waitForStart();

  // Distance sensor
	boolean d = false;
	while (d == false) {
	double sensor_range;
			sensor_range = sensorRange.getDistance(DistanceUnit.CM);
	d= getCloser(sensor_range);
	}
	
		// Check the status of the x button on the gamepad
	  bCurrState = gamepad1.x;

	  // If the button state is different than what it was, then act
	  if (bCurrState != bPrevState) {
		// If the button is (now) down, then toggle the light
		if (bCurrState) {
		  if (colorSensor instanceof SwitchableLight) {
			SwitchableLight light = (SwitchableLight)colorSensor;
			light.enableLight(!light.isLightOn());
		  }
		}
	  }
	  bPrevState = bCurrState;
	// Read the sensor
	   boolean p = false;
	while (p == false) {
	  NormalizedRGBA colors = colorSensor.getNormalizedColors();
			Color.colorToHSV(colors.toColor(), hsvValues);
	  telemetry.addLine()
			  .addData("H", "%.3f", hsvValues[0])
			  .addData("S", "%.3f", hsvValues[0])
			  .addData("V", "%.3f", hsvValues[0]);
	  telemetry.addLine()
			  .addData("a", "%.3f", colors.alpha)
			  .addData("r", "%.3f", colors.red)
			  .addData("g", "%.3f", colors.green)
			  .addData("b", "%.3f", colors.blue);

	  /** We also display a conversion of the colors to an equivalent Android color integer.
	   * @see Color */
	  int color = colors.toColor();
	  telemetry.addLine("raw Android color: ")
			  .addData("a", "%02x", Color.alpha(color))
			  .addData("r", "%02x", Color.red(color))
			  .addData("g", "%02x", Color.green(color))
			  .addData("b", "%02x", Color.blue(color));

	  // Balance the colors. The values returned by getColors() are normalized relative to the
	  // maximum possible values that the sensor can measure. For example, a sensor might in a
	  // particular configuration be able to internally measure color intensity in a range of
	  // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
	  // so as to return a value it the range [0,1]. However, and this is the point, even so, the
	  // values we see here may not get close to 1.0 in, e.g., low light conditions where the
	  // sensor measurements don't approach their maximum limit. In such situations, the *relative*
	  // intensities of the colors are likely what is most interesting. Here, for example, we boost
	  // the signal on the colors while maintaining their relative balance so as to give more
	  // vibrant visual feedback on the robot controller visual display.
	  float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
	  colors.red   /= max;
	  colors.green /= max;
	  colors.blue  /= max;
	  color = colors.toColor();

	  telemetry.addLine("normalized color:  ")
			  .addData("a", "%02x", Color.alpha(color))
			  .addData("r", "%02x", Color.red(color))
			  .addData("g", "%02x", Color.green(color))
			  .addData("b", "%02x", Color.blue(color));
	  telemetry.update();

	  // convert the RGB values to HSV values.
	  Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

	p = firstskystone(hsvValues[0]);
boolean success = false;
	if(p){
	  success = picktheblock();
	}
	}
	telemetry.addLine("SECOND");
	telemetry.update();
	//second distance sensor
	 boolean f = false;
	while (f == false) {
		leftServoArmClamp.setPower(0.9);
	double sensor_range;
			sensor_range = sensorRange.getDistance(DistanceUnit.CM);
	f = getCloser(sensor_range);
	
	}
	//second color sensor
		boolean t = false;
	while (t == false) {
	  NormalizedRGBA colors = colorSensor.getNormalizedColors();
			Color.colorToHSV(colors.toColor(), hsvValues);
	  telemetry.addLine()
			  .addData("H", "%.3f", hsvValues[0])
			  .addData("S", "%.3f", hsvValues[0])
			  .addData("V", "%.3f", hsvValues[0]);
	  telemetry.addLine()
			  .addData("a", "%.3f", colors.alpha)
			  .addData("r", "%.3f", colors.red)
			  .addData("g", "%.3f", colors.green)
			  .addData("b", "%.3f", colors.blue);

	  /** We also display a conversion of the colors to an equivalent Android color integer.
	   * @see Color */
	  int color = colors.toColor();
	  telemetry.addLine("raw Android color: ")
			  .addData("a", "%02x", Color.alpha(color))
			  .addData("r", "%02x", Color.red(color))
			  .addData("g", "%02x", Color.green(color))
			  .addData("b", "%02x", Color.blue(color));

	  // Balance the colors. The values returned by getColors() are normalized relative to the
	  // maximum possible values that the sensor can measure. For example, a sensor might in a
	  // particular configuration be able to internally measure color intensity in a range of
	  // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
	  // so as to return a value it the range [0,1]. However, and this is the point, even so, the
	  // values we see here may not get close to 1.0 in, e.g., low light conditions where the
	  // sensor measurements don't approach their maximum limit. In such situations, the *relative*
	  // intensities of the colors are likely what is most interesting. Here, for example, we boost
	  // the signal on the colors while maintaining their relative balance so as to give more
	  // vibrant visual feedback on the robot controller visual display.
	  float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
	  colors.red   /= max;
	  colors.green /= max;
	  colors.blue  /= max;
	  color = colors.toColor();

	  telemetry.addLine("normalized color:  ")
			  .addData("a", "%02x", Color.alpha(color))
			  .addData("r", "%02x", Color.red(color))
			  .addData("g", "%02x", Color.green(color))
			  .addData("b", "%02x", Color.blue(color));
	  telemetry.update();

	  // convert the RGB values to HSV values.
	  Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

	 t = firstskystone2(hsvValues[0]);
	if(t){
		picktheblock2();
	}
}
}

protected boolean getCloser(double sensor_range){
	leftServoArmClamp.setPower(0.9);
				LeftServoArm.setPower(0.7);
			if(sensor_range > 9) {
				rf.setPower(-0.315);
				lf.setPower(0.3);
				rb.setPower(0.3);
				lb.setPower(-0.3);
		return false;
			} else if (sensor_range < 8) {
			 rf.setPower(0);
				lf.setPower(0);
				rb.setPower(0);
				lb.setPower(0);
				sleep(100);
		return true;
			}
	return false;
}

protected boolean firstskystone(float p) {
leftServoArmClamp.setPower(0.9);
				LeftServoArm.setPower(0.7);

	 if(p < 90) {
	rf.setPower(-0.2);
	lf.setPower(-0.2);
	rb.setPower(-0.2);
	lb.setPower(-0.2);
	return false;
	} else if(p > 100){
	  rf.setPower(0);
	  lf.setPower(0);
	  rb.setPower(0);
	  lb.setPower(0);
	  return true;
	} 
	return false;
 }
 
 protected boolean firstskystone2(float p) {
leftServoArmClamp.setPower(0.9);
				LeftServoArm.setPower(0.7);
	 if(p < 90) {
	rf.setPower(-0.2);
	lf.setPower(-0.203);
	rb.setPower(-0.2);
	lb.setPower(-0.203);
	return false;
	} else if(p > 100){
	  rf.setPower(0);
	  lf.setPower(0);
	  rb.setPower(0);
	  lb.setPower(0);
	  sleep(200);
	  rotateCCW(-0.2, 50);
	  return true;
	} 
	return false;
 }
 
 protected boolean picktheblock() {
	strafeBackward(0.2, 110);
	LeftServoArm.setPower(0.7);
	leftServoArmClamp.setPower(1);
	rotateCCW(-0.2, 20);
	strafeLeft(0.2, 90);
	ServoArmClamp(0.5, 300);
	rotateCCW(-0.2, 15);
	//strafeRight(0.3, 200);
	servoArmDown(0.18, 800);
	strafeRight(0.3, 200);
	strafeRight(0, 100);
	ServoArmClamp(-0.4, 600);
	strafeRight(0, 50);
	servoArmUp(0.9,500);
	strafeLeft(0.3, 500);
	//rotateCCW(-0.2, 10);
   //try the drive straight code instead of strafeforward
	strafeForward(0.8, 1400); 
	servoArmDown(0.18, 1000);
	ServoArmClamp(0.5, 300);
	servoArmUp(0.9,500);
	//strafeLeft(0.3, 700);
	ServoArmClamp(-0.4, 300);
	strafeLeft(0.3, 20);
	strafeBackward(0.8, 1800); 
	strafeRight(0.3, 300);
		rotateCCW(-0.2, 120);
  // rotateC
  // strafeBackward(1, 1800);
 
  
 return true;
   
 }
 protected void picktheblock2() {
				LeftServoArm.setPower(0.7);
leftServoArmClamp.setPower(0.9);
	 //strafeLeft(0.2, 100);
	 //rotateCCW(0.3, 50);
	  ServoArmClamp(0.5, 300);
	  //strafeLeft(0.2, 150);
   //strafeBackward(0.1, 250);
   strafeBackward(0.2, 150); 
	// strafeRight(0.2, 50);
	strafeLeft(0.2, 120);
  servoArmDown(0.18, 500);
   strafeRight(0, 500);
 strafeRight(0.3, 600);
   ServoArmClamp(-0.4, 300);
   servoArmUp(0.9,500);
  strafeLeft(0.34, 700);
  //rotateCCW(0.2, 280);
   //strafeForward(1, 1500);
   //strafeForward(0, 300);
   strafeForward(0.8, 1850); 
//   DriveRoboStraight(0, 150); 
  
	servoArmDown(0.18, 500);
	  ServoArmClamp(0.5, 300);
   servoArmUp(0.9,500);
	  ServoArmClamp(-0.4, 300);
   //strafeBackward(0.6, 950);
   DriveRoboStraight(-0.6, 780); 
	 /*strafeRight(0.3, 1300);
   servoArmDown(0.18, 500);
	  ServoArmClamp(0.5, 300);
   servoArmUp(0.9,500);
   strafeLeft(0.3, 100);
   strafeForward(0.8, 800);
   strafeForward(0.4, 200);
ServoArmClamp(-0.4, 300);
 rotateCCW(0.5, 600);
 strafeBackward(0,100);
  strafeBackward(0.8, 300);
  ls.setPower(1);
  rs.setPower(1);
  sleep(300);
  strafeForward(1, 2000);
  ls.setPower(0);
  rs.setPower(0);
  sleep(300);
  rotateCCW(0.5, 500);
  strafeForward(0.8, 1200);
  */
 }
}
