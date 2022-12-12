

package org.firstinspires.ftc.teamcode.sample_code.other;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name = "Blue: SkyStonesAndPlatform", group = "")
//@Disabled
public class SkystonesAndPlatformBlue extends LinearOpMode {
	private DcMotor rf;
	private DcMotor lb;
	private DcMotor rb;
	private DcMotor lf;
	private DcMotor rightspinny;
	private DcMotor leftspinny;
	private DcMotor clawArm;
	private CRServo servoArm;
 private CRServo ls;
 private CRServo ServoArmClamp;
	private CRServo rs;
	private void teleLog(String message) {
		telemetry.addData(">", message);
		telemetry.update();
	}

	private void initHardwareMembers() {
	  rf  = hardwareMap.get(DcMotor.class, "rf");
	  rb  = hardwareMap.get(DcMotor.class, "rb");
	  lf  = hardwareMap.get(DcMotor.class, "lf");
	  lb = hardwareMap.get(DcMotor.class, "lb");
ls = hardwareMap.get(CRServo.class,"ls");
		rs = hardwareMap.get(CRServo.class,"rs");
		ServoArmClamp = hardwareMap.get(CRServo.class,"ServoArmClamp");


	  rf.setDirection(DcMotor.Direction.REVERSE);
	  lf.setDirection(DcMotor.Direction.FORWARD);
	  rb.setDirection(DcMotor.Direction.REVERSE);
	  lb.setDirection(DcMotor.Direction.FORWARD);

	  servoArm = hardwareMap.get(CRServo.class, "servoArm");
   //   clawArm = hardwareMap.get(DcMotor.class, "Arm");
	  rightspinny  = hardwareMap.get(DcMotor.class, "rightspinny");
	  leftspinny  = hardwareMap.get(DcMotor.class, "leftspinny");
	}
	
	
	
	private void strafeRight(double power, double msTime) {
		ElapsedTime runtime = new ElapsedTime();
		runtime.reset();

		while (runtime.milliseconds() < msTime) {
			lf.setPower(power);
			rf.setPower(-power);
			rb.setPower(power);
			lb.setPower(-power);

		}
		stopRobo();
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
			lf.setPower(0.5);
			rf.setPower(-0.5);
			rb.setPower(0.5);
			lb.setPower(-0.5);
		}
		strafeRight(power, msTime - 500);
	}
	
	private void strafeLeftStraight(double power, double msTime) {
		ElapsedTime runtime = new ElapsedTime();
		runtime.reset();
		while (runtime.milliseconds() < 500) {
			lf.setPower(-0.5);
			rf.setPower(0.5);
			rb.setPower(-0.5);
			lb.setPower(0.5);
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

	@Override
	public void runOpMode() {
		/** Wait for the game to begin */
		teleLog("Press Play to start op mode");
	
		initHardwareMembers();
		teleLog("Initialized Multi Move 23");
		
		waitForStart();
	
		// (1) Fetch first skystone
		// Put the claw arm down
		 clawArm.setPower(-0.1); 
		sleep(150);
		
		// Move forward to stone line
		strafeRightStraight(0.5, 1600);
/*	
		// Grab 
		servoArmDown(0.18, 200);
		sleep(1600);
		
		// Come back a bit
		strafeLeftStraight(0.8, 750);
	
		// Strafe to drag the stone to the other side of the bridge
		strafeForward(1.0, 1700);
	
		// Release the skystone
		servoArmUp(0.55, 200);
	
		// (2) Fetch second skystone
		// Come back across bridge
		strafeBackward(1.0, 2200);
		
		// turn to fix robot angle
		//lf.setPower(0.5);
		  //  rf.setPower(-0.5);
			//rb.setPower(-0.5);
			//lb.setPower(0.5);
		//sleep(100);
		
		
		
		// Approach stone line
		strafeRightStraight(0.8, 735);
	
		// Grab 
		servoArmDown(0.18, 200);
		sleep(1000);
		
		// Come back a bit
		strafeLeftStraight(0.8, 880);


			rb.setPower(-0.5);
			lb.setPower(0.5);
			sleep(420);
			
			
		// Strafe to drag the stone to the other side of the bridge
		strafeForward(1.0, 2200);
	
	
			
	// Release the skystone
		servoArmUp(0.55, 200);
		
		rb.setPower(0.5);
			lb.setPower(-0.5);
			sleep(120);
		
		
	// head to platform
			strafeForward(1.0, 1000);

			//turn to face platform
			lf.setPower(-0.5);
			rf.setPower(0.5);
			rb.setPower(0.5);
			lb.setPower(-0.5);
		sleep(1000);
	
	// hit platform
		strafeBackward(0.8, 770);
		
		lf.setPower(0);
			rf.setPower(0);
			rb.setPower(0);
			lb.setPower(0);
			sleep(100);
			
		
		//down servos
		ls.setPower(1);
			 rs.setPower(1);
			 sleep(1000);
			 
			 lf.setPower(0);
			rf.setPower(0);
			rb.setPower(0);
			lb.setPower(0);
			sleep(300);
			
			
			 //go forward to triangle
			 strafeForward(0.8, 2000);
			 
			 //lift servos
			  ls.setPower(0);
			 rs.setPower(0);
			 sleep(100);
			 
			 
			 
			 //strafe right
			 strafeLeftStraight(0.7, 2500);
			 
			
			 
		
			 //go backward
			 strafeBackward(0.8, 1250);
			 
		
			 
			  //park
			 strafeLeftStraight(0.7, 1500);
			 
			 strafeBackward(0.2, 200);
  */  
	}

}
