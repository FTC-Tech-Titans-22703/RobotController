/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *javascript:void(0);
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.other;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;




/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MovingMexhanumWheels", group="Linear Opmode")

public class MovingMexhanumWheels extends LinearOpMode {

	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	private DcMotor rf = null;
	private DcMotor lb = null;
	private DcMotor rb = null;
	private DcMotor lf = null;
	private DcMotor rightspinny = null;
	private DcMotor leftspinny = null;
	private CRServo ls;
	private CRServo rs;
   // private CRServo BottomServo;
   // private CRServo MiddleServo;
  //  private CRServo preload;
	private CRServo servoArm;
		private CRServo topServo;
	private CRServo bottomServo;
   // private DcMotor Arm = null;
		private CRServo ServoArmClamp;
//float leftX, rightY, rightX;
//float leftXfast, rightYfast, rightXfast;
private CRServo MiddleLinearSlide;
private CRServo BottomLinearSlide;
private CRServo LeftServoArm;
private CRServo leftServoArmClamp;
private CRServo ClampServo;

   
	
	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		// Initialize the hardware variables. Note that the strings used here as parameters
		// to 'get' must correspond to the names assigned during the robot configuration
		// step (using the FTC Robot Controller app on the phone).
		rf  = hardwareMap.get(DcMotor.class, "rf");
		lb = hardwareMap.get(DcMotor.class, "lb");
		rb  = hardwareMap.get(DcMotor.class, "rb");
		lf  = hardwareMap.get(DcMotor.class, "lf");
		rightspinny  = hardwareMap.get(DcMotor.class, "rightspinny");
		leftspinny  = hardwareMap.get(DcMotor.class, "leftspinny");
		ls = hardwareMap.get(CRServo.class,"ls");
		rs = hardwareMap.get(CRServo.class,"rs");
	 //   Arm  = hardwareMap.get(DcMotor.class, "Arm");
	  //  preload = hardwareMap.get(CRServo.class,"preload");
	 //   Arm  = hardwareMap.get(DcMotor.class, "Arm");
				servoArm = hardwareMap.get(CRServo.class,"servoArm");
				topServo = hardwareMap.get(CRServo.class,"topServo");
//MiddleServo = hardwareMap.get(CRServo.class,"MiddleServo");
			 //   BottomServo = hardwareMap.get(CRServo.class,"BottomServo");
				ServoArmClamp = hardwareMap.get(CRServo.class,"ServoArmClamp");
				MiddleLinearSlide = hardwareMap.get(CRServo.class,"MiddleLinearSlide");
				BottomLinearSlide = hardwareMap.get(CRServo.class,"BottomLinearSlide");
				LeftServoArm = hardwareMap.get(CRServo.class,"LeftServoArm");
				leftServoArmClamp = hardwareMap.get(CRServo.class,"leftServoArmClamp");
				ClampServo = hardwareMap.get(CRServo.class,"ClampServo");

	 
		
	
		// bottomServo = hardwareMap.get(CRServo.class,"bottomServo");


		

		
		
		// Most robots need the motor on one side to be reversed to drive forward
		// Reverse the motor that runs backwards when connected directly to the battery
		rf.setDirection(DcMotor.Direction.FORWARD);
		lf.setDirection(DcMotor.Direction.REVERSE);
		rb.setDirection(DcMotor.Direction.FORWARD);
		lb.setDirection(DcMotor.Direction.REVERSE);
		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		runtime.reset();

		// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {
			
			 if(gamepad1.dpad_down) {
			leftspinny.setPower(0.7);
			rightspinny.setPower(-.7);
			sleep(100);
			leftspinny.setPower(0);
			rightspinny.setPower(0);
		} else if(gamepad1.dpad_up) {
			leftspinny.setPower(-.7);
			rightspinny.setPower(.7);
			sleep(100);
			leftspinny.setPower(0);
			rightspinny.setPower(0);
			
		} 
		
if(gamepad2.dpad_left) {
  
			MiddleLinearSlide.setPower(-0.4);
		  
			
		   
		   
		} else if(gamepad2.dpad_right) {
		  
			MiddleLinearSlide.setPower(-0.8);
			
		} 
		
		 
	   
			if (gamepad2.dpad_up) {
			 BottomLinearSlide.setPower(1);
			} else if  (gamepad2.dpad_down) {
			 BottomLinearSlide.setPower(0.1);
			}
			// Setup a variable for each drive wheel to save power level for telemetry
				// Choose to drive using either Tank Mode, or POV Mode
			// Comment out the method that's not used.  The default below is POV.

			// POV Mode uses left stick to go forward, and right stick to turn.
			// - This uses basic math to combine motions and is easier to drive straight.
			double forward = gamepad1.right_stick_y;
			double turn  =  gamepad1.left_stick_x;
			double strafe = -gamepad1.right_stick_x;
			
		  //  double forward2 = gamepad2.right_stick_y;
			//double turn2  =  gamepad2.left_stick_x;
			//double strafe2 = -gamepad2.right_stick_x;
			
			
			
			rf.setPower(forward+turn-strafe);
			lb.setPower(forward+turn+strafe);			
			rb.setPower(forward-turn-strafe);			
			lf.setPower(forward-turn+strafe);   
			
			//rf.setPower(forward2+turn2-strafe2);
			//lb.setPower(forward2+turn2+strafe2);			
			//rb.setPower(forward2-turn2-strafe2);			
			//lf.setPower(forward2-turn2+strafe2);   
			
			
			


  //		   Tests1.setPosition(0.6);
  //		  Tests2.setPosition(0.27);
	
			 // Put loop blocks here.
			if (gamepad2.a) {
			 ls.setPower(0.1);
			rs.setPower(-0.6);
			} else if  (gamepad2.b) {
			 ls.setPower(-1);
			rs.setPower(0.4);
			}
			 if (gamepad2.x) {
			 topServo.setPower(0.5);
			} else if  (gamepad2.y) {
			 topServo.setPower(-1);
			}
			
			if (gamepad2.right_bumper) {
				 ClampServo.setPower(0.9);
			} else if (gamepad2.left_bumper) {
			   ClampServo.setPower(-0.8);
			}
		   
		   if (gamepad1.y) {
				servoArm.setPower(-0.8);
			} else if (gamepad1.x) {
				servoArm.setPower(1);
			}
		 
					if (gamepad1.a) {
				leftServoArmClamp.setPower(-0.3);
			} else if (gamepad1.b) {
				 leftServoArmClamp.setPower(0.9);
			}
		 
		  if (gamepad1.right_bumper) {
				ServoArmClamp.setPower(-0.3);
			} else if (gamepad1.left_bumper) {
				ServoArmClamp.setPower(0.9);
			}
			 if (gamepad1.dpad_right) {
				 LeftServoArm.setPower(-0.8);
				
			} else if (gamepad1.dpad_left) {
			   
				LeftServoArm.setPower(0.7);
			}
		 
		 
			
		  //	leftPower	= Range.clip(drive + turn, -1.0, 1.0) ;
		  //  rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

			// Tank Mode uses one stick to control each wheel.
			// - This requires no math, but it is hard to drive forward slowly and keep straight.
			// leftPower  = -gamepad1.left_stick_y ;
			// rightPower = -gamepad1.right_stick_y ;
 
			// Send calculated power to wheels
			//leftDrive.setPower(leftPower);
			//rightDrive.setPower(rightPower);

			// Show the elapsed game time and wheel power.
			telemetry.addData("Status", "Run Time: " + runtime.toString());
			//telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
			telemetry.update();
		}
	}
}

