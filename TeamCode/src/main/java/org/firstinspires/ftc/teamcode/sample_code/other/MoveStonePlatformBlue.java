

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
@Autonomous(name = "MoveStonePlatformBlue", group = "")
//@Disabled
public class MoveStonePlatformBlue extends LinearOpMode {
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;
    private DcMotor lf;
    private DcMotor rightspinny;
    private DcMotor leftspinny;
    private DcMotor clawArm;
    private CRServo servoArm;
    private CRServo ServoArmClamp;
    private CRServo ls;
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
      ServoArmClamp= hardwareMap.get(CRServo.class, "ServoArmClamp");
     ls = hardwareMap.get(CRServo.class,"ls");
     rs = hardwareMap.get(CRServo.class,"rs");

     rf.setDirection(DcMotor.Direction.REVERSE);
      lf.setDirection(DcMotor.Direction.FORWARD);
      rb.setDirection(DcMotor.Direction.REVERSE);
      lb.setDirection(DcMotor.Direction.FORWARD);

      servoArm = hardwareMap.get(CRServo.class, "servoArm");
      clawArm = hardwareMap.get(DcMotor.class, "Arm");
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
        /** Wait for the game to begin */
        teleLog("Press Play to start op mode");
    
        initHardwareMembers();
        teleLog("Initialized Multi Move 23");
        
        waitForStart();
    
        // (1) Fetch first skystone
        // Put the claw arm down
        
    
        // Move forward to stone line
        strafeRightStraight(0.3, 3300);
        sleep(200);
    
        // Grab 
        servoArmDown(0.18, 200);
        sleep(1200);
        ServoArmClamp(-0.3, 100);
        sleep(200);
         
        servoArmUp(0.9, 200);
        sleep(200);
      
         // Come back a bit    
        strafeLeftStraight(0.3, 100);     
    
        // Strafe to drag the stone to the other side of the bridge
        strafeBackward(0.7, 2600);
        sleep(100);
        
        // go forward towards the platform
        strafeRightStraight(0.3, 1300);
        sleep(100);
      
        // Release the skystone
        servoArmDown(0.18, 200);
          sleep(100);   
        ServoArmClamp(0.8, 100);
        sleep(100);
         
        servoArmUp(0.9, 200);
        sleep(1000);
        
        
        
     
        strafeLeftStraight(0.3, 300);
        
        //turn to face platform
        lf.setPower(-0.65);
        rf.setPower(0.65);
        rb.setPower(0.65);
        lb.setPower(-0.65);
        sleep(100);
     
 
        servoArmUp(0.9, 200);
       sleep(100);
    
       // hit platform
        strafeBackward(0.3, 500);
        sleep(100);
        

    
        lf.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        sleep(1000);
        
        
        
    
        
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
             strafeForward(0.9, 2200);
             
             //lift servos
              ls.setPower(0);
             rs.setPower(0);
             sleep(100);
             
             
             
             //strafe right
             strafeRightStraight(0.5, 2300);
             
             
             
             //go backward
             strafeBackward(0.8, 800);
             
             
              //park
             strafeRightStraight(0.4, 1680);

     
   /*
        // Come back a bit    
       strafeLeftStraight(0.3, 50);
        
       // rf.setPower(0.1);
        // lf.setPower(0);
        sleep(100);
        
        
        // Come back across bridge
        strafeForward(0.7, 2100);
        sleep(100);
    
    
        // (2) Fetch second skystone
        
        //Move forward slightly before grabbing
        strafeRightStraight(0.3, 800);
        
        //Code to grab and deliver second skystone
        // Grab
        sleep(100);
        servoArmDown(0.18, 200);
        sleep(1200);
        ServoArmClamp(-0.3, 100);
        sleep(400);
         
        servoArmUp(0.9, 200);
        sleep(1200);
      
        // Come back a bit
      //  strafeLeftStraight(0.3, 90);
    
        //rf.setPower(0.8);
        //lb.setPower(-0.8);
        sleep(200);
        
        // Strafe to drag the stone to the other side of the bridge
        strafeBackward(0.7, 2500);
      
        // Go forward towards the platform
        strafeRightStraight(0.4, 200);
        
      
        servoArmDown(0.18, 200);
        sleep(1200);
        ServoArmClamp(0.5, 100);
        sleep(400);
         
         servoArmUp(0.9, 200);
        sleep(1200);
             
             
        
        /*
        // turn to fix robot angle
        //lf.setPower(0.5);
          //  rf.setPower(-0.5);
            //rb.setPower(-0.5);
            //lb.setPower(0.5);
        //sleep(100);
        
       /* strafeForward(0.6, 200);
        
        // Approach stone line
        strafeRightStraight(0.8, 1200);
    
        // Grab 
        servoArmDown(0.18, 200);
        sleep(1000);
        
        // Come back a bit
        strafeLeftStraight(0.8, 1090);


            rb.setPower(-0.5);
            lb.setPower(0.5);
            sleep(340);
            
            
        // Strafe to drag the stone to the other side of the bridge
        strafeBackward(1.0, 2000);
    
    
            
    // Release the skystone
        servoArmUp(0.55, 200);
        
        rb.setPower(-0.5);
            lb.setPower(0.5);
            sleep(230);
            
        // (3) Fetch third skystone
        // Come back across bridge
        strafeForward(1.0, 2380);
        
        // turn to fix robot angle
        lf.setPower(0.4);
            rf.setPower(-0.4);
            rb.setPower(-0.4);
            lb.setPower(0.4);
        // sleep(100);
        
         strafeForward(1.0, 230);
         
        // Approach stone line
        strafeRightStraight(0.8, 1100);
        
        // Grab 
        servoArmDown(0.18, 200);
        sleep(1000);
        
        // Come back a bit
        strafeLeftStraight(0.8, 1090);
    
        // Strafe to drag the stone to the other side of the bridge
        strafeBackward(1.0, 2300);
    
        // Release the skystone
        servoArmUp(0.55, 200);
    
        // (4) Parking
        strafeForward(0.8, 500);  */  }

}
