

package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.other;

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
@Autonomous(name = "JustParkRedBlock", group = "")
//@Disabled
public class JustParkRedBlock extends LinearOpMode {
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;
    private DcMotor lf;
    private DcMotor rightspinny;
    private DcMotor leftspinny;
   // private DcMotor clawArm;
    private CRServo servoArm;
        private CRServo ls;

    private CRServo rs;

     private CRServo ServoArmClamp;

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

      rf.setDirection(DcMotor.Direction.REVERSE);
      lf.setDirection(DcMotor.Direction.FORWARD);
      rb.setDirection(DcMotor.Direction.REVERSE);
      lb.setDirection(DcMotor.Direction.FORWARD);

      servoArm = hardwareMap.get(CRServo.class, "servoArm");
            ls = hardwareMap.get(CRServo.class, "ls");

      rs = hardwareMap.get(CRServo.class, "rs");

     // clawArm = hardwareMap.get(DcMotor.class, "Arm");
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
        sleep(0);
        
    strafeLeft(0.7,70);//close to the wall:70, far from the wall: 1000
       strafeForward(0.4, 1000);  }

}
