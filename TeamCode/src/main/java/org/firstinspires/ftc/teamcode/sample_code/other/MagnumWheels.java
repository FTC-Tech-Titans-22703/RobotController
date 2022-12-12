package org.firstinspires.ftc.teamcode.sample_code.other;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MagnumWheels {
    private DcMotor rightBack;
    private DcMotor leftBack;

    public DcMotor getRightBack() {
        return rightBack;
    }

    public DcMotor getLeftBack() {
        return leftBack;
    }

    public DcMotor getRightFront() {
        return rightFront;
    }

    public DcMotor getLeftFront() {
        return leftFront;
    }

    private DcMotor rightFront;
    private DcMotor leftFront;
    public MagnumWheels(HardwareMap map)
    {
         rightBack = map.get(DcMotor.class, "rightBack");
         leftBack = map.get(DcMotor.class, "leftBack");
         rightFront = map.get(DcMotor.class, "rightFront");
         leftFront = map.get(DcMotor.class, "leftFront");
         rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         rightBack.setTargetPosition(0);
         rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         rightFront.setTargetPosition(0);
         rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         leftBack.setTargetPosition(0);
         leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         leftFront.setTargetPosition(0);
         leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         leftFront.setDirection(DcMotor.Direction.REVERSE);
         leftBack.setDirection(DcMotor.Direction.REVERSE);
         }
    public void stopAndReset(){
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setTargetPosition(0);
         rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
rightFront.setTargetPosition(0);
         rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
leftBack.setTargetPosition(0);
         leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
leftFront.setTargetPosition(0);
         leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}

