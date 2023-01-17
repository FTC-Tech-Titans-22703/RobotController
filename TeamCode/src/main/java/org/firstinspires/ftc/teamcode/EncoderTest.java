package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Encoder Test", group = "1")
public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor a = hardwareMap.get(DcMotor.class, "leftFront");
        a.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        a.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor b = hardwareMap.get(DcMotor.class, "rightFront");
        b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor c = hardwareMap.get(DcMotor.class, "leftRear");
        c.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        c.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("pos1", a.getCurrentPosition());
            telemetry.addData("pos2", b.getCurrentPosition());
            telemetry.addData("pos3", c.getCurrentPosition());
            telemetry.update();
        }
    }
}
