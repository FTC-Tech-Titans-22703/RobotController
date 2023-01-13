package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Encoder Test", group = "Test")
public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor e = hardwareMap.get(DcMotor.class, "e");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("pos", e.getCurrentPosition());
            telemetry.update();
        }
    }
}
