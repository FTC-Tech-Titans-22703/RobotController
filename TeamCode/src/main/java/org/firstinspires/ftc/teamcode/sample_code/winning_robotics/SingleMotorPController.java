package org.firstinspires.ftc.teamcode.sample_code.winning_robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="P-Controller Single Motor control based on encoder", group="Exercises")
public class SingleMotorPController extends LinearOpMode {
    DcMotor elbowMotor;
    @Override
    public void runOpMode() throws InterruptedException {

        int encoderDegreesToAttain = 200;
        double minPower = 0.01;
        double maxPower = 0.5;
        PController pController = new PController(0.03);
        pController.setInputRange(50, 500);
        pController.setSetPoint(encoderDegreesToAttain);
        pController.setOutputRange(minPower, maxPower);

        elbowMotor = hardwareMap.dcMotor.get("elbow_motor");

        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("encoder position", elbowMotor.getCurrentPosition());
            telemetry.addData("power",
                    pController.getComputedOutput(elbowMotor.getCurrentPosition()));
            telemetry.update();
            if(elbowMotor.getCurrentPosition() <  encoderDegreesToAttain){
                elbowMotor.setPower(minPower +
                        pController.getComputedOutput(elbowMotor.getCurrentPosition()));
            } else {
                elbowMotor.setPower(minPower -
                        pController.getComputedOutput(elbowMotor.getCurrentPosition()));
            }

        }
        elbowMotor.setPower(0);
    }
}