package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "park")
public class Park extends LinearOpMode {
    private static final String VUFORIA_KEY = "AZFoUtb/////AAABmVcqdqh4G0zSnWXpJgpaoplvJEqgE1OC7SQ3+f030e5I5eXU/VORYQq9Zw361nR+CuE0RgRgugUi3cdKGWDRMh7VdPkf629Y9YmHScVYl47vqN6ZPUuwZkcCwS5nix00Sl6+R1zV5LdkyxFPRJgY2kguEx5LOGszqEgiWcD/lS8FzGBjGuv37KZ9r4TbKypGeMbBlIaNP/3ZehcJBzyhojFKrya0eYtxpSrm3fdOieH9ReMhSURRoVF+NE+cJoUSrRvGtc0dhN/8Yr3dgC5s3fUm/yMoJ+x4aQJtvrBgjKqHtBDUci2u0PuZSpEkObOkcgqkv271tzspK2NP+0Q0wUi+HBoJSKQ1Z0a8i3nwDw+0";
    private VuforiaSkyStone vuforiaSkyStone;
    private VuforiaLocalizer vuforia;
    private TfodSkyStone tfodSkyStone;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    ElapsedTime time;
    MagnumWheels drive;
    DcMotor slide;
    DcMotor parallelBar;
    CRServo armServo;
    Servo dragOne;
    Servo dragTwo;
    public void runOpMode() {
        
        time = new ElapsedTime();
        
        drive = new MagnumWheels(hardwareMap);
        telemetry.addData("hehe",113);
    telemetry.update();
        slide = hardwareMap.get(DcMotor.class,"slide");
        parallelBar = hardwareMap.get(DcMotor.class,"parallelBar");
        armServo = hardwareMap.get(CRServo.class,"armServo");
        dragOne = hardwareMap.get(Servo.class,"dragOne");
        dragTwo = hardwareMap.get(Servo.class,"dragTwo");

        final double right = -1;
        final double left = -10;
        //  Instantiate the Vuforia engine
        // Initialization
        telemetry.addData("Init ", "started");
        telemetry.update();
        // Init Vuforia because Tensor Flow needs it.
        // Let's use 70% minimum confidence and
        // and no object tracker.
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        
        waitForStart();
        //do stuff here

        drive.stopAndReset();
      drive.stopAndReset();
        moveForward(40,825,"Encoder");
        drive.stopAndReset();
        //150
        moveRight(100,150,"Encoder");
    
        center();
    }
    public void left(){
        drive.stopAndReset();
        //party
        moveLeft(50,200,"Encoder");
        drive.stopAndReset();
        //party
        parallelBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelBar.setTargetPosition(0);
        //party
        parallelBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&parallelBar.isBusy()){
            //party

        }
        moveArmUp(100,125*4,"Encoder");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&slide.isBusy()){

        }
        moveSlideBackward(100,750*4,"Encoder");
        parallelBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelBar.setTargetPosition(0);
        parallelBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&parallelBar.isBusy()){

        }
        moveArmDown(100,1200,"Encoder");
        drive.stopAndReset();
        moveForward(50,450,"Encoder");
        armServo.setPower(-1);
        time.reset();
        while(opModeIsActive() &&!isStopRequested()&&time.milliseconds()<1500){

        }
        armServo.setPower(0);
        drive.stopAndReset();
        moveBackward(50,450,"Encoder");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&slide.isBusy()){

        }
        moveSlideForward(100,312.5*4.3,"Encoder");
        armServo.setPower(-.5);
        drive.stopAndReset();
        moveLeft(75,3125,"Encoder");
        parallelBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelBar.setTargetPosition(0);
        parallelBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&parallelBar.isBusy()){

        }
        armServo.setPower(-.5);
        moveArmUp(100,500*4,"Encoder");
        drive.stopAndReset();
        moveForward(100,800,"Encoder");
        armServo.setPower(1);
        time.reset();
        while(opModeIsActive() &&!isStopRequested()&&time.milliseconds()<1000){

        }
        drive.stopAndReset();
        moveBackward(100,425,"Encoder");//party
        //party
          parallelBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          //party
        parallelBar.setTargetPosition(0);
        //party
        parallelBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //party
        while (opModeIsActive() &&!isStopRequested()&&parallelBar.isBusy()){

        }
        moveArmDown(100,250*4,"Encoder");
        drive.stopAndReset();
        moveTurnCenterCCW(100,850,"EncOder");
        drive.stopAndReset();
        moveRight(30,600,"Encoder");
        dragOne.setPosition(1);
        dragTwo.setPosition(.18);
        time.reset();
        while (opModeIsActive() &&!isStopRequested()&&time.milliseconds() < 1000){
            
        }
        drive.stopAndReset();
        moveLeft(75,1200,"Encoder");
        drive.stopAndReset();
        moveBackward(75,1200,"Encoder");
        drive.stopAndReset();
        moveTurnCenterCCW(50,1300,"ENCODER");
        drive.stopAndReset();
        moveForward(50,300,"Encoder");
        dragOne.setPosition(.18);
        dragTwo.setPosition(1);
        time.reset();
        while (opModeIsActive() &&!isStopRequested()&&time.milliseconds() < 1000){
            
        }
        drive.stopAndReset();
        moveRight(75,500,"Encoder");
        drive.stopAndReset();
        moveTurnCenterCCW(100,1300,"Encoder");
        drive.stopAndReset();
        
        moveForward(100,1100,"Encoder");
        
   
    }
    private void center() {
        drive.stopAndReset();
        //party
        moveLeft(50,75,"Encoder");
        drive.stopAndReset();
        //party
        parallelBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelBar.setTargetPosition(0);
        //party
        parallelBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&parallelBar.isBusy()){
            //party

        }
        moveArmUp(100,125*4,"Encoder");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&slide.isBusy()){

        }
        moveSlideBackward(100,750*4,"Encoder");
        parallelBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelBar.setTargetPosition(0);
        parallelBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&parallelBar.isBusy()){

        }
        moveArmDown(100,1200,"Encoder");
        drive.stopAndReset();
        moveForward(50,450,"Encoder");
        armServo.setPower(-1);
        time.reset();
        while(opModeIsActive() &&!isStopRequested()&&time.milliseconds()<1500){

        }
        armServo.setPower(0);
        drive.stopAndReset();
        moveBackward(50,450,"Encoder");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&slide.isBusy()){

        }
        moveSlideForward(100,312.5*4.3,"Encoder");
        armServo.setPower(-.5);
        drive.stopAndReset();
        moveLeft(75,3250,"Encoder");
        parallelBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelBar.setTargetPosition(0);
        parallelBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&parallelBar.isBusy()){

        }
        armServo.setPower(-.5);
        moveArmUp(100,500*4,"Encoder");
        drive.stopAndReset();
        moveForward(100,800,"Encoder");
        armServo.setPower(1);
        time.reset();
        while(opModeIsActive() &&!isStopRequested()&&time.milliseconds()<1000){

        }

    }
    public  void right(){
    
        drive.stopAndReset();
        moveRight(70,100,"Encoder");
        //party
        parallelBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelBar.setTargetPosition(0);
        //party
        parallelBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&parallelBar.isBusy()){
            //party

        }
        moveArmUp(100,125*4,"Encoder");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&slide.isBusy()){

        }
        moveSlideBackward(100,750*4,"Encoder");
        parallelBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelBar.setTargetPosition(0);
        parallelBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&parallelBar.isBusy()){

        }
        moveArmDown(100,1200,"Encoder");
        drive.stopAndReset();
        moveForward(50,450,"Encoder");
        armServo.setPower(-1);
        time.reset();
        while(opModeIsActive() &&!isStopRequested()&&time.milliseconds()<1500){

        }
        armServo.setPower(0);
        drive.stopAndReset();
        moveBackward(50,450,"Encoder");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&slide.isBusy()){

        }
        moveSlideForward(100,312.5*4.3,"Encoder");
        armServo.setPower(-.5);
        drive.stopAndReset();
        moveLeft(75,3325,"Encoder");
        parallelBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelBar.setTargetPosition(0);
        parallelBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() &&!isStopRequested()&&parallelBar.isBusy()){

        }
        armServo.setPower(-.5);
        moveArmUp(100,500*4,"Encoder");
        drive.stopAndReset();
        moveForward(100,800,"Encoder");
        armServo.setPower(1);
        time.reset();
        while(opModeIsActive() &&!isStopRequested()&&time.milliseconds()<1000){

        }
        drive.stopAndReset();
        moveBackward(100,425,"Encoder");//party
        //party
          parallelBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          //party
        parallelBar.setTargetPosition(0);
        //party
        parallelBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //party
        while (opModeIsActive() &&!isStopRequested()&&parallelBar.isBusy()){

        }
        moveArmDown(100,250*4,"Encoder");
          drive.stopAndReset();
        moveTurnCenterCCW(100,850,"EncOder");
        drive.stopAndReset();
        moveRight(30,600,"Encoder");
        dragOne.setPosition(1);
        dragTwo.setPosition(.18);
        time.reset();
        while (opModeIsActive() &&!isStopRequested()&&time.milliseconds() < 1000){
            
        }
        drive.stopAndReset();
        moveLeft(75,1200,"Encoder");
        drive.stopAndReset();
        moveBackward(75,1200,"Encoder");
        drive.stopAndReset();
        moveTurnCenterCCW(50,1300,"ENCODER");
        drive.stopAndReset();
        moveForward(50,300,"Encoder");
        dragOne.setPosition(.18);
        dragTwo.setPosition(1);
        time.reset();
        while (opModeIsActive() &&!isStopRequested()&&time.milliseconds() < 1000){
            
        }
        drive.stopAndReset();
        moveRight(75,500,"Encoder");
        drive.stopAndReset();
        moveTurnCenterCCW(100,900,"Encoder");
        drive.stopAndReset();
        
        moveForward(100,1000,"Encoder");
    }
   
    public HashMap<MotorWithDirection,Boolean>[] getMotorToUse(String direction)
    {
        String[] list = direction.split(" ");

        HashMap<MotorWithDirection,Boolean>[] motors = new HashMap[list.length];
        for (int k =0; k<motors.length;k+=1){
            motors[k] = new HashMap<>();
        }
        for (int i=0; i< list.length;i++) {
            switch (direction) {
                case "Left":
                    motors[i].put(new MotorWithDirection(drive.getRightBack(), true), true);
                    motors[i].put(new MotorWithDirection(drive.getLeftBack(), true), true);
                    motors[i].put(new MotorWithDirection(drive.getRightFront(), true), true);
                    motors[i].put(new MotorWithDirection(drive.getLeftFront(), true), true);
                    break;
                case "Right":
                    motors[i].put(new MotorWithDirection(drive.getRightBack(), false), true);
                    motors[i].put(new MotorWithDirection(drive.getLeftBack(), false), true);
                    motors[i].put(new MotorWithDirection(drive.getRightFront(), false), true);
                    motors[i].put(new MotorWithDirection(drive.getLeftFront(), false), true);
                    break;
                case "Forward":
                    motors[i].put(new MotorWithDirection(drive.getRightBack(), true), true);
                    motors[i].put(new MotorWithDirection(drive.getLeftBack(), false), true);
                    motors[i].put(new MotorWithDirection(drive.getRightFront(), false), true);
                    motors[i].put(new MotorWithDirection(drive.getLeftFront(), true), true);
                    break;
                case "Backward":
                    motors[i].put(new MotorWithDirection(drive.getRightBack(), false), true);
                    motors[i].put(new MotorWithDirection(drive.getLeftBack(), true), true);
                    motors[i].put(new MotorWithDirection(drive.getRightFront(), true), true);
                    motors[i].put(new MotorWithDirection(drive.getLeftFront(), false), true);
                    break;
                case "TurnCenterCCW":
                    motors[i].put(new MotorWithDirection(drive.getRightBack(), true), true);
                    motors[i].put(new MotorWithDirection(drive.getLeftBack(), false), true);
                    motors[i].put(new MotorWithDirection(drive.getRightFront(), true), true);
                    motors[i].put(new MotorWithDirection(drive.getLeftFront(), false), true);
                    break;
                case "TurnCenterCW":
                    motors[i].put(new MotorWithDirection(drive.getRightBack(), false), true);
                    motors[i].put(new MotorWithDirection(drive.getLeftBack(), true), true);
                    motors[i].put(new MotorWithDirection(drive.getRightFront(), false), true);
                    motors[i].put(new MotorWithDirection(drive.getLeftFront(), true), true);
                    break;
                case "SlideForward":
                    motors[i].put(new MotorWithDirection(slide, true), true);
                    break;
                case "SlideBackward":
                    motors[i].put(new MotorWithDirection(slide, false), true);
                    break;
                case "ArmUp":
                    motors[i].put(new MotorWithDirection(parallelBar, true), true);
                    break;
                case "ArmDown":
                    motors[i].put(new MotorWithDirection(parallelBar, false), true);
                    break;

            }
        }
        return motors;
    }
 private void move(double[] speed,double[] distanceOrTime,String encoderOrTime,String movementType)
    {
        for (int i =0; i< speed.length;i++){
            speed[i] /=100;
        }

        HashMap<MotorWithDirection,Boolean>[] motorConfig = getMotorToUse(movementType);
        boolean isBusy[] = new boolean[motorConfig.length];
        for (int i=0;i<isBusy.length;i++){
            isBusy[i] = true;
        }
        for (int i=0 ;i<motorConfig.length;i++) {
                for (Map.Entry<MotorWithDirection, Boolean> entry : motorConfig[i].entrySet()) {
                    DcMotor motor = entry.getKey().getMotor();
                    boolean direction = entry.getKey().getDirection();
                    motor.setPower(direction ? speed[i] : speed[i] * -1);
                    motor.setTargetPosition((int) (direction ? distanceOrTime[i] : distanceOrTime[i] * -1));
                
            }
        }
        while (opModeIsActive() &&!isStopRequested()&& atleastOneBusy(isBusy)) {
            for (int j=0; j< isBusy.length;j++) {
                for (Map.Entry<MotorWithDirection, Boolean> entry : motorConfig[j].entrySet()) {
                    DcMotor motor = entry.getKey().getMotor();
                    if (!motor.isBusy()) isBusy[j] = false;
                }
            }
            for (int j=0; j< isBusy.length;j++) {
                if (!isBusy[j]) {
                    for (Map.Entry<MotorWithDirection, Boolean> entry : motorConfig[j].entrySet()) {
                        DcMotor motor = entry.getKey().getMotor();
                        motor.setPower(0);
                    }
                }
            }
        }
    }
    private boolean atleastOneBusy(boolean[] b){
        for (boolean bool:b){
            if (bool) return true;
        }
        return false;
    }
    private void moveForward(double speed,double distanceOrTime,String encoderOrTime)
    {
        double[] spee = {speed};
        double[] distanceOrTim = {distanceOrTime};
        move(spee,distanceOrTim,encoderOrTime,"Forward");
    }
    private void moveBackward(double speed,double distanceOrTime,String encoderOrTime)
    {
        double[] spee = {speed};
        double[] distanceOrTim = {distanceOrTime};
        move(spee,distanceOrTim,encoderOrTime,"Backward");
    }
    private void moveRight(double speed,double distanceOrTime,String encoderOrTime)
    {
        double[] spee = {speed};
        double[] distanceOrTim = {distanceOrTime};
        move(spee,distanceOrTim,encoderOrTime,"Right");
    }
    private void moveLeft(double speed,double distanceOrTime,String encoderOrTime)
    {
        double[] spee = {speed};
        double[] distanceOrTim = {distanceOrTime};
        move(spee,distanceOrTim,encoderOrTime,"Left");
    }
    private void moveTurnCenterCCW(double speed,double distanceOrTime,String encoderOrTime)
    {
        double[] spee = {speed};
        double[] distanceOrTim = {distanceOrTime};
        move(spee,distanceOrTim,encoderOrTime,"TurnCenterCCW");
    }
    private void moveTurnCenterCW(double speed,double distanceOrTime,String encoderOrTime)
    {
        double[] spee = {speed};
        double[] distanceOrTim = {distanceOrTime};
        move(spee,distanceOrTim,encoderOrTime,"TurnCenterCW");
    }
    private void moveArmUp(double speed,double distanceOrTime,String encoderOrTime){
        double[] spee = {speed};
        double[] distanceOrTim = {distanceOrTime};
        move(spee,distanceOrTim,encoderOrTime,"ArmUp");
    }
    private void moveArmDown(double speed,double distanceOrTime,String encoderOrTime){
        double[] spee = {speed};
        double[] distanceOrTim = {distanceOrTime};
        move(spee,distanceOrTim,encoderOrTime,"ArmDown");
    }
    private void moveSlideForward(double speed,double distanceOrTime,String encoderOrTime){
        double[] spee = {speed};
        double[] distanceOrTim = {distanceOrTime};
        move(spee,distanceOrTim,encoderOrTime,"SlideForward");
    }
    private void moveSlideBackward(double speed,double distanceOrTime,String encoderOrTime){
        double[] spee = {speed};
        double[] distanceOrTim = {distanceOrTime};
        move(spee,distanceOrTim,encoderOrTime,"SlideBackward");
    }
}

