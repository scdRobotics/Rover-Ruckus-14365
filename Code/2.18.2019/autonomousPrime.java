/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;




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
@Autonomous(name="AutonomousPrime", group="Linear Opmode")
@Disabled
public class autonomousPrime extends LinearOpMode {

    // Declare OpMode members.
    //counter to help keep runtime
    protected ElapsedTime runtime = new ElapsedTime();
    //** our wheel motors
    protected DcMotorEx leftDrive = null;
    protected DcMotorEx rightDrive = null;
    //**The lift motor and the lift "latch"
    protected DcMotor lift = null;
    protected Servo liftPin = null;
    protected Servo plowServo = null;
    //**The plow motor

    //** motors to move sampling arms
    //** using DcMotorEx because we may need to change 
    //** PID settings to hold arms in place
    protected DcMotorEx rightSampleArm;
    protected DcMotorEx leftSampleArm;


    //The two distance sensors on the sampling arms - not currently used
    protected DistanceSensor lDistance;
    protected DistanceSensor rDistance;
    //The two Color sensors on the sampling arms
    protected ColorSensor lColor;
    protected ColorSensor rColor;
    
    //** The two motors that extend and rotate the collector arm respectively
    //
    protected DcMotorEx intakeExtension;
    protected DcMotorEx intakeLift;
    //protected Servo intakeWheel; //*  rubber band wheel that collects elements
    //protected Servo intakeUnfold;  //* unfolds collection bucket
    
    
    //** used to vary the power to the motors
    protected double motorPower = 0.4; 
    protected double liftPower = 0.1;
    protected double sampleArmPower = 0.5;
    //** multipliers - change to CAPITAL since they are constants
    protected final int countPerRotation = 1120;
    protected final int countPerArmCM = 10;
    protected final double countPerDegree = 16;
    protected final double countPerLiftCM = 250;
    
    protected static  double NEW_P = 8.0;
    protected static  double NEW_I = 0.05;
    protected static  double NEW_D = 0.0;
    protected static int tolerance = 10;
    
    public void mapObjects(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        //*************************************************************************
        //** ultimately want to move all this setup to a seprate class called robot
        //** Currently getting cannot find symbol: variable hardwareMap 
        //** Robot robot = new Robot();
        //*************************************************************************
        
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightDrive  = hardwareMap.get(DcMotorEx.class, "leftDrive");
        leftDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        lift = hardwareMap.get(DcMotor.class, "lift");

        rightSampleArm = hardwareMap.get(DcMotorEx.class,"rightSampleArm");
        leftSampleArm = hardwareMap.get(DcMotorEx.class,"leftSampleArm");
        rightSampleArm.setDirection(DcMotor.Direction.REVERSE);
        leftSampleArm.setDirection(DcMotor.Direction.FORWARD);
        initializeSamplingArms();
        
        liftPin = hardwareMap.get(Servo.class,"liftPin");
        plowServo = hardwareMap.get(Servo.class,"plowServo");
        
        lColor = hardwareMap.colorSensor.get("lColor");
        lDistance = hardwareMap.get(DistanceSensor.class, "lColor");
        lColor.enableLed(true);
        
        rColor = hardwareMap.colorSensor.get("rColor");
        rDistance = hardwareMap.get(DistanceSensor.class, "rColor");
        rColor.enableLed(true);
        //intakeExtension = hardwareMap.get(DcMotorEx.class,"intakeExtend");;
        intakeLift = hardwareMap.get(DcMotorEx.class,"intakeLift");;
    
        //intakeWheel = hardwareMap.get(Servo.class,"intakeWheel"); 
        //intakeUnfold = hardwareMap.get(Servo.class,"intakeUnfold");
        
        changePidSettings(NEW_P,NEW_I,NEW_D);
        
        tolerance = 10;
        leftDrive.setTargetPositionTolerance(tolerance);
        rightDrive.setTargetPositionTolerance(tolerance);
    }
    @Override
    public void runOpMode() {
        
        mapObjects();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        

 
        
        //*Begin Autonomous
        //** Forward is where the Sampling Arms are

        //*Step #1 - Lower Robot

        //*** No encoder on the raise motor yet 
        //** moves very fast so we should get this on en encoder before use
            
            //Step #2 - detach hanging pin
        if (true) {
            raiseRobotArm(1.95); //landing
            liftPin.setPosition(0.75);
            pause(1);
        }

        //Step 3 - Move toward Sample

            forwardEncoder(2.44);//was 2.43

            pause(.1); 
            

            deployLeftSamplingArmToSample();
            deployRightSamplingArmToSample();

            //* pause to provide enough time for sampling arms to move
            //**Step #5 - sample using color sensors move arms to appropriate spots
            
            motorPower = 0.3;
            String SamplePos = getSample(lColor,rColor);
             if (SamplePos.equals("Left")){
                 goldLeft(); //** move right arm out of the way
             } else if (SamplePos.equals("Right")) {
                 goldRight(); //** move left arm out of the way
             } else {
                 goldMiddle();
             }
            storeBothSamplingArms(); //Just to be sure they are stored and safe
                     
        //** back up from Sample to prepare for turn
        motorPower = 0.4;
        reverseEncoder(1.25);
        
        //*** initial Move toward wall from Lander
        motorPower = 0.3;
        rightEncoder(90);
        motorPower = 0.4;
        reverseEncoder(5.2);
        
        //** First Adjust movement toward Claim
        motorPower = 0.4;
        leftEncoder(35);
        
        reverseEncoder(1.0);
        //** Second Adjust movement toward Claim
        motorPower = 0.4;
        leftEncoder(15);
        
        //** Move toward Claim
        motorPower = 0.9;
        reverseEncoder(4.5);
        //** At Claim - drop token
        dropPlow_Servo();
        motorPower = 0.9;
        //** Race to park
        forwardEncoder(7.0);
        leftEncoder(20);
        forwardEncoder(2.5);
        raisePlow_Servo();
        pause(1);
        stop();
 
    }
    
    //** !!!!!!!!!!!!!!! END OF RUNOP Method !!!!!!!!!!!!!!!!!!!!
    
    //******************** Start of all custom methods *************************
    
    //**************************************************************************
    //** move robot forward n number of distance
    //**************************************************************************    
    public void forwardEncoder(double pos){
    
        leftDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        leftDrive.setTargetPosition((int)(-pos*countPerRotation));
        rightDrive.setTargetPosition((int)(-pos*countPerRotation));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftDrive.setPower(motorPower);
        rightDrive.setPower(motorPower);
        //RIGHT IS THE ONLY ENCODER WORKING RIGHT NOW
        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())){
  
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    
    //**************************************************************************
    //** used to pause when previous operation needs time to complete
    //**************************************************************************
    public void pause(double secs){
        ElapsedTime mRuntime = new ElapsedTime();
        while(mRuntime.time()< secs){
            
        }
    }
    //**************************************************************************
    //** move robot backward n number of distance
    //**************************************************************************
    public void reverseEncoder(double pos){

        rightDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        leftDrive.setTargetPosition((int)(pos*countPerRotation));
        rightDrive.setTargetPosition((int)(pos*countPerRotation));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftDrive.setPower(motorPower);
        rightDrive.setPower(motorPower);
        //Wait for both motors to stop
        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())){
    
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    

    
    //**************************************************************************
    //** move robot left n number of degrees
    //**************************************************************************    
    public void leftEncoder(double pos){

        leftDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        leftDrive.setTargetPosition((int)(-pos*countPerDegree));
        rightDrive.setTargetPosition((int)(pos*countPerDegree));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftDrive.setPower(motorPower);
        rightDrive.setPower(motorPower);

        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())){
            
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    
    //**************************************************************************
    //** move robot right n number of degrees
    //**************************************************************************
    public void rightEncoder(double pos){
    
        leftDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        leftDrive.setTargetPosition((int)(pos*countPerDegree));
        rightDrive.setTargetPosition((int)(-pos*countPerDegree));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftDrive.setPower(motorPower);
        rightDrive.setPower(motorPower);
        //RIGHT IS THE ONLY ENCODER WORKING RIGHT NOW
        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())){
            
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    
    //**************************************************************************
    //** Raises Robot arm - which results in it being lowered to ground
    //**************************************************************************
    public void raiseRobotArm(double secs){
        ElapsedTime mRuntime = new ElapsedTime();
        lift.setPower(-1);
    
        while(mRuntime.time()< secs){
                
        }
        lift.setPower(0);
    
    }
    
    //**************************************************************************
    //** Raises Robot arm - which results in it being lowered to ground
    //**************************************************************************
    public void raiseRobotArmEncoder(double pos){
        lift.setMode(DcMotor.RunMode.RESET_ENCODERS);

        lift.setTargetPosition((int)(-pos*countPerLiftCM));

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        lift.setPower(1.0);

        //RIGHT IS THE ONLY ENCODER WORKING RIGHT NOW
        while (lift.isBusy()){
            //Do nothing
            
        }
        lift.setPower(0);
    
    }
    
    //**************************************************************************
    //** Lower Robot arm - which results in it being raised from ground
    //**************************************************************************    
    public void lowerRobotArmEncoder(double pos){
        lift.setMode(DcMotor.RunMode.RESET_ENCODERS);

        lift.setTargetPosition((int)(pos*countPerLiftCM));

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        lift.setPower(1.0);

        //RIGHT IS THE ONLY ENCODER WORKING RIGHT NOW
        while (lift.isBusy()){
            //Do nothing
            
        }
        lift.setPower(0);
    
    }
    
    //**************************************************************************
    //** Commands if gold is in the left position
    //**************************************************************************
    public void goldLeft(){
        deployLeftSamplingArmToSample();
        deployLeftSamplingArmToVertical();
        forwardEncoder(0.7);
        pause(.1);
        reverseEncoder(0.7);
    }
    
    
    //**************************************************************************
    //** Commands if gold is in the right position
    //**************************************************************************
    public void goldRight(){
        deployRightSamplingArmToSample();
        deployRightSamplingArmToVertical();
        forwardEncoder(0.7);
        pause(.1);
        reverseEncoder(0.7);
    }
    
    //**************************************************************************
    //** Commands if gold is in the middle position
    //**************************************************************************
    public void goldMiddle(){
        storeBothSamplingArms();
        forwardEncoder(1.50);
        pause(.5);
        reverseEncoder(1.50);
        pause(.5);
    }
    
    //**************************************************************************
    //** Method takes in the color values of both the left and right sampling 
    //** arms. The method then uses a ratio of blue to red values on each to 
    //** determine if it detects gold or silver. Depending on what it finds it 
    //** returns either "Right", "Left", or "Middle" - these are then used to 
    //** position the sampling arms for the push step.
    //**************************************************************************
    public String getSample(ColorSensor leftColor, ColorSensor rightColor) {
        //***  This routine is our algorithm to determine if the color sensor in
        //***  sensing the gold in the left, right or center position
        int lRed=0;
        int lGreen=0;
        int lBlue=0;
        int rRed=0;
        int rGreen=0;
        int rBlue=0;
        String goldPosition = "Left";
            
        lRed = leftColor.red();
        lBlue = leftColor.blue();
        lGreen = leftColor.green();
        //alpha = lColor.alpha();
        rRed = rightColor.red();
        rBlue = rightColor.blue();
        rGreen = rightColor.green();
        double lRatio = (double)lBlue/(double)lRed;
        double rRatio = (double)rBlue/(double)rRed;
            
        //** This is a very basic algorithm to determine "gold"
        //** We need to make this better
        if (lRatio < 0.8 && rRatio > 0.8){
            goldPosition = "Right";
        } else if (rRatio < 0.8 && lRatio > 0.8) {
            goldPosition = "Left";
        } else if (rRatio < 0.8 && rRatio < 0.8) {
            goldPosition = "Middle";
        } else if (rRatio > 0.8 && lRatio > 0.8) {
            goldPosition = "Middle";
        } else {
            goldPosition = "Middle";
        }
        telemetry.addData("left red (%)", lRed);
        telemetry.addData("left blue (%)", lBlue);
        telemetry.addData("left green (%)", lGreen);
        telemetry.addData("right red (%)", rRed);
        telemetry.addData("right blue (%)", rBlue);
        telemetry.addData("right green (%)", rGreen);
        telemetry.addData("Gold Position (%)", goldPosition);
        telemetry.addData("rRatio (%)", rRatio);
        telemetry.addData("lRatio (%)", lRatio);
    
        telemetry.update();
        
        return goldPosition;
        }
       
        
    //**************************************************************************
    //** Moved the hanger arm to release the robot from the lander
    //**************************************************************************
    public void moveHangArmOff(){
        liftPin.setPosition(0.5);
        pause(1); //keep this so we ensure robot does not move until pin is moved
    }
    
    
    //**************************************************************************
    //** Moved the hanger arm to attach the robot to the lander
    //**************************************************************************
    public void moveHangArmOn(){
        liftPin.setPosition(0.3);
        pause(1);
    }
    
    //**************************************************************************
    //** Moved the hanger arm to detach the robot from the lander
    //**************************************************************************
    public  void slideArmRight(double secs){
        liftPin.setPosition(0.3);
        
    }
    
    
    //**************************************************************************
    //** Moved the hanger arm to attach the robot to the lander
    //**************************************************************************
    public void slideArmLeft(double secs){
        liftPin.setPosition(0.6);
        
    }
    
    //**************************************************************************
    //** Lift Plow with new Servo
    //**************************************************************************
    public  void raisePlow_Servo(){
        plowServo.setPosition(0.1);
        
    }
    //**************************************************************************
    //** Drop Plow with new Servo
    //**************************************************************************
    public void dropPlow_Servo(){
        plowServo.setPosition(0.8);
        pause(0.5);
        
    }
    
    //**************************************************************************
    //** Moved the left sampling arm to sampling position
    //**************************************************************************
    public void deployLeftSamplingArmToSample(){
        leftSampleArm.setTargetPosition(110);

        leftSampleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftSampleArm.setPower(sampleArmPower);
        //RIGHT IS THE ONLY ENCODER WORKING RIGHT NOW
        while (leftSampleArm.isBusy()){
        //while (rightDrive.isBusy() ){
            
        }
        leftSampleArm.setPower(0);
    }
    
    
    //**************************************************************************
    //** Moved the right sampling arm to sampling position
    //**************************************************************************
    public void deployRightSamplingArmToSample(){
       rightSampleArm.setTargetPosition(100);

        rightSampleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        rightSampleArm.setPower(sampleArmPower);
        //RIGHT IS THE ONLY ENCODER WORKING RIGHT NOW
        while (rightSampleArm.isBusy()){
        //while (rightDrive.isBusy() ){
            
        }
        rightSampleArm.setPower(0);
    }
    
    //**************************************************************************
    //** Moved the left sampling arm to a vertical position
    //**************************************************************************
    public void deployLeftSamplingArmToVertical(){
        leftSampleArm.setTargetPosition(0);

        leftSampleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftSampleArm.setPower(sampleArmPower);
        //RIGHT IS THE ONLY ENCODER WORKING RIGHT NOW
        while (leftSampleArm.isBusy()){
        //while (rightDrive.isBusy() ){
            
        }
        leftSampleArm.setPower(0);
    }
    
    //**************************************************************************
    //** Moved the right sampling arm to a vertical position
    //**************************************************************************
    public void deployRightSamplingArmToVertical(){
       rightSampleArm.setTargetPosition(28);

        rightSampleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        rightSampleArm.setPower(sampleArmPower);
        //RIGHT IS THE ONLY ENCODER WORKING RIGHT NOW
        while (rightSampleArm.isBusy()){
        //while (rightDrive.isBusy() ){
            
        }
        rightSampleArm.setPower(0);
    }
    
    //**************************************************************************
    //** Moved the left & right sampling arm to a vertical position
    //**************************************************************************    
    public void bothArmVertical(){
        deployLeftSamplingArmToVertical();
        deployRightSamplingArmToVertical();
    }
    
    
    //**************************************************************************
    //** Moved the left sampling arm to middle position
    //** No longer used do to change in strategy
    //**************************************************************************
    public void deployLeftSamplingArmToMiddle(){
         leftSampleArm.setTargetPosition(-70);

        leftSampleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftSampleArm.setPower(sampleArmPower);
        //RIGHT IS THE ONLY ENCODER WORKING RIGHT NOW
        while (leftSampleArm.isBusy()){
        //while (rightDrive.isBusy() ){
            
        }
        leftSampleArm.setPower(0);
    }
    
    
    //**************************************************************************
    //** Store both sampling arms out of way
    //**************************************************************************
    public void storeBothSamplingArms(){
        storeLeftSamplingArm();
        storeRightSamplingArm();

    }
    
    
    //**************************************************************************
    //** Store right sampling arm out of way
    //**************************************************************************
    public void storeRightSamplingArm(){

        rightSampleArm.setTargetPosition(0);

        rightSampleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        rightSampleArm.setPower(sampleArmPower);
        //RIGHT IS THE ONLY ENCODER WORKING RIGHT NOW
        while (rightSampleArm.isBusy() ){
        //while (rightDrive.isBusy() ){
            
        }
        rightSampleArm.setPower(0);

    }
    
    
    //**************************************************************************
    //** Store left sampling arm out of way
    //**************************************************************************
    public void storeLeftSamplingArm() {

        leftSampleArm.setTargetPosition(10);

        leftSampleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftSampleArm.setPower(sampleArmPower);
        //RIGHT IS THE ONLY ENCODER WORKING RIGHT NOW
        while (leftSampleArm.isBusy()){
        //while (rightDrive.isBusy() ){
            
        }
        leftSampleArm.setPower(0);

    }
    
    
    //**************************************************************************
    //** initialize sampling arms
    //**************************************************************************
    public void initializeSamplingArms() {
        leftSampleArm.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightSampleArm.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        leftSampleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSampleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        rightSampleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSampleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        leftSampleArm.setPower(0);
        rightSampleArm.setPower(0);
        
    }
    
    
 
    //**************************************************************************
    //** Change our drive motor configuraitons
    //**************************************************************************
    public void changePidSettings(double newP, double newI, double newD){
        //get the PID coeeficients for the run using encoder
        //*** NOTE:  original settings were 10, 3, and 0
        
        PIDCoefficients pidNew = new PIDCoefficients(newP, newI, newD);
        leftDrive.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,pidNew );
        rightDrive.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,pidNew );
        
        //leftDrive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidNew );
        //rightDrive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidNew );
 
        //telemetry.addData("Runtime", "%.03f", getRuntime());
        //telemetry.addData("P,I,D (RUE)"," %.04f, %.04f, %.04f", pidOrig.p, pidOrig.i, pidOrig.d);
        //telemetry.addData("P,I,D (RTP)"," %.04f, %.04f, %.04f", pidOrig2.p, pidOrig2.i, pidOrig2.d);
        //telemetry.addData("P,I,D (RUE)"," %.04f, %.04f, %.04f", pidOrig3.p, pidOrig3.i, pidOrig3.d);
        //telemetry.addData("P,I,D (RTP)"," %.04f, %.04f, %.04f", pidOrig4.p, pidOrig4.i, pidOrig4.d);
        //telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f", pidMod.p, pidMod.i, pidMod.d);
        //telemetry.update();
    }
    
}
