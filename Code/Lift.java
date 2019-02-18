package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="Lift", group="Iterative Opmode")
public class Lift extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor lift = null;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    
    private DcMotorEx rightSampleArm;
    private DcMotorEx leftSampleArm;
    private DcMotor intakeExtension = null;
    private DcMotor intakeLift = null;
    private Servo liftPin;
    private Servo plowServo;
    private CRServo intake;
    private double dpadPower = 1.0;
    private double motorPower = 0.25;
    private int countPerRotation = 1120;
    private double countPerDegree = 16;
    private double liftDpadPower = 1.0;
    
    private double sampleArmPower = 0.5;
    private boolean initialize = true;
    String driveType = "intake";
    
    @Override
    public void init() {
    telemetry.addData("Status", "Initialized");
        
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        lift = hardwareMap.get(DcMotor.class, "lift");
       
        intake = hardwareMap.get(CRServo.class, "intakeWheel");
        
        liftPin = hardwareMap.get(Servo.class,"liftPin");
        plowServo = hardwareMap.get(Servo.class,"plowServo");
        
        //leftArm = hardwareMap.get(DcMotor.class, "leftArm");
        //rightArm = hardwareMap.get(DcMotor.class, "rightArm");
        
        //intakeExtension = hardwareMap.get(DcMotor.class, "intakeExtension");
        intakeLift = hardwareMap.get(DcMotor.class, "intakeLift");

        rightSampleArm = hardwareMap.get(DcMotorEx.class,"rightSampleArm");
        leftSampleArm = hardwareMap.get(DcMotorEx.class,"leftSampleArm");
        
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
        // rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
    }
    
    @Override
    public void start() {
        
    }
    
    @Override
    public void loop() {
        //*** everything inside initialize will happen at beggining
        //** of teleop - if we have any initial work you want to automate
        //** once run it will not run again
        if (initialize){
            //leftSampleArm.setMode(DcMotor.RunMode.RESET_ENCODERS);
            //leftSampleArm.setTargetPosition(25);
            //leftSampleArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //leftSampleArm.setPower(0.25);
            //while (leftSampleArm.isBusy())   
            //{
            //     telemetry.addData("encoder-fwd", leftArm.getCurrentPosition() + "  busy=" + leftArm.isBusy());
            //     telemetry.update();
            //}
            //leftSampleArm.setPower(0.0);
            initialize = false;
        }
   
        
        double leftPower = gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;
        double screwPower = gamepad2.left_stick_y;
        
        double plowPower = gamepad2.right_stick_y;
        
        double leftArmPower = .1;
        double rightArmPower = .1;
        
        double sampleArmPower = 0.5;

        telemetry.addData("leftPower",leftPower);
        telemetry.addData("rightPower",rightPower);

        leftDrive.setPower(-leftPower * dpadPower);
        rightDrive.setPower(-rightPower  * dpadPower);
        
        lift.setPower(-screwPower * liftDpadPower);
        
        // leftArm.setPower(leftArmPower);
        // rightArm.setPower(rightArmPower);
            
        double lpin = liftPin.getPosition();
        telemetry.addData("lpin",lpin);
    
    
    
    if (gamepad1.left_trigger==1) {
        //liftPin.setPosition(0.3);
        dpadPower = 0.5;
        telemetry.addData("SLOW","Drive Systems");
        telemetry.update();
    } else {
        dpadPower = 1.0;
    }
    
    intake.setPower(0);
    if (gamepad2.left_bumper == true) {
        intake.setPower(1);
    }else {
        intake.setPower(0);
    }
    if (gamepad2.right_bumper == true) {
        intake.setPower(-1);
    }else {
        //intake.setPower(0);
    }
    
    intakeLift.setPower(gamepad2.right_stick_y * .40);
    
    if(gamepad1.dpad_up==true) {
        leftDrive.setPower(0.5*dpadPower);
        rightDrive.setPower(0.5*dpadPower);
    }
    
    if(gamepad1.dpad_down==true) {
        leftDrive.setPower(-0.5*dpadPower);
        rightDrive.setPower(-0.5*dpadPower);
    }
    
    if (gamepad1.left_trigger==1) {
        //liftPin.setPosition(0.3);
        liftDpadPower = 0.3;
        telemetry.addData("SLOW","Lift Mechanism");
    } else {
        liftDpadPower = 1.0;
    }
    //**** Plow Servo
    if (gamepad2.a==true) {
        plowServo.setPosition(0.1);
        telemetry.addData("plowServo","Plow Down");
    } 
    if (gamepad2.y==true) {
        plowServo.setPosition(0.8);
        telemetry.addData("plowServo","Plow Up");
    } 
    //*******
   // plow.setPower(plowPower*.3);
    
    if (gamepad2.right_trigger==1) {
        liftPin.setPosition(0.3);
        telemetry.addData("test","test");
    } else {
        liftPin.setPosition(0.6); //deadbolt detach was 0.88
    }
    
    if (gamepad1.dpad_up==true) {
        leftDrive.setPower(-1);
        rightDrive.setPower(-1);
    } else {
        
    }

    if (gamepad1.dpad_down==true) {
        leftDrive.setPower(1 * dpadPower);
        rightDrive.setPower(1 * dpadPower);
    } else {
        
    }
    
    if (gamepad2.dpad_up==true) {
        leftSampleArm.setPower(0.3);
        rightSampleArm.setPower(-0.3);
    } else {
        if (gamepad2.dpad_down==true) {
            leftSampleArm.setPower(-0.3);
            rightSampleArm.setPower(0.3);
        } else {
            leftSampleArm.setPower(0);
            rightSampleArm.setPower(0);
        }
    }
    
    // (gamepad2.right_bumper==true) {
    //    intakeExtension.setPower(1);
    //}
    
    //if (gamepad2.left_bumper==true) {
    //    intakeExtension.setPower(-1);
    //}
    
    //if (gamepad2.left_bumper==false&&gamepad2.right_bumper==false){
    //    intakeExtension.setPower(0);
    //}
    
    telemetry.addData("servo",liftPin.getPosition());

    }

    public void deployLeftSamplingArmToVertical(){
        leftSampleArm.setTargetPosition(50);

        leftSampleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftSampleArm.setPower(sampleArmPower);
        while (leftSampleArm.isBusy()){

        }
        leftSampleArm.setPower(0);
    }
    
    public void deployRightSamplingArmToVertical(){
       rightSampleArm.setTargetPosition(28);

        rightSampleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        rightSampleArm.setPower(sampleArmPower);
        while (rightSampleArm.isBusy()){

            
        }
        rightSampleArm.setPower(0);
        
    }
    

        
    @Override
    public void stop() {

    }
}