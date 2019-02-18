package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@TeleOp (name="Lift_Encoder", group="Iterative Opmode")

public class Lift_Encoder extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor lift = null;
    private DcMotor plow = null;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private Servo liftPin;
    private double dpadPower = 1.0;
    private double motorPower = 0.25;
    private int countPerRotation = 1120;
    private double countPerDegree = 16;
    private double liftDpadPower = 1.0;
    
    @Override
    public void init() {
    telemetry.addData("Status", "Initialized");
        
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        lift = hardwareMap.get(DcMotor.class, "lift");
        plow = hardwareMap.get(DcMotor.class, "plow");
        
        liftPin = hardwareMap.get(Servo.class,"liftPin");
        
        leftArm = hardwareMap.get(DcMotor.class, "leftSampleArm");
        rightArm = hardwareMap.get(DcMotor.class, "rightSampleArm");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    @Override
    public void start() {
    }
    
    @Override
    public void loop() {
        
        leftArm.setTargetPosition(0);
        leftArm.setPower(0.25);
        while (leftArm.isBusy())   
        {
            telemetry.addData("encoder-fwd", leftArm.getCurrentPosition() + "  busy=" + leftArm.isBusy());
            telemetry.update();
        }
        leftArm.setPower(0.0);
        
        rightArm.setTargetPosition(0);
        rightArm.setPower(0.25);
        while (rightArm.isBusy())   
        {
            telemetry.addData("encoder-fwd", rightArm.getCurrentPosition() + "  busy=" + rightArm.isBusy());
            telemetry.update();
        }
        rightArm.setPower(0.0);
        
        double leftPower = gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;
        double screwPower = gamepad2.left_stick_y;
        
        double plowPower = gamepad2.right_stick_y;
        
        double leftArmPower = .1;
        double rightArmPower = .1;

        telemetry.addData("leftPower",leftPower);
        telemetry.addData("rightPower",rightPower);

        leftDrive.setPower(rightPower * dpadPower);
        rightDrive.setPower(leftPower  * dpadPower);
        
        lift.setPower(-screwPower * liftDpadPower);
        
        // leftArm.setPower(leftArmPower);
        // rightArm.setPower(rightArmPower);
            
        double lpin = liftPin.getPosition();
        telemetry.addData("lpin",lpin);
        
    
    
    if (gamepad1.left_trigger==1) {
        //liftPin.setPosition(0.3);
        dpadPower = 0.5;
        telemetry.addData("SLOW","Drive Systems");
    } else {
        dpadPower = 1.0;
    }
    
    if(gamepad1.dpad_up==true) {
        leftDrive.setPower(0.5*dpadPower);
        rightDrive.setPower(0.5*dpadPower);
    }
    
    if(gamepad1.dpad_down==true) {
        leftDrive.setPower(-0.5*dpadPower);
        rightDrive.setPower(-0.5*dpadPower);
    }
    
    if (gamepad2.left_trigger==1) {
        //liftPin.setPosition(0.3);
        liftDpadPower = 0.3;
        telemetry.addData("SLOW","Lift Mechanism");
    } else {
        liftDpadPower = 1.0;
    }
    
    plow.setPower(plowPower*.3);
    
    if (gamepad2.right_trigger==1) {
        liftPin.setPosition(0.3);
        telemetry.addData("test","test");
    } else {
        liftPin.setPosition(0.5); //deadbolt detach was 0.88
    }
    
    if (gamepad1.dpad_up==true) {
        leftDrive.setPower(-1);
        rightDrive.setPower(-1);
    } else {
        
    }
    //** Slow down the motor if leftTrigger on gamepad1 is pressed
    if (gamepad1.dpad_down==true) {
        leftDrive.setPower(1 * dpadPower);
        rightDrive.setPower(1 * dpadPower);
    } else {
        
    }
    
    // reverseEncoder(0);
    
    telemetry.addData("servo",liftPin.getPosition());

    telemetry.addData("plow",plow.getCurrentPosition());
    
    }
    
    
// public void reverseEncoder(double pos){
//     //** use encoder instead of time on motors 
//     //** We still need to come up with a method to convert distance to pos
    
//         leftDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
//         rightDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        
//         leftArm.setTargetPosition((int)(pos*countPerRotation));
//         rightArm.setTargetPosition((int)(pos*countPerRotation));

//         leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
//         leftArm.setPower(motorPower);
//         rightArm.setPower(motorPower);
//         //Wait for both motors to stop
//         while (leftDrive.isBusy() || rightDrive.isBusy()){

// }
// }
        
    @Override
    public void stop() {

    }
}
