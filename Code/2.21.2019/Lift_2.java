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

@TeleOp (name="Lift_2", group="Iterative Opmode")
public class Lift_2 extends OpMode {
@Disabled

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor lift = null;
    private DcMotor plow = null;
    private Servo liftPin;
    private double dpadPower = 1.0;
    
    double leftPower;
    double rightPower;
    double screwPower;
    double plowPower;
    double slowButton;
    double latchButton;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        lift = hardwareMap.get(DcMotor.class, "lift");
        plow = hardwareMap.get(DcMotor.class, "plow");
        liftPin = hardwareMap.get(Servo.class,"liftPin");
        //** set the direction of our motors
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    
    @Override
    public void start() {
    }
    
    @Override
    public void loop() {
        
        //** mapping the controls to certain values that will be
        //** used to control motors and servos
        //** Wheel motors
        leftPower = gamepad1.right_stick_y;
        rightPower = gamepad1.left_stick_y;
        //** Lift mechanism
        screwPower = gamepad2.left_stick_y;
        //*** Plow lower/raise
        plowPower = gamepad2.right_stick_y;

        slowButton = gamepad1.left_trigger;
        latchButton = gamepad2.right_trigger;

        
            
        double lpin = liftPin.getPosition();
        telemetry.addData("lpin",lpin);
        
        //** left trigger is a "slow mode" for driving
        if (slowButton==1) {
            dpadPower = 0.5;
        } else {
            dpadPower = 1.0;
        }
        //** wanted to slow the plow a little bit
        plow.setPower(plowPower*.3);
        
        //** Control the latching pin once ready to raise
        if (latchButton==1) {
            liftPin.setPosition(0.3);
        } else {
            liftPin.setPosition(0.88);
        }
        
        leftDrive.setPower(leftPower * dpadPower);
        rightDrive.setPower(rightPower  * dpadPower);
        
        lift.setPower(-screwPower);
    
        telemetry.addData("servo",liftPin.getPosition());
        telemetry.addData("plow",plow.getCurrentPosition());
    
    }

    @Override
    public void stop() {

    }
}
