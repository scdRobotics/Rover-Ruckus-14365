package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="PIDExperiment", group="Iterative Opmode")
@Disabled
public class PIDExperiment extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor lift = null;
    private DcMotor plow = null;
    private DcMotorEx rightSampleArm;
    private DcMotorEx leftSampleArm;
    private Servo liftPin;
    private DcMotor dumpingBin = null;
    private double dpadPower = 1.0;
    private double motorPower = 0.25;
    private int countPerRotation = 1120;
    private double countPerDegree = 16;
    private double liftDpadPower = 1.0;
    
    public static  double NEW_P = 2.5;
    public static  double NEW_I = 0.1;
    public static  double NEW_D = 0.2;
    
    @Override
    public void init() {
    telemetry.addData("Status", "Initialized");
        
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        lift = hardwareMap.get(DcMotor.class, "lift");
        plow = hardwareMap.get(DcMotor.class, "plow");
        
        dumpingBin = hardwareMap.get(DcMotor.class, "dumpingBin");
        
        liftPin = hardwareMap.get(Servo.class,"liftPin");
        
        rightSampleArm = hardwareMap.get(DcMotorEx.class,"rightSampleArm");
        leftSampleArm = hardwareMap.get(DcMotorEx.class,"leftSampleArm");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        changePidSettings(NEW_P,NEW_I,NEW_D);
    }
    
    @Override
    public void start() {
    }
    
    @Override
    public void loop() {
        
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
    
    if (gamepad1.x==true) {
        dumpingBin.setPower(0.5);
        telemetry.addData("dump","up");
    } else {
        if (gamepad1.b==true) {
            dumpingBin.setPower(-0.2);
            telemetry.addData("dump","down");
        } else {
            if (gamepad1.y==true) {
                dumpingBin.setPower(0.1);
                telemetry.addData("dump","fighting");
            } else {
                dumpingBin.setPower(0);
                telemetry.update();
            }
        }
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
    
    public void changePidSettings(double newP, double newI, double newD){
        //get the PID coeeficients for the run using encoder
        //*** NOTE:  original settings were 10, 3, and 0
        PIDCoefficients pidOrig = leftSampleArm.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidOrig2 = leftSampleArm.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        
        //read settings to confirm
//        PIDCoefficients pidMod = leftSampleArm.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        //telemetry.addData("Runtime", "%.03f", getRuntime());
        PIDCoefficients pidNew = new PIDCoefficients(newP, newI, newD);
        leftSampleArm.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidNew );
        
 
        leftSampleArm.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidNew);
        PIDCoefficients pidOrig3 = leftSampleArm.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidOrig4 = leftSampleArm.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Runtime", "%.03f", getRuntime());
        telemetry.addData("P,I,D (RUE)"," %.04f, %.04f, %.04f", pidOrig.p, pidOrig.i, pidOrig.d);
        telemetry.addData("P,I,D (RTP)"," %.04f, %.04f, %.04f", pidOrig2.p, pidOrig2.i, pidOrig2.d);
        telemetry.addData("P,I,D (RUE)"," %.04f, %.04f, %.04f", pidOrig3.p, pidOrig3.i, pidOrig3.d);
        telemetry.addData("P,I,D (RTP)"," %.04f, %.04f, %.04f", pidOrig4.p, pidOrig4.i, pidOrig4.d);
        //telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f", pidMod.p, pidMod.i, pidMod.d);
        telemetry.update();
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
