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

@TeleOp (name="play_copy2", group="Iterative Opmode")
@Disabled
public class play_Copy2 extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    private DcMotor lift3 = null;
    private DcMotor lift4 = null;
    // private servo hook = null;
    // private ColorSensor lColor; 
    // private DistanceSensor sensorColorRange; 
    // red color level sence
    // private int r;
    // blue color level sence
    // private int b;
    // greem color level sence
    // private int g;
    // private int alpha;
    // private double dist;
    // private int target = 0;
    
    @Override
    public void init() {
    telemetry.addData("Status", "Initialized");
    
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift3 = hardwareMap.get(DcMotor.class, "lift3");
        lift4 = hardwareMap.get(DcMotor.class, "lift4");
        // sensorColorRange = hardwareMap.get(DistanceSensor.class, "lColor");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        lift1.setDirection(DcMotor.Direction.FORWARD);
        lift2.setDirection(DcMotor.Direction.FORWARD);
        lift3.setDirection(DcMotor.Direction.FORWARD);
        lift4.setDirection(DcMotor.Direction.FORWARD);
        
        
        //leftArm.setDirection(DcMotor.Direction.REVERSE);
        //rightArm.setDirection(DcMotor.Direction.FORWARD);
        
        // lColor = hardwareMap.colorSensor.get("lColor");
        
        
         
        
        telemetry.addData("asdfasdf","asdfasdf");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();  
    }

    @Override
    public void loop() {
        
        /*states movement control stick*/
        double rightPower = gamepad1.left_stick_y;
        double leftPower = gamepad1.right_stick_y;
        double screwPower = gamepad2.left_stick_y;
        
        /*         if(r >=50)
         {
            leftPower = 1;
            rightPower = 1;
            if(r < 50)
            {
                leftPower = gamepad1.left_stick_y;
                rightPower = gamepad1.right_stick_y;
                
            }
           
         } 
         */
        /*states power of motor movement*/
        if (gamepad2.left_trigger < .01) {
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        
            lift1.setPower(screwPower);
            lift2.setPower(screwPower);
            lift3.setPower(screwPower);
            lift4.setPower(screwPower);
        }
        
        if (gamepad1.left_trigger > 0.2) {
            leftDrive.setPower(leftPower / 3);
            rightDrive.setPower(rightPower / 3);
        }
        
        if (gamepad2.left_trigger > .02) {
            lift1.setPower(gamepad2.left_stick_y);
            lift2.setPower(gamepad2.left_stick_x);
            lift3.setPower(gamepad2.right_stick_y);
            lift4.setPower(gamepad2.right_stick_x);
            telemetry.addData("asdfsadfasdfasdfasdfasd","1234");
        }
        
        
        //leftArm.setPower(LArmPower);
        //rightArm.setPower(RArmPower);

        
        /*data for motor*/
        // telemetry.addData("Status", "Run Time: " + runtime.toString());
        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        // // telemetry.addData("Motors", "left (%.2f), right (%.2f)", LScrewPower, RScrewPower);
        // //telemetry.addData("Motors","left (%.2f), right (%.2f)", LArmPower, RArmPower);
        // telemetry.addData("RUNS ", target);
        //  r = lColor.red();
        //  b = lColor.blue();
        //  g = lColor.green();
        //  alpha = lColor.alpha();
        //  lColor.enableLed(true);
        //  dist = sensorColorRange.getDistance(DistanceUnit.CM);
        
        //  telemetry.addData("red (%)", r);
        //  telemetry.addData("blue (%)", b);
        //  telemetry.addData("green (%)", g);
        //  telemetry.addData("alpha (%)", alpha);
        //  telemetry.addData("distance in cm", dist);
         
//        if (leftPower != 0){
//            target += 1;
//        }
//        if (rightPower != 0){
//            target += 1;
//        }
        
    }

    @Override
    public void stop() {

    }
}
