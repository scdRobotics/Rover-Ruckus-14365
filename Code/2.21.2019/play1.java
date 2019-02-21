package org.firstinspires.ftc.teamcode;

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

@TeleOp (name="play1", group="Iterative Opmode")
@Disabled
public class play1 extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor lift3;
    private DcMotor lift4;
    //private DcMotor leftScrew = null;
    //private DcMotor rightScrew = null;
 //   private DcMotor leftArm = null;
 //   private DcMotor rightArm = null;
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
    
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        //leftArm = hardwareMap.get(DcMotor.class, "leftArm");
        //rightArm = hardwareMap.get(DcMotor.class, "rightArm");
        // leftScrew = hardwareMap.get(DcMotor.class, "leftScrew");
        //rightScrew = hardwareMap.get(DcMotor.class, "rightScrew");
        // sensorColorRange = hardwareMap.get(DistanceSensor.class, "lColor");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        // leftScrew.setDirection(DcMotor.Direction.FORWARD);
        // rightScrew.setDirection(DcMotor.Direction.FORWARD);
        
        
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
        double leftPower = gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;
        boolean lift = gamepad1.x;
        boolean lower = gamepad1.y;
        // double LScrewPower = gamepad2.left_stick_y;
        // double RScrewPower = gamepad2.right_stick_y;
        //double LArmPower = gamepad2.left_stick_y / 3;
        //double RArmPower = gamepad2.left_stick_y / 3;
        
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
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        if (lift = true){
            raiseLift(4.5);
        }
        if (lower = true){
            lowerLift(4.5);
        }
        // leftScrew.setPower(LScrewPower/2);
        // rightScrew.setPower(RScrewPower/2);
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
    
    public void raiseLift(double secs){
        ElapsedTime mRuntime = new ElapsedTime();
        lift1.setPower(-1);
        lift2.setPower(-1);
        lift3.setPower(-1);
        lift4.setPower(-1);
        while(mRuntime.time()< secs){
            
        }
        lift1.setPower(0);
        lift2.setPower(0);
        lift3.setPower(0);
        lift4.setPower(0);
    }
    public void lowerLift(double secs){
        ElapsedTime mRuntime = new ElapsedTime();
        lift1.setPower(1);
        lift2.setPower(1);
        lift3.setPower(1);
        lift4.setPower(1);
        while(mRuntime.time()< secs){
            
        }
        lift1.setPower(0);
        lift2.setPower(0);
        lift3.setPower(0);
        lift4.setPower(0);
    }
}
