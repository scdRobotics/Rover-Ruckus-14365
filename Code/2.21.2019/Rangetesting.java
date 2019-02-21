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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
@Autonomous(name="RangeTesting", group="Linear Opmode")
@Disabled
public class Rangetesting extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    
    private DcMotor lift = null;
    private Servo liftPin;
   
    private Servo rightSampleArm;
    private Servo leftSampleArm;
    private Gyroscope imu;
    private Blinker expansion_Hub_2;
    //The two distance sensors on the sampling arms
    private DistanceSensor lDistance;
    private DistanceSensor rDistance;
    //The two Color sensors on the sampling arms
    private ColorSensor lColor;
    private ColorSensor rColor;
    
    
    private DistanceSensor rearLeftDistance;
    private DistanceSensor rearRightDistance;
    
    private String adjustMotor;
    private int adjustAmount;
    
    

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        lift = hardwareMap.get(DcMotor.class, "lift");
        //lift1 = hardwareMap.get(DcMotor.class, "lift1");
        //lift2 = hardwareMap.get(DcMotor.class, "lift2");
        //lift1.setDirection(DcMotor.Direction.FORWARD);
        //lift2.setDirection(DcMotor.Direction.FORWARD);

        //lift3 = hardwareMap.get(DcMotor.class, "lift3");
        //lift4 = hardwareMap.get(DcMotor.class, "lift4");
        //lift3.setDirection(DcMotor.Direction.FORWARD);
        //lift4.setDirection(DcMotor.Direction.FORWARD);

        //rightSampleArm = hardwareMap.get(Servo.class,"rightSampleArm");
        //leftSampleArm = hardwareMap.get(Servo.class,"leftSampleArm");
        liftPin = hardwareMap.get(Servo.class,"liftPin");
        
        lColor = hardwareMap.colorSensor.get("lColor");
        lDistance = hardwareMap.get(DistanceSensor.class, "lColor");
        lColor.enableLed(true);
        
        rColor = hardwareMap.colorSensor.get("rColor");
        rDistance = hardwareMap.get(DistanceSensor.class, "rColor");
        rColor.enableLed(true);
        
        rearLeftDistance = hardwareMap.get(DistanceSensor.class, "BackLRange");
        rearRightDistance = hardwareMap.get(DistanceSensor.class, "BackRRange");
        
        //attachArm = hardwareMap.get(CRServo.class,"attachArm");
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        runtime.reset();
        
        while(opModeIsActive()){
            DistanceTelemetry(rearLeftDistance, rearRightDistance);
        }

    }

    
    public void DistanceTelemetry(DistanceSensor leftSensor, DistanceSensor rightSensor) {
    //***  This routine is our algorithm to determine if the color sensor in
    //***  sensing the gold in the left, right or center position
        double lDist=0;
        double rDist=0;

        lDist = leftSensor.getDistance(DistanceUnit.CM);
        rDist = rightSensor.getDistance(DistanceUnit.CM);
               //** This is a very basic algorithm to determine "gold"
        
        telemetry.addData("left distance (%)", lDist);
        telemetry.addData("right Distance (%)", rDist);
        

        //telemetry.addData("alpha (%)", alpha);
        telemetry.update();

    }
    
}
