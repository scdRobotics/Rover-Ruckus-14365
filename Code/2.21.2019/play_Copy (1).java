package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp (name="play_copy", group="Iterative Opmode")
@Disabled
public class play_Copy extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo leftSampleArm;
    
    @Override
    public void init() {
    telemetry.addData("Status", "Initialized");

        leftSampleArm = hardwareMap.get(Servo.class, "leftSampleArm");


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();  
    }

    @Override
    public void loop() {
        
        leftSampleArm.setPosition(.4);
         

    }

    @Override
    public void stop() {

    }
}
