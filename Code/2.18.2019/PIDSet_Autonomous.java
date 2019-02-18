package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Autonomous(name = "PIDSet_Autonomous", group = "")
@Disabled
public class PIDSet_Autonomous extends LinearOpMode {

    DcMotor REV403;
 
    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;


  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
    @Override
    public void runOpMode() {
        REV403 = hardwareMap.dcMotor.get("REV40-3");
    
            // Put initialization blocks here.
        waitForStart();
        
            // get a reference to the motor controller and cast it as an extended functionality controller.
            // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx REV403Ex = (DcMotorControllerEx)REV403.getController();
        
            // get the port number of our configured motor.
        int motorIndex = ((DcMotorEx)REV403Ex).getPortNumber();

            // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients pidR_U_E = REV403Ex.getPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

            // change coefficients.
 //     PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
 //     REV403Ex.setPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

            // get the PID coefficients for the RUN_TO_POSITION  modes.
        PIDCoefficients pidR_T_P = REV403Ex.getPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_TO_POSITION);

            // display info to user.
            while(opModeIsActive()) {
                telemetry.addData("Runtime", "%.03f", getRuntime());
                telemetry.addData("P,I,D (R_U_E)", "%.04f, %.04f, %.0f",
                    pidR_U_E.p, pidR_U_E.i, pidR_U_E.d);
                telemetry.addData("P,I,D (R_T_P)", "%.04f, %.04f, %.04f",
                    pidR_T_P.p, pidR_T_P.i, pidR_T_P.d);
                telemetry.update();
            }
    }
}
