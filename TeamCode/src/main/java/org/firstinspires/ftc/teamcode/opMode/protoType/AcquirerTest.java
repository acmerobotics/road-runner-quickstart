package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Acquirer;

//Code for testing the acquirer with the addition of the Drivetrain.
@TeleOp(name="AcquirerTest",group="TeleOp")
public class AcquirerTest extends LinearOpMode {
    private Acquirer acquirer = new Acquirer();


    @Override
    public void runOpMode() throws InterruptedException{
        acquirer.init(hardwareMap);
        boolean formerA = false;
        boolean formerX = false;

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
//why doesnt my code work
        while(opModeIsActive()){
            acquirer.run(gamepad1.left_trigger > 0.2, gamepad1.right_trigger > 0.2);
            telemetry.addData("Left_trigger",gamepad1.left_trigger);
            telemetry.addData("Right_trigger",gamepad1.right_trigger);
            telemetry.update();


        }
    }

}