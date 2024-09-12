package org.firstinspires.ftc.teamcode.New.Opmodes.Testing.Claw;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.New.SubSystems.Claw;

@Disabled
@TeleOp(name = "Teleop", group = "Linear OpMode")
public class ClawOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Claw claws = new Claw(hardwareMap);


        waitForStart();

        claws.timer.reset();
        while(opModeIsActive()){

            claws.update();

            telemetry.addData("Claw", claws.leftClaw.getPosition());
            telemetry.update();


        }


    }



}
