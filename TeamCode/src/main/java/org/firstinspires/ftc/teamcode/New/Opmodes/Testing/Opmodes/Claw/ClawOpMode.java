package org.firstinspires.ftc.teamcode.New.Opmodes.Testing.Opmodes.Claw;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.New.SubSystems.Bromine.Java.Claws;


@TeleOp(name = "Claws", group = "Linear OpMode")
public class ClawOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Claws claws = new Claws(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a){
                claws.rightClaw.state = Claws.RightClaw.States.Open;
                claws.leftClaw.state = Claws.LeftClaw.States.Open;
            }

            if(gamepad1.b){
                claws.rightClaw.state = Claws.RightClaw.States.Closed;
                claws.leftClaw.state = Claws.LeftClaw.States.Open;
            }

            if(gamepad1.x){
                claws.rightClaw.state = Claws.RightClaw.States.Closed;
                claws.leftClaw.state = Claws.LeftClaw.States.Closed;
            }

            if(gamepad1.y){
                claws.rightClaw.state = Claws.RightClaw.States.Open;
                claws.leftClaw.state = Claws.LeftClaw.States.Closed;
            }

            claws.update();

            telemetry.update();


        }


    }


}
