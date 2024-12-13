package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotFTC", group="Linear Opmode")
public class TeleOp extends GlobalScope
{
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {

        Initialise();

        waitForStart();

        Controler();

        while (opModeIsActive())
        {
            MiscareBaza();
            SliderExtend();
            SliderBaza();
            Roteste();
            //Cleste();
            //Intake();
            //Outake();
            ActiuneAuto();
            telemetry.update();
            telemetry.addData("Stanga ", BazaStanga.getPosition());
            telemetry.addData("Dreapta ", BazaDreapta.getPosition());
            telemetry.addData("Rotire", ServoRotire.getPosition());

            telemetry.addData("GhearaOutake", ServoGhearaOutake.getPosition());
            telemetry.addData("OutakeStanga", OutakeStanga.getPosition());

            /**IntakeSus.readValue();
            IntakeJos.readValue();
            if(IntakeSus.wasJustPressed()){
                BazaDreapta.setPosition(BazaDreapta.getPosition() + 0.01);
                BazaStanga.setPosition(BazaStanga.getPosition() + 0.01);
            }
            if(IntakeJos.wasJustPressed()){
                BazaDreapta.setPosition(BazaDreapta.getPosition() - 0.01);
                BazaStanga.setPosition(BazaStanga.getPosition() - 0.01);
            }*/
        }
    }
}