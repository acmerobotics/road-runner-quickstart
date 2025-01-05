package org.firstinspires.ftc.teamcode.TestArchive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;

@TeleOp
public class ResetPos extends LinearOpMode {
    @Override
    public void runOpMode() {
        Claw claw = new Claw(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Intaker intake = new Intaker(hardwareMap);

        waitForStart();
        
    }
}
