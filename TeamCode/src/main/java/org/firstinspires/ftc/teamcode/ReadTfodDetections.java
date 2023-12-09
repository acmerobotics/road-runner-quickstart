package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name= "ReadTfodDetections", group = "testing")
public class ReadTfodDetections extends LinearOpMode {
    AutoFluffy autoFluffy;
    public void runOpMode(){
        initialize();
        waitForStart();

    }

    public void initialize(){
        autoFluffy = new AutoFluffy(this, "Red");
    }
}
