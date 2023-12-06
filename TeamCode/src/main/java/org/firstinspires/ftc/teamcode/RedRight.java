package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public class RedRight extends LinearOpMode {
    AutoFluffy fluffy;
    List<Recognition> currentRecognitions;
    public void runOpMode(){
        initialize();
        while(!isStarted() && !isStopRequested()){
            currentRecognitions=fluffy.getRecognitions();
        }
    }
    public void initialize(){
        fluffy = new AutoFluffy(this, "Red");
    }
}
