package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Config
@TeleOp(name = "RedRight", group = "robot")
public class RedRight extends LinearOpMode {
    //BRING GRABBER UP FIRST THING
    AutoFluffy fluffy;
    public static String PATH = "Center";
    List<Recognition> currentRecognitions;
    public void runOpMode(){
        initialize();
        while(!isStarted() && !isStopRequested()){
            currentRecognitions=fluffy.getRecognitions();
            if(gamepad1.x){
                PATH = "Left";
            }
            if(gamepad1.y){
                PATH = "Center";
            }
            if(gamepad1.b){
                PATH = "Right";
            }
        }
        if(PATH == "Left"){
            deliverPurpleLeft();
        } else if(PATH == "Right"){
            deliverPurpleRight();
        }else{
            deliverPurpleCenter();
        }
    }
    public void initialize(){
        fluffy = new AutoFluffy(this, "Red");
    }

    public void deliverPurpleLeft(){

    }
    public void deliverPurpleCenter(){

    }
    public void deliverPurpleRight(){

    }
}
