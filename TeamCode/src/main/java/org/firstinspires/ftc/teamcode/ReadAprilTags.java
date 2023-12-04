package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name="ReadAprilTags", group = "testing")


public class ReadAprilTags extends LinearOpMode {
  AutoFluffy autoFluffy;
    public void runOpMode(){
      initialize();
      waitForStart();
      AprilTagDetection detection=assignID("Left", "Red");
      telemetryDetection (detection);
    }
    public void initialize(){
        autoFluffy=new AutoFluffy(this);
    }


    AprilTagDetection assignID (String propLocation, String side){
      int idNum=0;

      if (side== "Blue"){
        if (propLocation=="Left"){
          idNum=1;
        }else if (propLocation=="Center"){
          idNum = 2;
        }else if (propLocation=="Right"){
          idNum= 3;
        }
      }else if (side== "Red"){
        if (propLocation=="Left"){
          idNum= 4;
        }else if (propLocation=="Center"){
          idNum=5;
        }else if (propLocation=="Right"){
          idNum=6;
        }
      }
      List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = autoFluffy.findDetections();
      for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections){
        if (detection.id == idNum){
          return detection;
        }
      }
      return null;
    }

    public void telemetryDetection (AprilTagDetection detection){
      if (detection.metadata!= null){
        telemetry.addData("ID: ", detection.id);
        telemetry.addData("Range(Distance from board): ", detection.ftcPose.range);
        telemetry.addData("Yaw: ", detection.ftcPose.yaw);
        telemetry.addData("Bearing: ", detection.ftcPose.bearing);
      }
    }
}


