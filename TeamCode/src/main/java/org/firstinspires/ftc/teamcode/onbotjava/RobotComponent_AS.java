package org.firstinspires.ftc.teamcode.onbotjava;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;


public abstract class RobotComponent_AS {
  
  Robot2024_AS robot;
  LinearOpMode opmode;
  HardwareMap hardwareMap;
  
  
  RobotComponent_AS(Robot2024_AS _robot){
    robot = _robot;
    opmode = robot.opmode;
    hardwareMap = opmode.hardwareMap;
    robot.registerComponent(this);
  }
  
  abstract public void loop();
  
  public void doTelemetry(Telemetry telemetry){
    
  }
  
  public void teleopLoop(Gamepad gamepad1, Gamepad gamepad2){
    
  }
  
  
}

