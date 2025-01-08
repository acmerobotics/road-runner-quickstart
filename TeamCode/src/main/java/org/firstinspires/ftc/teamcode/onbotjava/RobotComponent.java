package org.firstinspires.ftc.teamcode.onbotjava;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;


public abstract class RobotComponent {
  
  Robot2024 robot;
  LinearOpMode opmode;
  HardwareMap hardwareMap;
  
  
  RobotComponent(Robot2024 _robot){
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

