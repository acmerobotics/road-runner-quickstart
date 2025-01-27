package org.firstinspires.ftc.teamcode.onbotjava;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeamGamepad_AS extends RobotComponent_AS {

 public Gamepad gamepad;
 public String gamepadName;
 public TeamGamepad_AS(Robot2024_AS _robot, String gamepadName, Gamepad gamepad) {
   super (_robot);
   this.gamepad = gamepad;
   this.gamepadName = gamepadName;
 }
 
public void loop() {
  
}


  public String getControlTelemetry(){
        StringBuilder buttons = new StringBuilder();
        if (gamepad.a) buttons.append("A ");
        if (gamepad.b) buttons.append("B ");
        if (gamepad.x) buttons.append("X ");
        if (gamepad.y) buttons.append("Y ");
        if (gamepad.dpad_up) buttons.append("dU ");
        if (gamepad.dpad_down) buttons.append("dD ");
        if (gamepad.dpad_left) buttons.append("dL ");
        if (gamepad.dpad_right) buttons.append("dR ");
        if (gamepad.left_bumper) buttons.append("LB ");
        if (gamepad.right_bumper) buttons.append("RB ");
        
        return String.format("L_J=%.1f/%.1f, R_J=%.1f/%.1f, T: %.1f/%.1f %s",
            gamepad.left_stick_x, gamepad.left_stick_y,
            gamepad.right_stick_x, gamepad.right_stick_y,
            gamepad.left_trigger, gamepad.right_trigger,
            buttons);
    }
  
}

