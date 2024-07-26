package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Teleop.PreviousTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "AirPlane")

@Disabled
public class ZAirPlane extends LinearOpMode {

    private Servo air;

    @Override
    public void runOpMode() {
        ElapsedTime Run_Time;


        air = hardwareMap.get(Servo.class, "air");

        waitForStart();
        Run_Time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        // Startup Positions
        air.scaleRange(0, 1);
        while (opModeIsActive()) {
            if (gamepad2.right_bumper && gamepad2.left_bumper) {
                air.setPosition(0);
            }
            else if(gamepad2.right_trigger > .5 && gamepad2.left_trigger > .5) {
                air.setPosition(.5);
            }
        }

    }
}