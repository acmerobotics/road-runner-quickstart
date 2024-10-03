package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java;
import static org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java.ColorSensor.States.Left;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Limit Switch Op Mode", group = "Switch")
public class LimitSwitchOpMode extends LinearOpMode {


    LimitSwitch LimitSwitchSubsystem;




    @Override
    public void runOpMode() throws InterruptedException {
        LimitSwitchSubsystem = new LimitSwitch(hardwareMap.get(DigitalChannel.class, "Switch"),0);




        while (opModeIsActive()) {
            LimitSwitchSubsystem.update();




            switch (LimitSwitchSubsystem.state) {
                case RELEASED:
                    telemetry.addData("Limit Switch", "Released");
                    break;
                case PRESSED:
                    telemetry.addData("Limit Switch","Pressed");
                    break;
            }




            telemetry.update();
        }
    }
}


