package org.firstinspires.ftc.teamcode.opMode.protoType;

//Code for playing around with servos. Go on, experiment!
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RetractableOdoSys;
import org.firstinspires.ftc.teamcode.hardware.util.BooleanManager;

@Config
@TeleOp (group = "prototype")
public class OdoSysTest extends LinearOpMode {
    RetractableOdoSys odoSys = new RetractableOdoSys();
            ;
    //Servo Main: START: 0.45; END: 0.95
    //Servo Supp: START: 0.6; END: 0.1
    //Clamp Servo: START: 0.1; END: 0.6
    public void init(HardwareMap hwMap){
        odoSys.init(hwMap);

    }
    @Override
    public void runOpMode() throws InterruptedException{
        boolean formerA = false;
        boolean formerB = false;
        init(hardwareMap);
        waitForStart();
        BooleanManager bButton = new BooleanManager(()->{
            odoSys = new RetractableOdoSys();
            init(hardwareMap);
        });
        while(opModeIsActive()&& !isStopRequested()){
            //press A to toggle odoSys
            if(gamepad1.a){
                formerA = true;
            }
            if(formerA){
                if(!gamepad1.a){
                    formerA = false;
                    odoSys.toggle();

                }
            }

            bButton.update(gamepad1.b);

        }
    }
}