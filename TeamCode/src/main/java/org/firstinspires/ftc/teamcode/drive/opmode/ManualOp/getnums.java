package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "CLAW WORK", group = "comp")
public class getnums extends OpMode {

    SampleMecanumDrive drive;

    private double degree = 0.0;

    private double on = 0;
    private double deci;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.gripServoB.setPosition(0);
        drive.gripServoF.setPosition(0);
        telemetry.addData("F pos", drive.gripServoF.getPosition());
        telemetry.addData("B pos", drive.gripServoB.getPosition());

    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up){
            on++;
            degree= on/20;
            deci = degree/300;
            drive.gripServoB.setPosition(deci);
            drive.gripServoF.setPosition(deci);
            telemetry.addData("F pos", drive.gripServoF.getPosition());
            telemetry.addData("B pos", drive.gripServoB.getPosition());
            telemetry.addData("degree",degree);
            telemetry.addData("on",on);
            telemetry.addData("deci",deci);
            telemetry.update();
        }
        if (gamepad2.dpad_down){
            on--;
            degree= on/20;
            deci = degree/300;
            drive.gripServoB.setPosition(deci);
            drive.gripServoF.setPosition(deci);
            telemetry.addData("F pos", drive.gripServoF.getPosition());
            telemetry.addData("B pos", drive.gripServoB.getPosition());
            telemetry.addData("degree",degree);
            telemetry.addData("on",on);
            telemetry.addData("deci",deci);
            telemetry.update();
        }
        if (gamepad2.dpad_left){
            on=0;
            degree=0;
            deci = 0;
            drive.gripServoB.setPosition(0);
            drive.gripServoF.setPosition(0);
            telemetry.addData("F pos", drive.gripServoF.getPosition());
            telemetry.addData("B pos", drive.gripServoB.getPosition());
            telemetry.addData("degree",degree);
            telemetry.addData("on",on);
            telemetry.addData("deci",deci);
            telemetry.update();
        }
        if (gamepad2.dpad_right){
            degree= 25;
            on= degree;
            deci = degree/300;
            drive.gripServoB.setPosition(deci);
            drive.gripServoF.setPosition(deci);
            telemetry.addData("F pos", drive.gripServoF.getPosition());
            telemetry.addData("B pos", drive.gripServoB.getPosition());
            telemetry.addData("degree",degree);
            telemetry.addData("on",on);
            telemetry.addData("deci",deci);
            telemetry.update();
        }
        if (gamepad2.square){
            degree= 22;
            on= degree;
            deci = degree/300;
            drive.gripServoB.setPosition(deci);
            drive.gripServoF.setPosition(deci);
            telemetry.addData("F pos", drive.gripServoF.getPosition());
            telemetry.addData("B pos", drive.gripServoB.getPosition());
            telemetry.addData("degree",degree);
            telemetry.addData("on",on);
            telemetry.addData("deci",deci);
            telemetry.update();
        }



    }
    public void words () {
        deci = degree/300;
        drive.gripServoB.setPosition(deci);
        drive.gripServoF.setPosition(deci);
        telemetry.addData("F pos", drive.gripServoF.getPosition());
        telemetry.addData("B pos", drive.gripServoB.getPosition());
        telemetry.addData("degree",degree);
        telemetry.addData("deci",deci);
        telemetry.update();

    }

}


