package org.firstinspires.ftc.teamcode.New.SubSystems;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp (name= "HangPracticetelel", group= "Linear OpMode")

public class HangPracticetelel extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();

    HangPracticeJohn hangPracticeJohn = new HangPracticeJohn(hardwareMap);

    @Override
    public void runOpMode(){
        waitForStart();
        timer.reset();

        hangPracticeJohn.hangstate = HangPracticeJohn.HangState.CLOSED;
        hangPracticeJohn.update();
        while (timer.seconds() < 5){
            telemetry.addData("State", hangPracticeJohn.hangstate);
            telemetry.update();
        }
        hangPracticeJohn.hangstate = HangPracticeJohn.HangState.RELEASED;
        hangPracticeJohn.update();

        telemetry.addData("State", hangPracticeJohn.hangstate);
        telemetry.update();
    }
}
