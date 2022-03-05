package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.ScoringArm;

@TeleOp (group = "prototype")
@Config
public class ScoringMechTest extends LinearOpMode {
    private ScoringArm score = new ScoringArm();
    //0.1
    //
    public static double position = 0.0;
    @Override
    public void runOpMode() throws InterruptedException{
        score.init(hardwareMap);
        boolean formerA = false;
        boolean formerB = false;
        boolean formerX = false;
        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
//why doesnt my code work
        while(opModeIsActive()){
            if (gamepad1.a){
                formerA = true;
            }

            if(formerA){
                if (!gamepad1.a){
                    //DO STUFF
                    score.goToEnd();
                    formerA = false;
                }
            }

            if (gamepad1.b){
                formerB = true;
            }

            if(formerB){
                if (!gamepad1.b){
                    //DO STUFF
                    score.goToStart();
                    formerB = false;
                }
            }

            if (gamepad1.x){
                formerX = true;
            }

            if(formerX){
                if (!gamepad1.x){
                    //DO STUFF
                    score.goTo(position);
                    formerX = false;
                }
            }

            telemetry.update();


        }
    }

}