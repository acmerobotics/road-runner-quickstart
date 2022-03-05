package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Carousel;


@TeleOp (group = "prototype")
@Config
public class encTest extends LinearOpMode {
    Carousel ccp = new Carousel();
    public static double endpos = 60;
    public static double maxV = 1;
    public static double maxA = 0.1;

    public static double startV = 0.5;
    public static double startA = 0;

    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, startV, startA),
            new MotionState(endpos, maxV, 0),
            maxV,
            maxA
    );
    @Override
    public void runOpMode() throws InterruptedException {
        boolean formerA = false;
        boolean formerDpadL = false;

        ccp.init(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        while(opModeIsActive()) {


            if(!formerDpadL){
                if(gamepad1.dpad_left){
                    timer.reset();
                }
            }

            if(gamepad1.dpad_left) {
                ccp.rrrun(profile, timer,-1);
                formerDpadL = true;
            }else {
                ccp.run(false);
                formerDpadL = false;
            }



            if(gamepad1.a){
                formerA = true;
            }

            if(formerA){
                if(!gamepad1.a){
                    formerA = false;
                    profile = MotionProfileGenerator.generateSimpleMotionProfile(
                            new MotionState(0, startV, startA),
                            new MotionState(endpos, maxV, 0),
                            maxV,
                            maxA
                    );


                }
            }

        }

    }
}
