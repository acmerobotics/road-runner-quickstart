package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class MeccRobot extends Mechanism{
    private boolean debug = true;


    private SampleMecanumDrive drive;
    private Acquirer acquirer = new Acquirer();

    private Carousel carousel = new Carousel();
    public static double maxV = 1;
    public static double maxA = 0.1;
    public static double startV = 0.5;
    public static double startA = 0;

    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, startV, startA),
            new MotionState(60, maxV, 0),
            maxV,
            maxA
    );

    ElapsedTime timer;
    private static int cDir = -1;


    private LiftScoringV2 scoringV2 = new LiftScoringV2();
    private FreightSensor blockSense = new FreightSensor();
    private SenseHub senseHub = new SenseHub();

    private boolean formerB = false;
    private boolean formerA = false;
    private boolean formerX = false;
    private boolean formerY = false;
    private boolean formerDpadL = false;
    private boolean formerDpadR = false;

    private boolean formerLeftBumper = false;
    private boolean formerRightBumper = false;

    private boolean formerLeftStick = false;

    int left_stick_inverted =  1;
    private boolean motionProfiling = false;

    Telemetry telemetry;

    public void init(HardwareMap hwMap){
        drive = new SampleMecanumDrive(hwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blockSense.init(hwMap);
        acquirer.init(hwMap);
        carousel.init(hwMap);
        scoringV2.init(hwMap);
        senseHub.init(hwMap);
    }

    public void init(HardwareMap hwmap, Telemetry telemetry){
        init(hwmap);
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hwmap, Telemetry telemetry, ElapsedTime timer){
        init(hwmap);
        this.telemetry = telemetry;
        this.timer = timer;
    }

    public void run(Gamepad gamepad){
        drive(gamepad);
        acquirerControls(gamepad);
        lift(gamepad);
        //colorRumble(gamepad);
        //ducks
        if(motionProfiling){
            mpCR(gamepad);
        }
        else{
            carouselRun(gamepad);
        }
        if(debug){
            telemetry.addData("has freight",blockSense.hasFreight());
            scoringV2.update();
            telemetry.update();
        }

    }

    public void drive(Gamepad gamepad){

        if(gamepad.left_stick_button){
            formerLeftStick = true;
        }

        if(formerLeftStick){
            if(!gamepad.left_stick_button){
                formerLeftStick = false;

                left_stick_inverted *= -1;

            }
        }

        Pose2d controls = new Pose2d(
                //Going to test if maybe negative)
                left_stick_inverted*gamepad.left_stick_y,
                left_stick_inverted*gamepad.left_stick_x,
                -gamepad.right_stick_x 
        );
        if(debug){
            telemetry.addData("Left_stick_y",gamepad.left_stick_y);
            telemetry.addData("Left_stick_X",gamepad.left_stick_x);
            telemetry.addData("right_stick_x",gamepad.right_stick_x);
        }

        drive.setWeightedDrivePower(controls);
    }

    public void acquirerControls(Gamepad gamepad){
        acquirerRun(gamepad.right_trigger,gamepad.left_trigger);
    }

    public void acquirerRun(double intake, double outake){
        boolean outaking = outake > 0.5;
        boolean intaking = intake > 0.5;

        if(intaking && blockSense.hasFreight()){
            outaking = true;
            intaking = false;
        }
        acquirer.run(outaking,intaking);
    }

    public void carouselRun(Gamepad gamepad){
        if(gamepad.dpad_up) carousel.run(false,true);
        else if (gamepad.dpad_down) carousel.run(true,false);
        else carousel.run(false,false);
    }


    public void colorRumble(Gamepad gamepad) {
        if(blockSense.hasFreight() && scoringV2.getMovementState()=="DETRACT") {
            gamepad.rumble(50, 50, 50);
        }
//        else if(senseHub.inRange() && scoringV2.getMovementState()=="EXTEND"){
//            gamepad.rumble(100, 100, 50);
//        }

        if(debug){
            telemetry.addData("Distance", senseHub.distance());
            telemetry.addData("InRange", senseHub.inRange());
        }


    }
    public void lift(Gamepad gamepad){
        //lift code here
        if(gamepad.left_bumper){
            formerLeftBumper = true;
        }

        if(formerLeftBumper){
            if(!gamepad.left_bumper){
                scoringV2.toggle("highgoal");
                formerLeftBumper = false;
            }
        }

        if(gamepad.a){
            formerA = true;
        }

        if(formerA & !gamepad.a){
            formerA = false;

            scoringV2.toggle("midgoal");
        }

        if(gamepad.x){
            formerX = true;
        }

        if(formerX){
            if(!gamepad.x){
                formerX = false;

                scoringV2.toggle("lowgoal");

            }
        }



        if(gamepad.right_bumper){
            formerRightBumper = true;
        }

        if(formerRightBumper){
            if(!gamepad.right_bumper){
                scoringV2.release();

                formerRightBumper = false;
            }
        }

//        if(gamepad.right_bumper){
//            formerRightBumper = true;
//        }
//
//        if(formerRightBumper){
//            if(!gamepad.right_bumper){
//                scoringV2.lower();
//
//                formerRightBumper = false;
//            }
//        }




        //scoring.run((int)lift.getCurrentPosition() == 3);
        if(debug){
            telemetry.addData("liftpos: ", scoringV2.getPos());
            telemetry.addData("targetlift: ", scoringV2.getTargetPos());
            telemetry.addData("REAL Lift Movement state",scoringV2.getMovementState());
            telemetry.addData("COLOR SENSOR OUTPUT", blockSense.hasFreight());
        }


    }

    public void mpCR(Gamepad gamepad1){
        if(!formerDpadL){
            if(gamepad1.dpad_left){
                timer.reset();
            }
        }

        if(gamepad1.dpad_left) {
            carousel.rrrun(profile, timer,cDir);
            formerDpadL = true;
        }else {
            carousel.run(false);
            formerDpadL = false;
        }

        if(gamepad1.dpad_right){
            formerDpadR = true;
        }

        if(formerDpadR){
            if(!gamepad1.dpad_right){
                formerDpadR = false;

                cDir *= -1;
            }
        }

    }


}
