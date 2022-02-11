package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class MeccRobot extends Mechanism{
    private SampleMecanumDrive drive;
    private Acquirer acquirer = new Acquirer();
    private Carousel carousel = new Carousel();
    private LiftScoringV2 scoringV2 = new LiftScoringV2();
    private FreightSensor blockSense = new FreightSensor();
    private SenseHub senseHub = new SenseHub();

    private boolean formerB = false;
    private boolean formerA = false;
    private boolean formerX = false;
    private boolean formerY = false;

    private boolean formerLeftBumper = false;
    private boolean formerRightBumper = false;


    Telemetry telemetry;

    public void init(HardwareMap hwMap){
        drive = new SampleMecanumDrive(hwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blockSense.init(hwMap);
        acquirer.init(hwMap);
        carousel.init(hwMap);
        scoringV2.init(hwMap);
    }

    public void init(HardwareMap hwmap, Telemetry telemetry){
        init(hwmap);
        this.telemetry = telemetry;
    }

    public void run(Gamepad gamepad){
        drive(gamepad);
        acquirerControls(gamepad);
        //carouselRun(gamepad);
        lift(gamepad);
        colorRumble(gamepad);
        //ducks
        runA(gamepad);
        runB(gamepad);
        telemetry.addData("has freight",blockSense.hasFreight());
        scoringV2.update();
        telemetry.update();
    }

    public void drive(Gamepad gamepad){
        Pose2d controls = new Pose2d(
                //Going to test if maybe negative)
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x
        );
        telemetry.addData("Left_stick_y",gamepad.left_stick_y);
        telemetry.addData("Left_stick_X",gamepad.left_stick_x);
        telemetry.addData("right_stick_x",gamepad.right_stick_x);


        if(!drive.isBusy()) drive.setWeightedDrivePower(controls);
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
        if(gamepad.dpad_left) carousel.run(false,true);
        else if (gamepad.dpad_right) carousel.run(true,false);
        else carousel.run(false,false);
    }

    public void runA(Gamepad gamepad){
        if(gamepad.a){
            formerA = true;
        }
        if(formerA){
            if(!gamepad.a){
                carousel.autoRun(-1);
                formerA = false;
            }
        }
    }
    public void runB(Gamepad gamepad){
        if(gamepad.b){
            formerB = true;
        }
        if(formerB){
            if(!gamepad.b){
                carousel.autoRun(1);
                formerB = false;
            }
        }
    }

    public void colorRumble(Gamepad gamepad) {
        if(blockSense.hasFreight() && scoringV2.getMovementState()=="DETRACT") {
            gamepad.rumble(50, 50, 50);
        }
        else if(senseHub.inRange() && scoringV2.getMovementState()=="EXTEND"){
            gamepad.rumble(100, 100, 50);
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

        if(gamepad.y){
            formerY = true;
        }

        if(formerY){
            if(!gamepad.y){
                scoringV2.release();

                formerY = false;
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

        telemetry.addData("liftpos: ", scoringV2.getPos());
        telemetry.addData("targetlift: ", scoringV2.getTargetPos());
        telemetry.addData("REAL Lift Movement state",scoringV2.getMovementState());
        telemetry.addData("COLOR SENSOR OUTPUT", blockSense.hasFreight());

    }


}
