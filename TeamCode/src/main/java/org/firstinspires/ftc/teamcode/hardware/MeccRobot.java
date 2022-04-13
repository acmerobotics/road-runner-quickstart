package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.util.BooleanManager;

public class MeccRobot extends Mechanism{
    private boolean debug = false;

    //Mechanisms Utilized
    private SampleMecanumDrive drive;
    int left_stick_inverted =  1;
    private boolean soft_dump = false;
    private Acquirer acquirer = new Acquirer();

    //values for carousel
    ElapsedTime timer;
    private boolean motionProfiling = true;
    private static int cDir = 1;
    private boolean formerDpadL = false;
    private boolean formerDpadR = false;
    private Carousel carousel = new Carousel();

    private LiftScoringV2 scoringV2 = new LiftScoringV2();

    private FreightSensor blockSense = new FreightSensor();
    private RetractableOdoSys odoSys = new RetractableOdoSys();
    private Capper capper = new Capper();
//
//    private SenseHub senseHub = new SenseHub();

    //BooleanManager for button presses
    BooleanManager leftStickManager = new BooleanManager(new Runnable() {
        @Override
        public void run() {
            left_stick_inverted *= -1;
        }
    });

    BooleanManager leftBumperManager = new BooleanManager(()->{
        scoringV2.toggle("highgoal");

    });

    BooleanManager xButtonManager = new BooleanManager(()->{
        scoringV2.toggle("lowgoal");

    });

    BooleanManager bButtonManager = new BooleanManager(()->{

        scoringV2.toggle("midgoalTele");
    });

    BooleanManager aButtonManager = new BooleanManager(()->{
        odoSys.toggle();
    });

    BooleanManager rightStickManager = new BooleanManager(new Runnable() {
        @Override
        public void run() {
            soft_dump = !soft_dump;
        }
    });


    BooleanManager rightBumperManager = new BooleanManager(()->{
        if(!soft_dump) scoringV2.releaseHard();
        else scoringV2.releaseSoft();
        
    });

    BooleanManager yButtonManager = new BooleanManager(()->{
        scoringV2.bottom();
    });


    BooleanManager rightBumper2 = new BooleanManager(new Runnable() {
        @Override
        public void run() {
            scoringV2.toggle("highgoal");
        }
    });

    BooleanManager aButton2 = new BooleanManager(new Runnable() {
        @Override
        public void run() {
            if(scoringV2.goalReach.equals("highgoal")) capper.release();
        }
    });
    BooleanManager bButton2 = new BooleanManager(new Runnable() {
        @Override
        public void run() {
            capper.raise();
        }
    });
    BooleanManager xButton2 = new BooleanManager(new Runnable() {
        @Override
        public void run() {
            if(scoringV2.getMovementState().equals("DETRACT")) capper.reset();
        }
    });
    BooleanManager yButton2 = new BooleanManager(new Runnable() {
        @Override
        public void run() {
            if(scoringV2.getMovementState().equals("DETRACT")) capper.grabCap();
        }
    });

    BooleanManager leftBumper2 = new BooleanManager(new Runnable() {
        @Override
        public void run() {
            scoringV2.toggle("midgoal");
        }
    });
    Telemetry telemetry;

    public void init(HardwareMap hwMap){
        drive = new SampleMecanumDrive(hwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        acquirer.init(hwMap);
        carousel.init(hwMap);
        scoringV2.init(hwMap,blockSense);
        odoSys.init(hwMap);
        capper.init(hwMap);

//        senseHub.init(hwMap);
    }

    /**
     * basic initialize *CANNOT USE WITH ROADRUNNER MP CAROUSEL
     * @param hwmap
     * @param telemetry
     */
    public void init(HardwareMap hwmap, Telemetry telemetry){
        init(hwmap);
        this.telemetry = telemetry;
        odoSys.toggle();

    }

    /**
     * initializes with timer -> for use with Roadrunner MP Carousel
     * @param hwmap
     * @param telemetry
     * @param timer
     */
    public void init(HardwareMap hwmap, Telemetry telemetry, ElapsedTime timer){
        init(hwmap);
        this.telemetry = telemetry;
        this.timer = timer;
        odoSys.toggle();
    }

    /**
     * run in teleop mode
     * @param gamepad1
     */
    public void run(Gamepad gamepad1, Gamepad gamepad2){
        drive(gamepad1);
        acquirerControls(gamepad1);
        lift(gamepad1);
        capperControl(gamepad2);
        //colorRumble(gamepad);

        //ducks
        if(motionProfiling){
            mpCR(gamepad1);
        }
        else{
            carouselRun(gamepad1);
        }
        if(debug){
            //telemetry.addData("has freight",blockSense.hasFreight());
            telemetry.update();
        }
        scoringV2.update();

    }

    /**
     * drive method -> controls DT
     * @param gamepad
     */
    public void drive(Gamepad gamepad){
        aButtonManager.update(gamepad.a);
        leftStickManager.update(gamepad.left_stick_button);

        //left stick inverted inverts controls if equal to 1;
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

    /**
     * controls acquirer
     * @param gamepad
     */
    public void acquirerControls(Gamepad gamepad){
        acquirerRun(gamepad.right_trigger,gamepad.left_trigger);
    }

    /**
     * runs acquirer based on inputes
     * @param intake double dictating intake movement
     * @param outake double dictating outake movement
     */
    public void acquirerRun(double intake, double outake){
        boolean outaking = outake > 0.5;
        boolean intaking = intake > 0.5;

        if(intaking && scoringV2.raisingStatus()){
            outaking = true;
            intaking = false;
        }
        acquirer.run(outaking,intaking);
    }

    /**
     * carousel runs with constant speed method
     * @param gamepad
     */
    public void carouselRun(Gamepad gamepad){
        if(gamepad.dpad_up) carousel.run(false,true);
        else if (gamepad.dpad_down) carousel.run(true,false);
        else carousel.run(false,false);
    }

    /**
     * rumbled controller given a block is intaked
     * UNSTABLE -> Lags code upon implementation due to constant rumble request. Requires fixing
     * @param gamepad
     */

//    @Deprecated
//    public void colorRumble(Gamepad gamepad) {
//        if(blockSense.hasFreight() && scoringV2.getMovementState()=="DETRACT") {
//            gamepad.rumble(50, 50, 50);
//        }
////        else if(senseHub.inRange() && scoringV2.getMovementState()=="EXTEND"){
////            gamepad.rumble(100, 100, 50);
////        }
//
//        if(debug){
//            telemetry.addData("Distance", senseHub.distance());
//            telemetry.addData("InRange", senseHub.inRange());
//        }
//
//
//    }

    /**
     * method controller scoring mechanism controls (not just the lift)
     * @param gamepad
     */
    public void lift(Gamepad gamepad){
        //lift code here
        leftBumperManager.update(gamepad.left_bumper);

        xButtonManager.update(gamepad.x);

        rightBumperManager.update(gamepad.right_bumper);

        yButtonManager.update(gamepad.y);

        bButtonManager.update(gamepad.b);

        rightStickManager.update(gamepad.right_stick_button);

        if(debug){
            telemetry.addData("softdump?", soft_dump);
            telemetry.addData("liftpos: ", scoringV2.getPos());
            telemetry.addData("targetlift: ", scoringV2.getTargetPos());
            telemetry.addData("REAL Lift Movement state",scoringV2.getMovementState());
            telemetry.addData("COLOR SENSOR OUTPUT", blockSense.hasFreight());
        }


    }

    /**
     * carousel RoadRunner motion profiling wrapper
     * @param gamepad1 gamepad input
     */
    public void mpCR(Gamepad gamepad1){
        boolean input = gamepad1.dpad_left ^ gamepad1.dpad_right;
        if(gamepad1.dpad_right) {
            cDir = -1;
        }else {
            cDir = 1;
        }
        if(!formerDpadL){
            if(input){
                timer.reset();
            }
        }

        if(input) {
            if(timer.seconds() <= 1) {
                carousel.rrrun(timer, 1);
            }else {
                if(cDir == 1) {
                    carousel.runmax(true, false);
                }else {
                    carousel.runmax(false, true);
                }
            }
            formerDpadL = true;
        }

        else {
            carousel.run(false);
            formerDpadL = false;
        }
    }

    public void capperControl(Gamepad gamepad2){
        rightBumper2.update(gamepad2.right_bumper);
        aButton2.update(gamepad2.a);
        bButton2.update(gamepad2.b);
        xButton2.update(gamepad2.x);
        yButton2.update(gamepad2.y);
        leftBumper2.update(gamepad2.left_bumper);
    }


}
