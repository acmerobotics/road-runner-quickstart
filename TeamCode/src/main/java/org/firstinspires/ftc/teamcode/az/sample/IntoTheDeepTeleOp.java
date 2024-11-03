package org.firstinspires.ftc.teamcode.az.sample;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IntoTheDeepTeleOp extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static final boolean FIELD_CENTRIC = false;
    SpecimenTool specimenTool = null;

    Arm arm = null;
    Slides slides = null;
    private boolean dpadUpProcessing;
    private boolean gamepad2DpadUpProcessing;
    private boolean dpadDownProcessing;
    private boolean gamepad2DpadDownProcessing;
    private boolean dpadRightProcessing;
    private boolean buttonAProcessing;
    private boolean buttonBProcessing;
    private boolean buttonXProcessing;
    private boolean buttonYProcessing;
    private boolean rightTriggerProcessing;
    private boolean leftTriggerProcessing;
    private boolean rightBumperProcessing;
    private boolean leftBumperProcessing;


    public void setup() {
        specimenTool.reset();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER
        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_435)
        );

        arm = new Arm(this);
        slides = new Slides(this);
        specimenTool = new SpecimenTool(this);


        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();

        specimenTool.specimenToolInit();

        while (!isStopRequested()) {


//            slides = new Motor(hardwareMap, "slides");
            if (gamepad1.a) { //x
               if(!buttonAProcessing ){
                   AZUtil.runInParallel(new Runnable() {
                       @Override
                       public void run() {
                           buttonAProcessing = true;
                           specimenTool.collect();
                           buttonAProcessing = false;
                       }
                   });
               }
            }

            if (gamepad1.b) { //circle
                if( !buttonBProcessing){
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            buttonBProcessing = true;
                            specimenTool.sampleDrop();
                            buttonBProcessing = false;
                        }
                    });
                }
            }

            if( gamepad1.x){ //square
                if( !buttonXProcessing){
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            buttonXProcessing = true;

                            if(arm.getCurrentPos() < 0) {
                                specimenTool.move();
                            }
                            else {
                                specimenTool.highReset();
                            }
                            buttonXProcessing = false;
                        }
                    });
                }
            }

            if(gamepad1.dpad_up){
                if( !dpadUpProcessing) {
                    dpadUpProcessing = true;
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            arm.setArmPos(Arm.ArmPos.BASKET_DROP);
                            slides.moveToPosition(Slides.SlidesPos.SPECIMEN_HANG);
                            dpadUpProcessing = false;
                        }
                    });

                }

            }


            if(gamepad1.dpad_down){
                if( !dpadDownProcessing){
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            dpadDownProcessing = true;
                            specimenTool.halfwayReset();
                            dpadDownProcessing = false;
                        }
                    });
                }

            }

            if( gamepad1.dpad_right){
                if( !dpadRightProcessing) {
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            dpadRightProcessing = true;
                            arm.setArmPos(Arm.ArmPos.BASKET_DROP);
                            slides.moveToPosition(Slides.SlidesPos.BASKET_DROP);
                            dpadRightProcessing = false;
                        }
                    });
                }

            }

            //extend the slides
            if( gamepad1.right_trigger > 0){
                //if not processing then perform this operation
                if( !rightTriggerProcessing) {
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            rightTriggerProcessing = true;
                            specimenTool.extend(gamepad1.right_trigger);
                            rightTriggerProcessing = false;
                        }
                    });
                }
            } else {

//                    specimenTool.extend(gamepad1.right_trigger);
                }

            if(gamepad1.right_bumper){
                if( !rightBumperProcessing){
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            rightBumperProcessing = true;
                            arm.moveUp();
                            rightBumperProcessing = false;
                        }
                    });
                }

            }

            if(gamepad1.left_bumper){
                if( !leftBumperProcessing){
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            leftBumperProcessing = true;
                            arm.moveDown();
                            leftBumperProcessing = false;
                        }
                    });
                }

            }

            if(gamepad2.dpad_right){
                if( !gamepad2DpadUpProcessing){
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            gamepad2DpadUpProcessing = true;
                            slides.moveUp();
                            gamepad2DpadUpProcessing = false;
                        }
                    });
                }

            }

            if(gamepad2.dpad_left){
                if( !gamepad2DpadDownProcessing){
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            gamepad2DpadDownProcessing = true;
                            slides.moveDown();
                            gamepad2DpadDownProcessing = false;
                        }
                    });
                }

            }

            if(gamepad1.y){
                if( !buttonYProcessing){
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            buttonYProcessing = true;
                            specimenTool.specimenHang();
                            buttonYProcessing = false;
                        }
                    });
                }

            }


            if (!FIELD_CENTRIC) {
                drive.driveRobotCentric(
                        -driverOp.getLeftX(),
                        -driverOp.getLeftY(),
                        -driverOp.getRightX(),
                        false
                );
            } else {
                drive.driveFieldCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY(),
                        driverOp.getRightX(),
                        imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                        false
                );
            }
            telemetry.addData("arm", arm.getCurrentPos());
            telemetry.addData("slides", slides.getCurrentPos());
            telemetry.update();
        }
    }

}