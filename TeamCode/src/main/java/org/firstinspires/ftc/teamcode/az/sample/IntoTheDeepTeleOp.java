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
    private boolean dpadDownProcessing;
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

        while (!isStopRequested()) {

//            slides = new Motor(hardwareMap, "slides");
            if (gamepad1.a) {
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

            if (gamepad1.b) {
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

            if( gamepad1.x){
                if( !buttonXProcessing){
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            buttonXProcessing = true;
                            specimenTool.move();
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
                            slides.moveToPosition(Slides.SlidesPos.RESET);
                            sleep(1000);
                            arm.setArmPos(Arm.ArmPos.RESET);
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
//            if( gamepad1.right_trigger > 0){
//                if( rightTriggerProcessing){
//                    specimenTool.extend(gamepad1.right_trigger);
//                }
//            }


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