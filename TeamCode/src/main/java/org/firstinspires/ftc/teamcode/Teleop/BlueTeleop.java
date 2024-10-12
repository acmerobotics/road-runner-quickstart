/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Vision.Pipeline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class BlueTeleop extends LinearOpMode {

    Pipeline vision = new Pipeline(telemetry);
    OpenCvWebcam webcam1 = null;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private enum LiftState {LIFTSTART, LIFTDEPOSIT, LIFTWALL, LIFTTOPBAR, LIFTBOTTOMBAR};
    private LiftState liftState = LiftState.LIFTSTART;

    private enum ExtendoState {EXTENDONOTHING, EXTENDORETRACT, EXTENDOSPIT};
    private ExtendoState extendoState = ExtendoState.EXTENDONOTHING;

    Pose2d StartPose1 = new Pose2d(40, 60, Math.toRadians(180));
    MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);


    public void drivetrain(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR){
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);
    }

    public void buttonpress(Extendo extendo, Intake intake, Slides slides, Claw claw) {
        double y = gamepad2.left_stick_y;
        double robotX = drive.pose.position.x;
        double robotY = drive.pose.position.y;

        switch (extendoState) {
            case EXTENDONOTHING:
                //TODO: change values to the min/max of how far extendo extends
                if (extendo.getPos() > 0 && extendo.getPos() < 10000) {
                    extendo.extendoLeft.setPower(y);
                    extendo.extendoRight.setPower(y);
                } else if (extendo.getPos() > 10000) {
                    extendo.extendoLeft.setPower(-0.6);
                    extendo.extendoRight.setPower(-0.6);
                } else if (extendo.getPos() < 0) {
                    extendo.extendoLeft.setPower(0.6);
                    extendo.extendoRight.setPower(0.6);
                }
                //TODO: change value to how far until its a bit extended out in front of the robot
                if (extendo.getPos() > 200) {
                    if (!Intake.flipped) {
                        runningActions.add(intake.flip());
                    }
                    if (vision.colorDetected().equals("Blue") || vision.colorDetected().equals("Yellow")) {
                        runningActions.add(new SequentialAction(
                                intake.flop(),
                                extendo.retract()
                        ));
                        extendoState = ExtendoState.EXTENDORETRACT;
                    } else if (vision.colorDetected().equals("Red")) {
                        intake.intakeMotor.setPower(-1);
                    } else {
                        intake.intakeMotor.setPower(1);
                    }
                } else {
                    if (Intake.flipped) {
                        runningActions.add(intake.flop());
                    }
                }
                break;
            case EXTENDORETRACT:
                //TODO: Set to the transfer position
                if (extendo.getPos() < 0) {
                    intake.intakeMotor.setPower(-0.2);
                    extendoState = ExtendoState.EXTENDOSPIT;
                }
                break;
            case EXTENDOSPIT:
                if (!vision.colorDetected().equals("Blue") && !vision.colorDetected().equals("Yellow")) {
                    intake.intakeMotor.setPower(0);
                    runningActions.add(claw.close());
                    extendoState = ExtendoState.EXTENDONOTHING;
                }
                break;
        }



        switch (liftState) {
            case LIFTSTART:
                if (gamepad2.x) {
                    if (gamepad2.left_trigger < 0.9) {
                        runningActions.add(new SequentialAction(
                                slides.slideTopBasket(),
                                claw.flip()
                        ));
                    } else {
                        runningActions.add(new SequentialAction(
                                slides.slideBottomBasket(),
                                claw.flip()
                        ));
                    }
                    liftState = LiftState.LIFTDEPOSIT;
                }
                if (gamepad2.y) {
                    runningActions.add(new SequentialAction(
                            claw.open(),
                            slides.slideWallLevel(),
                            claw.flip()
                    ));
                    liftState = LiftState.LIFTWALL;
                }
                break;
            case LIFTDEPOSIT:
                if (gamepad2.x) {
                    runningActions.add(new SequentialAction(
                            claw.open(),
                            claw.flop(),
                            slides.retract()
                    ));
                    liftState = LiftState.LIFTSTART;
                }
                if (gamepad2.b) {
                    runningActions.add(new SequentialAction(
                            claw.flop(),
                            slides.retract()
                    ));
                    liftState = LiftState.LIFTSTART;
                }
                break;
            case LIFTWALL:
                if (gamepad2.y) {
                    if (gamepad2.left_trigger < 0.9) {
                        runningActions.add(new SequentialAction(
                                claw.close(),
                                slides.slideTopBar()
                        ));
                        liftState = LiftState.LIFTTOPBAR;
                    } else {
                        runningActions.add(new SequentialAction(
                                claw.close(),
                                slides.slideBottomBar()
                        ));
                        liftState = LiftState.LIFTBOTTOMBAR;
                    }
                }
                break;
            case LIFTTOPBAR:
                if (gamepad2.y) {
                    runningActions.add(new SequentialAction(
                            slides.slideBottomBar(),
                            new SleepAction(1.2),
                            slides.retract()
                    ));
                    liftState = LiftState.LIFTSTART;
                }
                break;
            case LIFTBOTTOMBAR:
                if (gamepad2.y) {
                    runningActions.add(slides.retract());
                    liftState = LiftState.LIFTSTART;
                }
                break;
            default:
                liftState = LiftState.LIFTSTART;
                break;
        }
    }
    @Override
    public void runOpMode() {

        Extendo extendo = new Extendo(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMoniterViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMoniterViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMoniterViewId);
        webcam1.setPipeline(new Pipeline(telemetry));
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            public void onOpened(){
                webcam1.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);

            }
            public void onError(int errorCode){

            }

        });

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            drivetrain(drive.leftFront, drive.rightFront, drive.leftBack, drive.rightBack);
            buttonpress(extendo, intake, slides, claw);

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

            telemetry.addData("intakeCamera", vision.colorDetected());
            telemetry.update();
            drive.updatePoseEstimate();
        }
    }
}
