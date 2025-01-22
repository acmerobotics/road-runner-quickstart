//package org.firstinspires.ftc.teamcode.opModes.comp.auto.qual2;
//
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.aimrobotics.aimlib.gamepad.AIMPad;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.opModes.comp.auto.supers.SupersAutoConstants;
//import org.firstinspires.ftc.teamcode.subsystems.v1.IntakeSystem;
//import org.firstinspires.ftc.teamcode.subsystems.v1.OuttakeSystem;
//import org.firstinspires.ftc.teamcode.subsystems.v1.Robot;
//
//@Autonomous(name = "Jankster", group = "AAA_COMP")
//public class jankster extends LinearOpMode {
//
//
//    Robot robot = new Robot(true, SupersAutoConstants.BLUE_STARTING_POSITION);
//
//    AIMPad aimPad1;
//    AIMPad aimPad2;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap);
//        robot.scoringSystem.specimenGrabber.grab();
//        aimPad1 = new AIMPad(gamepad1);
//        aimPad2 = new AIMPad(gamepad2);
//
//        while (!isStarted() && !isStopRequested()) {
//            robot.scoringSystem.intakeSystem.multiAxisArm.resetClosed();
//
//            robot.scoringSystem.intakeSystem.pivotDown();
//
//            robot.scoringSystem.intakeSystem.setSlidesPosition(IntakeSystem.SlidesPosition.RESET);
//            robot.scoringSystem.outtakeSystem.reset();
//            robot.scoringSystem.specimenGrabber.grab();
//
//            aimPad1.update(gamepad1);
//            aimPad2.update(gamepad2);
//
//            robot.telemetry(telemetry);
//
//            telemetry.update();
//        }
//        waitForStart();
//        while (opModeIsActive()) {
//            robot.drivebase.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.3, 0), 0));
//            sleep(2500);
//            robot.drivebase.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//            break;
//        }
//    }
//}
