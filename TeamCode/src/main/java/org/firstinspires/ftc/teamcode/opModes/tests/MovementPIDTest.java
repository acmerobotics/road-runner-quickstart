//package org.firstinspires.ftc.teamcode.opModes.tests;
//
//import com.aimrobotics.aimlib.gamepad.AIMPad;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.gb.pinpoint.driver.Pose2D;
//import org.firstinspires.ftc.teamcode.subsystems.v1.Robot;
//import org.firstinspires.ftc.teamcode.subsystems.v1.ScoringSystem;
//
//@TeleOp(name = "MovementPIDTest", group = "TESTING")
//public final class MovementPIDTest extends OpMode {
//    Robot robot = new Robot(true, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
//    ScoringSystem scoringSystem = new ScoringSystem(true);
//
//    AIMPad aimPad1;
//    AIMPad aimPad2;
//
//    @Override
//    public void init() {
//        robot.init(hardwareMap);
//        aimPad1 = new AIMPad(gamepad1);
//        aimPad2 = new AIMPad(gamepad2);
//    }
//
//    @Override
//    public void loop() {
//        robot.scoringSystem.setActiveScoringState(ScoringSystem.ScoringState.AUTO_PERIOD);
//        robot.loop(aimPad1, aimPad2);
//        aimPad1.update(gamepad1);
//        aimPad2.update(gamepad2);
//
//        telemetry.addData("Previous State", aimPad1.getPreviousState());
//        telemetry.addData("Current State", aimPad1.getCurrentState());
//        robot.telemetry(telemetry);
//    }
//}
