package org.firstinspires.ftc.teamcode.New.Opmodes.Teleop
//todo
//import com.acmerobotics.roadrunner.Pose2d
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import com.qualcomm.robotcore.hardware.Gamepad
//import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Actions
//import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.Claw
//import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.Drive
//import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.Shoulder
//import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.TeleLocalizer
//import org.firstinspires.ftc.teamcode.Previous.Kotlin_Bromine_Arya.Auto.Localizer.Localizer
//import kotlin.math.abs
//
//@TeleOp(name = "Teleop", group = "Linear OpMode")
//class Teleop : LinearOpMode() {
//    //gamepad objects
//    private var currentGamepad1: Gamepad = Gamepad()
//    private var currentGamepad2: Gamepad = Gamepad()
//    private var previousGamepad1: Gamepad = Gamepad()
//    private var previousGamepad2: Gamepad = Gamepad()
//
//
//    override fun runOpMode() {
//        val actions = Actions(hardwareMap)
//        val localizer = Localizer(hardwareMap, Pose2d(0.0,0.0,0.0))
//        val drive = Drive(hardwareMap,localizer)
//
//        Actions.isAuto = false
//
//        val startOfMatch =
//            abs(gamepad1.right_stick_x) > 0 || abs(gamepad1.left_stick_x) > 0 || abs(gamepad1.left_stick_y) > 0
//        var gamepadInput: Array<Float>
//
//        waitForStart()
//        while (opModeIsActive()) {
//            actions.loop()
//            //localizer.updateHeading()
//
//            gamepadInput = arrayOf(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x)
//            drive.drive(gamepadInput)
//
//            //previousGamepad1.copy(currentGamepad1)
//            //previousGamepad2.copy(currentGamepad2)
//            //currentGamepad1.copy(gamepad1)
//            //currentGamepad2.copy(gamepad2)
//            //TODO(FIX GAMEPAD)
//
//            if (gamepad1.y) {
//                actions.launchAirPlane()
//            }
//
//            //            Camera.visionPortal.resumeLiveView()
//            //            Camera.visionPortal.resumeStreaming()
//
//            //timer.startClock(startOfMatch)
//
//            //Gamepad1
//
//            //            //AutoDrive
//            //            if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
//            //                Camera.DetectAprilTag()
//            //            }
//
//
//            if (gamepad1.x) {
//               // localizer.resetHeading()
//            }
//
//            //Gamepad2
//
//            //Shoulder
//            if (gamepad2.y) {
//                actions.raiseArm()
//            }
//            if (gamepad2.a) {
//                actions.lowerArm()
//            }
//
//
//            if (gamepad2.x) {
//                actions.autoGrabBoth()
//            }
//            if (gamepad2.dpad_up && !previousGamepad2.dpad_up && Shoulder.shoulderLevel < 5) {
//                Shoulder.shoulderLevel++
//            } else if (gamepad2.dpad_down && !previousGamepad2.dpad_down && Shoulder.shoulderLevel > 1) {
//                Shoulder.shoulderLevel--
//            }
//
//            //Claw
//            if (gamepad2.right_trigger > .1) {
//                actions.manualGrab()
//            }
//            if (gamepad2.left_trigger > .1) {
//                actions.release()
//            }
//            if (gamepad2.b) {
//                Claw.clawChoice = Claw.ClawChoice.Both
//            } else if (gamepad2.left_bumper && !previousGamepad2.left_bumper) {
//                Claw.clawChoice = Claw.ClawChoice.Left
//            } else if (gamepad2.right_bumper && !previousGamepad2.right_bumper) {
//                Claw.clawChoice = Claw.ClawChoice.Right
//            }
//
//        }
//
//
//    }
//
//
//}
