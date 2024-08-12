package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.`Testing$Tuning`.Auto.Localizer

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Localizer.Localizer
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.Drive

@TeleOp(name = "RRLocalizer", group = "Linear OpMode")
class drive : LinearOpMode(){
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val drive = Localizer(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        val move = Drive(hardwareMap, drive)
        Drive.driveType = Drive.DriveType.Manual

        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            drive.update()

            move.drive(arrayOf( gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x))

            telemetry.addData("x",drive.xPos)
            telemetry.addData("y",drive.yPos)
            telemetry.addData("h",Math.toDegrees(drive.heading.toDouble()))
            telemetry.update()
        }
    }
}